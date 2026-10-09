#pragma once
#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../skill_loader.h"
#include "../ros_backend.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../trajectory/min_snap.h"
#include "../trajectory/position_controller.h"

/**
 * @brief 起飞/悬停/轨迹技能执行器：IDLE → TAKEOFF → HOVER ⇄ TRAJECTORY → LANDING 状态机
 *
 * 进自动模式即常驻（onStart 返回 RUNNING 进入 IDLE），自行拥有 CH5 的全部语义：
 *   IDLE       未解锁时什么都不写（等 arm_action）；已解锁写怠速通道；
 *              检测 CH5 上升沿(>1750) 且锁存已清 → 建立任务系(起飞点机头系) → SPOOL_UP
 *   SPOOL_UP   开环油门 S 曲线 ramp 到悬停值以下（电机起转但不起飞）；时长跑满 → TAKEOFF；CH5 拨低 → abort()
 *   TAKEOFF    rest-to-rest snap 垂直段 + 统一位置控制器；到达悬停高度(v≈0) → HOVER；CH5 拨低/超时 → LANDING
 *   HOVER      上位机 ANGLE 定高定点（水平 VINS 闭环 + 垂直高度源闭环，CH6 恒 1000=ANGLE）；
 *              触发边沿（CH6 上升沿 或 /traj_trigger 话题，按 trigger_source）→ 去抖+悬停稳定
 *              → TRAJECTORY；CH5 拨低 → LANDING
 *   TRAJECTORY 最小 snap 轨迹 + 位置控制器；到达 → 回 HOVER；CH5 拨低 → LANDING；
 *              超时 / 位置误差超限 / VINS 失效 → abort()；飞行中 CH6 状态不影响轨迹
 *   LANDING    写 CH7=2000（RTH，无 GPS 时飞控自动降落）；检测着陆 → abort()（上锁+回 IDLE）
 *
 * 控制器统一：起转=开环油门 ramp（唯一例外）；起飞/悬停/轨迹=PositionController(差分平坦 PD)。
 * 控制循环频率 = tick_rate（默认 100Hz，见 bt_config.yaml），悬停/轨迹输出经 out_slew_max 变化率限幅。
 * 悬停提前切出：z ≥ hover_reach_threshold(默认 0.9m) 持续 hold 时间即切悬停（跳过轨迹剩余段）。
 *
 * 锁存 ch5_release_required 由本节点统一拥有：abort() 置位，runIdle 检测 CH5 回落清零。
 * arm_action 只读该锁存（置位时跳过重新解锁），从而避免「中止→立即重解锁」的循环。
 *
 * Ports:
 *   skill (string):  技能名（对应 skills/ 子目录）
 *
 * Blackboard I/O:
 *   读: "channels"、参数 enable_height_monitor / landing_height_threshold / landing_hold_time
 *       + 轨迹参数 feedback_mode / target_mode / trigger_source / target_* / 控制器增益等
 *   写: "channels"、"ch5_release_required"
 */
class SkillExecutor : public BT::StatefulActionNode
{
public:
    SkillExecutor(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config)
        , traj_index_(0), start_time_(ros::Time(0))
        , state_(State::IDLE), enable_height_monitor_(false)
        , landing_height_threshold_(0.1), landing_hold_time_(2.0)
        , landed_since_(ros::Time(0))
        , feedback_mode_("closed_loop")
        , target_(2.0, 0.0, 1.0)
        , duration_mode_("auto"), fixed_duration_(0.0)
        , cruise_speed_(0.5), T_min_(2.0), T_max_(8.0)
        , arrive_tol_(0.1), takeoff_arrive_tol_(0.6), vel_tol_(0.1), arrive_hold_(1.0)
        , danger_tol_(0.8), timeout_traj_(12.0), odom_stale_timeout_(0.3)
        , ch6_mid_low_(1400), ch6_mid_high_(1600), ch6_debounce_(0.3)
        , gimbal_lock_(true)
        , traj_start_time_(ros::Time(0))
        , trigger_since_(ros::Time(0))
        , arrived_since_(ros::Time(0))
        , height_source_("rangefinder")
        , hover_hold_before_traj_(5.0), override_threshold_(10)
        , hover_target_(0.0, 0.0, 0.0)
        , hover_entered_since_(ros::Time(0))
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("skill") };
    }

    BT::NodeStatus onStart() override
    {
        std::string skill_name;
        if (!getInput("skill", skill_name)) {
            ROS_ERROR("SkillExecutor: missing 'skill' port");
            return BT::NodeStatus::FAILURE;
        }

        // 读参数（带默认值）
        auto bb = config().blackboard;
        bb->get("enable_height_monitor", enable_height_monitor_);
        bb->get("landing_height_threshold", landing_height_threshold_);
        bb->get("landing_hold_time", landing_hold_time_);

        // ---- 起转 + 起飞（rest-to-rest snap）参数 ----
        bb->get("spool_duration", spool_duration_);
        bb->get("spool_peak_ratio", spool_peak_ratio_);
        bb->get("spool_throttle_min", spool_throttle_min_);
        bb->get("takeoff_duration", takeoff_duration_);
        bb->get("takeoff_lateral", takeoff_lateral_);

        // ---- 轨迹跟踪参数 ----
        bb->get("feedback_mode", feedback_mode_);
        bb->get("target_mode", target_mode_);
        bb->get("trigger_source", trigger_source_);
        double tx = target_.x(), ty = target_.y(), tz = target_.z();
        bb->get("target_x", tx); bb->get("target_y", ty); bb->get("target_z", tz);
        target_ = Eigen::Vector3d(tx, ty, tz);
        bb->get("duration_mode", duration_mode_);
        bb->get("fixed_duration", fixed_duration_);
        bb->get("cruise_speed", cruise_speed_);
        bb->get("T_min", T_min_);
        bb->get("T_max", T_max_);
        bb->get("arrive_tol", arrive_tol_);
        bb->get("takeoff_arrive_tol", takeoff_arrive_tol_);
        bb->get("vel_tol", vel_tol_);
        bb->get("arrive_hold", arrive_hold_);
        bb->get("danger_tol", danger_tol_);
        bb->get("timeout_traj", timeout_traj_);
        bb->get("odom_stale_timeout", odom_stale_timeout_);
        bb->get("vins_jump_max_speed", vins_jump_max_speed_);
        bb->get("ch6_mid_low", ch6_mid_low_);
        bb->get("ch6_mid_high", ch6_mid_high_);
        bb->get("ch6_debounce", ch6_debounce_);
        bb->get("gimbal_lock", gimbal_lock_);
        bb->get("height_source", height_source_);
        bb->get("hover_hold_before_traj", hover_hold_before_traj_);
        bb->get("override_threshold", override_threshold_);

        // ---- 位置控制器参数 ----
        ctrl_params_.feedback_enabled = (feedback_mode_ == "closed_loop");
        bb->get("kp_xy", ctrl_params_.kp_xy);
        bb->get("kp_z", ctrl_params_.kp_z);
        bb->get("kd_xy", ctrl_params_.kd_xy);
        bb->get("kd_z", ctrl_params_.kd_z);
        bb->get("gravity", ctrl_params_.gravity);
        bb->get("angle_max", ctrl_params_.angle_max);
        bb->get("hover_throttle", ctrl_params_.hover_throttle);
        bb->get("k_thr", ctrl_params_.k_thr);
        bb->get("pitch_sign", ctrl_params_.pitch_sign);
        bb->get("roll_sign", ctrl_params_.roll_sign);
        bb->get("yaw_enabled", ctrl_params_.yaw_enabled);
        bb->get("yaw_kp", ctrl_params_.yaw_kp);
        bb->get("out_slew_max", out_slew_max_);

        // ---- 悬停安全限幅 ----
        bb->get("hover_danger_tol", hover_danger_tol_);
        bb->get("hover_danger_hold", hover_danger_hold_);
        bb->get("hover_max_height", hover_max_height_);
        bb->get("hover_min_height", hover_min_height_);
        bb->get("hover_safety_action", hover_safety_action_);
        bb->get("hover_bias_enabled", hover_bias_enabled_);
        bb->get("hover_bias_ki", hover_bias_ki_);
        bb->get("hover_bias_max", hover_bias_max_);
        bb->get("hover_bias_ep_max", hover_bias_ep_max_);
        bb->get("hover_bias_vz_max", hover_bias_vz_max_);

        // 路径约定: <skills_dir>/<skill_name>/trajectory.csv + meta.yaml
        std::string skills_dir;
        bb->get("skills_dir", skills_dir);
        std::string traj_path = skills_dir + "/" + skill_name + "/trajectory.csv";
        std::string meta_path = skills_dir + "/" + skill_name + "/meta.yaml";

        meta_       = SkillLoader::loadMeta(meta_path);
        // 起飞已改 rest-to-rest snap，不再使用 CSV 轨迹；保留加载能力，空轨迹不再 FAILURE
        trajectory_ = SkillLoader::loadTrajectory(traj_path);

        traj_index_   = 0;
        start_time_   = ros::Time(0);
        state_        = State::IDLE;
        landed_since_ = ros::Time(0);
        override_prev_ = false;
        hover_bias_    = 0.0;
        last_bias_time_ = ros::Time(0);

        ROS_INFO("SkillExecutor [%s]: 待命 (IDLE, height_monitor=%s)",
                 skill_name.c_str(), enable_height_monitor_ ? "on" : "off");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // 快照遥测数据（读取输入 CH5、测距高度、armingFlags、触发信号）
        auto& rb = RosBackend::instance();
        bool ch5_high = false, ch5_low = false, override_active = false, trigger_rising = false;
        double current_z = 0.0;
        uint32_t flags = 0;
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            const auto& ch = rb.remoteChannels();
            uint16_t ch5 = (ch.size() > 4) ? ch[4] : 0;
            uint16_t ch6 = (ch.size() > 5) ? ch[5] : 0;
            ch5_high = (ch5 > 1750);
            ch5_low  = (ch5 < 1750);
            override_active = lateralOverrideDetected(ch);
            current_z = rb.rangefinder();
            flags = rb.armingFlags();

            // 触发边沿：CH6 上升沿 或 /traj_trigger 计数变化（按 trigger_source）
            const bool ch6_mid = (ch6 >= ch6_mid_low_ && ch6 <= ch6_mid_high_);
            const bool ch6_rising = ch6_mid && !ch6_prev_mid_;
            ch6_prev_mid_ = ch6_mid;
            if (trigger_source_ == "topic") {
                const uint64_t cnt = rb.triggerCount();
                trigger_rising = (cnt != trigger_prev_count_);
                trigger_prev_count_ = cnt;
            } else {
                trigger_rising = ch6_rising;
            }
        }

        // 每拍回写偏置估计值（各状态统一用当前 hover_bias_）+ 记录状态（供 bt_server 发布 /bt_engine/skill_state）
        ctrl_params_.hover_bias = hover_bias_;
        config().blackboard->set("skill_state", static_cast<int>(state_));

        switch (state_) {
            case State::IDLE:      return runIdle(ch5_high, ch5_low, flags);
            case State::SPOOL_UP:  return runSpoolUp(ch5_high);
            case State::TAKEOFF:   return runTakeoff(ch5_high);
            case State::HOVER:     return runHover(ch5_low, trigger_rising, override_active, current_z);
            case State::TRAJECTORY: return runTrajectory(ch5_low, override_active);
            case State::LANDING:   return runLanding(current_z, flags);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_WARN("SkillExecutor: HALTED → 中止上锁");
        disarmAndIdle();  // 外部 halt（重新解锁/手动模式）：上锁+复位，但不置锁存
    }

private:
    enum class State { IDLE, SPOOL_UP, TAKEOFF, HOVER, TRAJECTORY, LANDING };

    // ---- 待命：等解锁 + CH5 上升沿 ----
    BT::NodeStatus runIdle(bool ch5_high, bool ch5_low, uint32_t flags)
    {
        auto bb = config().blackboard;
        bool latch = false;
        bb->get("ch5_release_required", latch);

        // CH5 回落 → 清锁存（中止后需 CH5 先拨低，才允许重新解锁+起飞）
        if (ch5_low && latch) {
            bb->set("ch5_release_required", false);
            latch = false;
        }

        // 未解锁：什么都不写，等 arm_action 解锁确认（避免抢 CH4/CH5）
        if (!(flags & 0x04u)) {
            return BT::NodeStatus::RUNNING;
        }

        // 已解锁：写怠速通道（CH3 油门最低 / CH6 模式低位 / CH7 关RTH；CH4/CH5 由 arm_action 管）
        setChannel(2, 1000);
        setChannel(5, 1000);
        setChannel(6, 1000);

        // CH5 上升沿（锁存已清）→ 起转
        if (ch5_high && !latch) {
            ROS_INFO("SkillExecutor: CH5 上升沿 → 起转 (spool-up)");
            establishTaskFrame();   // 记录起飞点原点 + 起飞时机头朝向
            state_            = State::SPOOL_UP;
            spool_start_time_ = ros::Time::now();
        }
        return BT::NodeStatus::RUNNING;
    }

    // S 曲线（smootherstep）：端点函数值/一阶导=0，起转油门 ramp 平滑
    static double smootherstep(double t)
    {
        t = std::max(0.0, std::min(1.0, t));
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    }

    // ---- 起转：开环油门 S 曲线 ramp 到悬停值以下，电机起转但不起飞 ----
    BT::NodeStatus runSpoolUp(bool ch5_high)
    {
        // 地面上 CH5 拨低 → 中止上锁（尚未离地，安全）
        if (!ch5_high) {
            ROS_WARN("SkillExecutor: 起转中 CH5 拨低 → 中止上锁");
            abort();
            return BT::NodeStatus::RUNNING;
        }

        double elapsed = (ros::Time::now() - spool_start_time_).toSec();
        if (elapsed >= spool_duration_) {
            enterTakeoff();
            return BT::NodeStatus::RUNNING;
        }

        // 油门 S 曲线（smootherstep：端点油门速率=0，接手平滑）
        double s = smootherstep(elapsed / std::max(1e-6, spool_duration_));
        // 峰值按「怠速→悬停」跨度取比例：1000 + (hover_throttle−1000)×ratio，而非 hover_throttle×ratio
        double peak = spool_throttle_min_ + (ctrl_params_.hover_throttle - spool_throttle_min_) * spool_peak_ratio_;
        double thr  = spool_throttle_min_ + (peak - spool_throttle_min_) * s;

        setChannel(2, static_cast<int>(std::round(thr)));  // CH3 油门 ramp
        setChannel(0, 1500);  // CH1 roll 中立
        setChannel(1, 1500);  // CH2 pitch 中立
        setChannel(3, 1500);  // CH4 yaw 中立
        setChannel(5, 1000);  // CH6 ANGLE
        setChannel(6, 1000);  // CH7 关 RTH

        ROS_DEBUG_THROTTLE(1.0, "SkillExecutor: SPOOL_UP t=%.2f thr=%.0f", elapsed, thr);
        return BT::NodeStatus::RUNNING;
    }

    // ---- 进入起飞：以当前测距高度为起点，rest-to-rest snap 爬到目标高度 ----
    void enterTakeoff()
    {
        auto& rb = RosBackend::instance();
        double z0 = 0.0;
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            z0 = rb.rangefinder();
        }
        Eigen::Vector3d p_start(0.0, 0.0, z0);
        Eigen::Vector3d goal(0.0, 0.0, meta_.target_z);
        traj_ = trajectory::MinSnapTrajectory::singleSegment(p_start, goal, takeoff_duration_);

        start_time_    = ros::Time::now();
        arrived_since_ = ros::Time(0);
        state_ = State::TAKEOFF;
        ROS_INFO("SkillExecutor: 起飞 rest-to-rest z=%.2f→%.2f T=%.2fs",
                 z0, meta_.target_z, takeoff_duration_);
    }

    // ---- 起飞：rest-to-rest snap + 统一位置控制器（离地后异常 → 降落 RTH，不上锁） ----
    BT::NodeStatus runTakeoff(bool ch5_high)
    {
        // 离地后 CH5 拨低 → 降落（RTH），而非上锁坠机
        if (!ch5_high) {
            ROS_WARN("SkillExecutor: 起飞中 CH5 拨低 → 降落 (RTH)");
            startLanding();
            return BT::NodeStatus::RUNNING;
        }

        double elapsed = (ros::Time::now() - start_time_).toSec();

        // 超时 → 降落（RTH）
        if (elapsed > meta_.timeout) {
            ROS_WARN("SkillExecutor: 起飞超时 (%.1fs > %.1fs) → 降落", elapsed, meta_.timeout);
            startLanding();
            return BT::NodeStatus::RUNNING;
        }

        // 读测量：首飞 takeoff_lateral=false → 不要求 VINS；读失败也走降落而非上锁
        Eigen::Vector3d p_M, v_M;
        if (!readState(p_M, v_M, takeoff_lateral_)) {
            ROS_WARN("SkillExecutor: 起飞读状态失败 → 降落");
            startLanding();
            return BT::NodeStatus::RUNNING;
        }

        // 参考轨迹（t 越界钳制到 T，保持终点悬停参考）
        Eigen::Vector3d p_ref, v_ref, a_ref;
        traj_.evaluate(elapsed, p_ref, v_ref, a_ref);
        RosBackend::instance().publishTrajRef(p_ref, v_ref, a_ref);

        // 到达判定 → 悬停（rest-to-rest 端点 v≈0，达到即平滑切换）
        double err = std::abs(p_M.z() - meta_.target_z);
        if (elapsed >= traj_.totalTime() && err <= takeoff_arrive_tol_ && std::abs(v_M.z()) <= vel_tol_) {
            if (arrived_since_ == ros::Time(0)) arrived_since_ = ros::Time::now();
            if ((ros::Time::now() - arrived_since_).toSec() >= arrive_hold_) {
                ROS_INFO("SkillExecutor: 起飞到达 z=%.2fm (v=%.2f) → 悬停", p_M.z(), v_M.z());
                enterHover();
                return BT::NodeStatus::RUNNING;
            }
        } else {
            arrived_since_ = ros::Time(0);
        }

        // 控制输出：首飞 takeoff_lateral=false → override=true → 仅垂直（CH1/CH2 中立）
        applyController(p_ref, v_ref, a_ref, p_M, v_M, !takeoff_lateral_);

        ROS_DEBUG_THROTTLE(2.0, "SkillExecutor: TAKEOFF t=%.2f z=%.2f ref_z=%.2f err=%.2f",
                           elapsed, p_M.z(), p_ref.z(), err);
        return BT::NodeStatus::RUNNING;
    }

    // ---- 悬停：上位机 ANGLE 定高定点，检测触发边沿 / 安全越界 / 降落 ----
    BT::NodeStatus runHover(bool ch5_low, bool trigger_rising, bool override_active, double current_z)
    {
        // 触发边沿（CH6 上升沿或 /traj_trigger 话题）→ 去抖 + 悬停稳定后触发一次轨迹
        if (trigger_rising)
            trigger_since_ = ros::Time::now();
        if (trigger_since_ != ros::Time(0)) {
            double stable = (ros::Time::now() - hover_entered_since_).toSec();
            if ((ros::Time::now() - trigger_since_).toSec() >= ch6_debounce_
                && stable >= hover_hold_before_traj_) {
                if (enterTrajectory()) {
                    trigger_since_ = ros::Time(0);   // 消费本次触发
                    return BT::NodeStatus::RUNNING;
                }
            }
        }

        // CH5 拨低 → 降落
        if (ch5_low) {
            ROS_INFO("SkillExecutor: 悬停后 CH5 拨低 → 降落 (CH7=2000 RTH)");
            startLanding();
            return BT::NodeStatus::RUNNING;
        }

        // 位置控制：悬停定高定点（目标 = hover_target_）
        Eigen::Vector3d p_M, v_M;
        if (!readState(p_M, v_M)) {
            abort();
            return BT::NodeStatus::RUNNING;
        }

        // 接管结束（下降沿）→ 悬停目标刷新为当前位置，避免松杆后弹回接管前的点
        if (override_prev_ && !override_active) {
            hover_target_ = p_M;
            ROS_INFO("SkillExecutor: 接管结束 → 悬停目标刷新为 (%.2f, %.2f, %.2f)",
                     p_M.x(), p_M.y(), p_M.z());
        }
        override_prev_ = override_active;

        // 悬停安全限幅：跟踪误差/绝对高度越界 → 降落（或上锁）
        if (checkHoverSafety(p_M, current_z)) {
            return BT::NodeStatus::RUNNING;   // 已切换 LANDING / abort
        }

        // 油门偏置估计：仅稳定悬停且未接管才积分，学习真实悬停油门
        if (hover_bias_enabled_ && !override_active) {
            ros::Time now = ros::Time::now();
            double dt = last_bias_time_.isZero() ? 0.0 : (now - last_bias_time_).toSec();
            last_bias_time_ = now;
            if (dt > 0.0 && dt <= 0.1) {
                double ep_z = hover_target_.z() - p_M.z();
                if (std::abs(ep_z) < hover_bias_ep_max_ && std::abs(v_M.z()) < hover_bias_vz_max_) {
                    hover_bias_ += hover_bias_ki_ * ep_z * dt;   // 偏低(ep_z>0)→增 bias→增油门
                    hover_bias_ = std::max(-hover_bias_max_, std::min(hover_bias_max_, hover_bias_));
                }
            }
        }

        // 悬停参考也发布，保证参考话题在整段飞行中连续
        RosBackend::instance().publishTrajRef(hover_target_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        applyController(hover_target_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        p_M, v_M, override_active);
        return BT::NodeStatus::RUNNING;
    }

    // 悬停安全限幅：跟踪误差 / 绝对高度越界持续 hover_danger_hold → landing（或 abort）
    bool checkHoverSafety(const Eigen::Vector3d& p_M, double current_z)
    {
        double err = (p_M - hover_target_).norm();
        bool breach = (err > hover_danger_tol_) ||
                      (current_z > hover_max_height_) ||
                      (current_z < hover_min_height_);
        if (breach) {
            if (hover_danger_since_ == ros::Time(0))
                hover_danger_since_ = ros::Time::now();
            if ((ros::Time::now() - hover_danger_since_).toSec() >= hover_danger_hold_) {
                ROS_ERROR("SkillExecutor: 悬停安全越界 err=%.2fm z=%.2fm → %s",
                          err, current_z, hover_safety_action_.c_str());
                if (hover_safety_action_ == "abort") abort();
                else startLanding();
                return true;
            }
        } else {
            hover_danger_since_ = ros::Time(0);
        }
        return false;
    }

    // 进入降落：CH7=2000 RTH（无 GPS 时飞控自动就地降落），不动 CH5（保持解锁）
    void startLanding()
    {
        state_ = State::LANDING;
        setChannel(2, 1000);  // CH3 油门最低
        setChannel(5, 1000);  // CH6 ANGLE
        setChannel(6, 2000);  // CH7 RTH
        landed_since_ = ros::Time(0);
        hover_danger_since_ = ros::Time(0);
        hover_bias_ = 0.0;          // 降落不学、不用残留偏置
        last_bias_time_ = ros::Time(0);
    }

    // ---- 轨迹跟踪：最小 snap + 位置控制器 ----
    BT::NodeStatus runTrajectory(bool ch5_low, bool override_active)
    {
        const bool closed = (feedback_mode_ == "closed_loop");

        // 0. CH5 拨低 → 降落（同悬停态；轨迹由边沿触发，飞行中不再受 CH6 状态影响）
        if (ch5_low) {
            ROS_WARN("SkillExecutor: 轨迹跟踪中 CH5 拨低 → 降落 (CH7=2000 RTH)");
            startLanding();
            return BT::NodeStatus::RUNNING;
        }

        double elapsed = (ros::Time::now() - traj_start_time_).toSec();

        // 2. 超时 → 中止
        if (elapsed > timeout_traj_) {
            ROS_WARN("SkillExecutor: 轨迹超时 (%.1fs > %.1fs) → 中止上锁", elapsed, timeout_traj_);
            abort();
            return BT::NodeStatus::RUNNING;
        }

        // 3. 参考轨迹
        Eigen::Vector3d p_ref, v_ref, a_ref;
        traj_.evaluate(elapsed, p_ref, v_ref, a_ref);
        RosBackend::instance().publishTrajRef(p_ref, v_ref, a_ref);

        // 4. 读测量（水平 VINS + 垂直高度源）；闭环失效返回 false
        Eigen::Vector3d p_M, v_M;
        if (!readState(p_M, v_M)) {
            abort();
            return BT::NodeStatus::RUNNING;
        }

        // 5. DANGER：位置误差超限（闭环且非接管让位）
        if (closed && !override_active) {
            double err = (p_M - p_ref).norm();
            if (err > danger_tol_) {
                ROS_ERROR("SkillExecutor: 位置误差 %.2fm > %.2fm → 中止上锁", err, danger_tol_);
                abort();
                return BT::NodeStatus::RUNNING;
            }
        }

        // 6. 到达判定：闭环按位置/速度，开环按时间
        bool done = false;
        if (closed) {
            double to_target = (p_M - traj_goal_).norm();   // 用实际终点 traj_goal_（relative 模式下 target_ 只是增量）
            done = (elapsed >= traj_.totalTime()) && (to_target <= arrive_tol_) && (v_M.norm() <= vel_tol_);
        } else {
            done = (elapsed >= traj_.totalTime());
        }

        if (done) {
            if (arrived_since_ == ros::Time(0)) arrived_since_ = ros::Time::now();
            if ((ros::Time::now() - arrived_since_).toSec() >= arrive_hold_) {
                ROS_INFO("SkillExecutor: 到达目标 (%.2f,%.2f,%.2f) → 回上位机 ANGLE 悬停",
                         p_M.x(), p_M.y(), p_M.z());
                hover_target_ = traj_goal_;
                hover_entered_since_ = ros::Time::now();
                state_ = State::HOVER;
                override_prev_ = false;
                arrived_since_ = ros::Time(0);
                return BT::NodeStatus::RUNNING;
            }
        } else {
            arrived_since_ = ros::Time(0);
        }

        // 7. 控制器输出（写 CH1-CH4，CH6 恒 ANGLE）
        applyController(p_ref, v_ref, a_ref, p_M, v_M, override_active);

        ROS_DEBUG_THROTTLE(2.0, "SkillExecutor: TRAJ t=%.2f p=(%.2f,%.2f,%.2f) ref=(%.2f,%.2f,%.2f)",
                           elapsed, p_M.x(), p_M.y(), p_M.z(), p_ref.x(), p_ref.y(), p_ref.z());

        return BT::NodeStatus::RUNNING;
    }

    // ---- 进入悬停：水平停当前位置，垂向目标 = 起飞目标高度 meta_.target_z（给油门偏置正确参考，否则 ep_z=0 学不到）----
    void enterHover()
    {
        Eigen::Vector3d p_M, v_M;
        readState(p_M, v_M);   // 失败时 p_M 为 0，后续 runHover 会因闭环失效 abort
        hover_target_ = Eigen::Vector3d(p_M.x(), p_M.y(), meta_.target_z);
        hover_entered_since_ = ros::Time::now();
        hover_danger_since_ = ros::Time(0);
        override_prev_ = false;
        state_ = State::HOVER;
        ROS_INFO("SkillExecutor: 进入悬停 target=(%.2f,%.2f,%.2f) height_source=%s",
                 hover_target_.x(), hover_target_.y(), hover_target_.z(), height_source_.c_str());
    }

    // ---- 建立任务系：原点 = 起飞点，x = 起飞时机头水平投影，z = 重力向上（飞控 roll/pitch）----
    void establishTaskFrame()
    {
        auto& rb = RosBackend::instance();
        double yaw0 = 0.0;
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            if (rb.hasOdom()) {
                p0_W_ = rb.odomPos();
                buildTaskFrame(rb.odomQuat(), rb.attitudeRoll(), rb.attitudePitch());
                yaw0 = rb.attitudeYaw();   // 起飞时机头航向，供 XY 速度帧对齐
            } else {
                p0_W_ = Eigen::Vector3d::Zero();
                R_WM_ = Eigen::Matrix3d::Identity();
            }
        }
        // 交给速度估计器做 XY 帧对齐并重置积分（内部自加锁）
        rb.setTaskFrame(R_WM_, p0_W_, yaw0);
        prev_p_valid_ = false;   // 重建任务系后，VINS 位置钳制基准清零
        ROS_INFO("SkillExecutor: 任务系已建立 (原点=起飞点, x=起飞时机头, z=重力参考 roll/pitch, yaw0=%.2f rad)",
                 yaw0);
    }

    // ---- 读取任务系测量（水平 VINS + 垂直高度源相对值）；闭环失效返回 false ----
    bool readState(Eigen::Vector3d& p_M, Eigen::Vector3d& v_M, bool require_odom = true)
    {
        const bool closed = (feedback_mode_ == "closed_loop");
        auto& rb = RosBackend::instance();
        p_M.setZero(); v_M.setZero();
        bool odom_ok = false; double odom_age = 1e9;
        ros::Time odom_stamp;

        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            if (rb.hasOdom()) {
                p_M = R_WM_.transpose() * (rb.odomPos() - p0_W_);
                // 水平速度：速度估计器（飞控加速度计积分 + VINS 位置差分去漂移），替代 odomVel(恒0)
                Eigen::Vector2d vxy = rb.estVelXY();
                v_M.x() = vxy.x();
                v_M.y() = vxy.y();
                odom_age = (ros::Time::now() - rb.odomStamp()).toSec();
                odom_stamp = rb.odomStamp();
                odom_ok = true;
            }
            // 垂直用高度源绝对值（离地/相对起飞点）；vins 源保持任务系 z
            if (height_source_ == "barometer") {
                p_M.z() = rb.altitude();
                v_M.z() = rb.estVelZ();   // 速度环反馈（一阶互补，测距仪去漂移）
            } else if (height_source_ != "vins") {   // 默认 rangefinder
                p_M.z() = rb.rangefinder();
                v_M.z() = rb.estVelZ();
            }
        }

        // VINS 位置(XY)单拍跳变钳制：位移/时间超阈值判跳变、沿用上一拍。
        // 只作用控制器位置反馈；速度估计走 ros_backend 原始 odom 差分，互不影响。
        if (odom_ok) {
            Eigen::Vector2d pxy(p_M.x(), p_M.y());
            if (prev_p_valid_) {
                double dt = (odom_stamp - prev_p_stamp_).toSec();
                if (dt > 1e-3) {
                    double max_step = vins_jump_max_speed_ * dt;
                    double jump = (pxy - prev_p_xy_).norm();
                    if (jump > max_step) {
                        ROS_WARN_THROTTLE(1.0, "SkillExecutor: VINS 位置跳变 %.2fm (限 %.2fm) → 沿用上一拍",
                                          jump, max_step);
                        p_M.x() = prev_p_xy_.x();
                        p_M.y() = prev_p_xy_.y();
                    } else {
                        prev_p_xy_ = pxy;
                        prev_p_stamp_ = odom_stamp;
                    }
                }
            } else {
                prev_p_xy_ = pxy;
                prev_p_stamp_ = odom_stamp;
                prev_p_valid_ = true;
            }
        }

        if (closed && require_odom && (!odom_ok || odom_age > odom_stale_timeout_)) {
            ROS_ERROR("SkillExecutor: VINS 位姿失效 (ok=%d, age=%.2fs) → 中止上锁", odom_ok, odom_age);
            return false;
        }
        return true;
    }

    // ---- 位置控制器输出（写 CH1-CH4，CH6 恒 ANGLE；override_active 时仅垂直） ----
    void applyController(const Eigen::Vector3d& p_ref, const Eigen::Vector3d& v_ref,
                         const Eigen::Vector3d& a_ref, const Eigen::Vector3d& p_M,
                         const Eigen::Vector3d& v_M, bool override_active)
    {
        ctrl_params_.lateral_enabled = !override_active;   // 接管 → 仅垂直让位
        trajectory::CtrlOutput out;
        trajectory::PositionController(ctrl_params_).update(p_ref, v_ref, a_ref, p_M, v_M, 0.0, out);

        out = applySlewLimit(out);        // 输出变化率限幅（防单拍野值/大误差 PWM 阶跃）

        setChannel(0, out.roll_pwm);      // CH1 roll
        setChannel(1, out.pitch_pwm);     // CH2 pitch
        setChannel(2, out.throttle_pwm);  // CH3 油门
        setChannel(3, out.yaw_pwm);       // CH4 yaw
        setChannel(5, 1000);              // CH6 ANGLE（全程）
    }

    // 输出变化率限幅：限制每通道 PWM 变化速率（out_slew_max<=0 或首拍/长时间未更新时直接放行）
    trajectory::CtrlOutput applySlewLimit(const trajectory::CtrlOutput& out)
    {
        if (out_slew_max_ <= 0.0) return out;
        ros::Time now = ros::Time::now();
        if (!ctl_init_) {
            ctl_init_ = true;
            last_out_ = out;
            last_ctl_time_ = now;
            return out;
        }
        double dt = (now - last_ctl_time_).toSec();
        if (dt <= 0.0 || dt > 0.5) {   // 时间异常/跨状态长时间未更新 → 重置基准，本拍不限幅
            last_out_ = out;
            last_ctl_time_ = now;
            return out;
        }
        double step = out_slew_max_ * dt;
        trajectory::CtrlOutput clamped = out;
        clamped.throttle_pwm = clampStep(out.throttle_pwm, last_out_.throttle_pwm, step);
        clamped.roll_pwm     = clampStep(out.roll_pwm,     last_out_.roll_pwm,     step);
        clamped.pitch_pwm    = clampStep(out.pitch_pwm,    last_out_.pitch_pwm,    step);
        clamped.yaw_pwm      = clampStep(out.yaw_pwm,      last_out_.yaw_pwm,      step);
        last_out_ = clamped;
        last_ctl_time_ = now;
        return clamped;
    }

    int clampStep(int v, int last, double step) const
    {
        double lo = static_cast<double>(last) - step;
        double hi = static_cast<double>(last) + step;
        return static_cast<int>(std::round(std::min(hi, std::max(lo, static_cast<double>(v)))));
    }

    // ---- 检测遥控姿态通道接管（CH1/CH2/CH4 偏离中立超阈值） ----
    bool lateralOverrideDetected(const std::vector<uint16_t>& ch)
    {
        static const int axes[3] = {0, 1, 3};  // CH1 roll / CH2 pitch / CH4 yaw
        for (int i : axes) {
            if (ch.size() > static_cast<size_t>(i) &&
                std::abs(static_cast<int>(ch[i]) - 1500) > override_threshold_)
                return true;
        }
        return false;
    }

    // ---- 进入轨迹跟踪：以当前悬停点为起点，目标 = 当前点 + 相对增量（或绝对点） ----
    bool enterTrajectory()
    {
        const bool closed = (feedback_mode_ == "closed_loop");

        // 轨迹起点 = 当前任务系位置（无跳变）
        Eigen::Vector3d p_start = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_dummy;
        if (closed && !readState(p_start, v_dummy)) {
            ROS_WARN("SkillExecutor: 触发轨迹但 VINS 无位姿 → 忽略，保持悬停");
            trigger_since_ = ros::Time(0);   // 本次触发作废，需重新触发
            return false;
        }

        // 目标：relative=当前点+增量；absolute=固定绝对点（起飞点机头系）
        traj_goal_ = (target_mode_ == "relative") ? (p_start + target_) : target_;
        const double T = planDuration((traj_goal_ - p_start).norm());
        traj_ = trajectory::MinSnapTrajectory::singleSegment(p_start, traj_goal_, T);

        traj_start_time_ = ros::Time::now();
        arrived_since_ = ros::Time(0);
        state_ = State::TRAJECTORY;

        if (gimbal_lock_)
            ROS_INFO("SkillExecutor: 轨迹跟踪开始，云台锁定（不发送云台指令）");
        ROS_INFO("SkillExecutor: 进入轨迹跟踪 start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f) T=%.2fs %s",
                 p_start.x(), p_start.y(), p_start.z(),
                 traj_goal_.x(), traj_goal_.y(), traj_goal_.z(), T, closed ? "closed_loop" : "open_loop");
        return true;
    }

    // 由 VINS 四元数（相机→世界, ENU）+ 飞控姿态 roll/pitch（重力参考）构建任务系
    // u_B = 重力向上在机体系(ENU)的方向，由飞控 roll/pitch 反推（近水平 ≈ [0,0,1]）：
    //   INAV 姿态为 FRD 系（x前/y右/z下），映射到 ENU（x前/y左/z上）后
    //   u_B = [sin(pitch), sin(roll)·cos(pitch), cos(roll)·cos(pitch)]
    // 注意：相机在云台上，此处假设起飞瞬间云台居中且相机≈机体（R_WB≈R_WC），否则 z_W 含云台倾斜。
    void buildTaskFrame(const Eigen::Quaterniond& q, double roll, double pitch)
    {
        Eigen::Matrix3d R_WB = q.toRotationMatrix();
        Eigen::Vector3d u_B(std::sin(pitch),
                            std::sin(roll) * std::cos(pitch),
                            std::cos(roll) * std::cos(pitch));
        Eigen::Vector3d z_W = R_WB * u_B;                        // 重力向上在 W 的表达
        z_W.normalize();
        Eigen::Vector3d x_W = R_WB * Eigen::Vector3d::UnitX();   // 机头
        x_W -= (x_W.dot(z_W)) * z_W;                             // 投影到水平面
        if (x_W.norm() < 1e-6) {
            Eigen::Vector3d y_B = R_WB * Eigen::Vector3d::UnitY();
            x_W = y_B - (y_B.dot(z_W)) * z_W;
        }
        x_W.normalize();
        Eigen::Vector3d y_W = z_W.cross(x_W);                    // 左
        R_WM_.col(0) = x_W;
        R_WM_.col(1) = y_W;
        R_WM_.col(2) = z_W;
    }

    double planDuration(double dist) const
    {
        if (duration_mode_ == "fixed" && fixed_duration_ > 0.0)
            return fixed_duration_;
        double T = (cruise_speed_ > 1e-6) ? dist / cruise_speed_ : dist;
        return std::max(T_min_, std::min(T_max_, T));
    }

    // ---- 降落：保持 RTH，检测着陆后上锁 ----
    BT::NodeStatus runLanding(double current_z, uint32_t flags)
    {
        setChannel(2, 1000);  // CH3 油门最低
        setChannel(5, 1000);  // CH6 ANGLE
        setChannel(6, 2000);  // CH7 保持 RTH

        bool landed = false;

        // 信号 1：INAV 自带着陆检测标志 bit30 (ARMING_DISABLED_LANDING_DETECTED)
        if (flags & 0x40000000u) {
            landed = true;
        }
        // 信号 2：飞控 disarm_on_landing 自动上锁 → ARMED(bit2) 清零
        if (!(flags & 0x04u)) {
            landed = true;
        }
        // 信号 3：测距高度持续低于阈值（主信号，不依赖飞控配置）
        if (current_z <= landing_height_threshold_) {
            if (landed_since_ == ros::Time(0)) landed_since_ = ros::Time::now();
            if ((ros::Time::now() - landed_since_).toSec() >= landing_hold_time_) {
                landed = true;
            }
        } else {
            landed_since_ = ros::Time(0);
        }

        if (landed) {
            ROS_INFO("SkillExecutor: 检测到着陆 → 上锁 (z=%.2fm, flags=0x%x)", current_z, flags);
            abort();
            return BT::NodeStatus::RUNNING;
        }

        return BT::NodeStatus::RUNNING;
    }

    // ---- 中止：上锁 + 复位 + 回 IDLE + 置锁存 ----
    void abort()
    {
        disarmAndIdle();
        config().blackboard->set("ch5_release_required", true);
    }

    // 上锁 + 复位 + 回 IDLE（不置锁存）
    void disarmAndIdle()
    {
        std::vector<uint16_t> channels;
        auto bb = config().blackboard;
        if (bb->get("channels", channels)) {
            if (channels.size() > 6) {
                channels[2] = 1000;  // CH3 油门最低
                channels[3] = 1500;  // CH4 偏航中立
                channels[4] = 1000;  // CH5 上锁
                channels[5] = 1000;  // CH6 低位 ANGLE
                channels[6] = 1000;  // CH7 关 RTH
            }
            bb->set("channels", channels);
        }

        // 回到待命，等待 CH5 回落→再拨高
        state_      = State::IDLE;
        traj_index_ = 0;
        landed_since_ = ros::Time(0);
        trigger_since_ = ros::Time(0);
        arrived_since_ = ros::Time(0);
        override_prev_ = false;      // 复位接管边沿，避免陈旧边沿误触发
        hover_bias_ = 0.0;           // 每趟飞行从头学，不携带上一趟陈旧偏置
        last_bias_time_ = ros::Time(0);
        bb->set("skill_state", 0);   // IDLE
    }

    void setChannel(int index, int value)
    {
        std::vector<uint16_t> channels;
        auto bb = config().blackboard;
        if (bb->get("channels", channels)) {
            if (static_cast<size_t>(index) < channels.size())
                channels[index] = static_cast<uint16_t>(value);
            bb->set("channels", channels);
        }
    }

    SkillLoader::Trajectory trajectory_;
    SkillLoader::Meta       meta_;
    size_t   traj_index_;
    ros::Time start_time_;

    State    state_;
    bool     enable_height_monitor_;
    double   landing_height_threshold_;
    double   landing_hold_time_;
    ros::Time landed_since_;

    // ---- 起转 + 起飞（rest-to-rest snap）成员 ----
    ros::Time spool_start_time_;
    double spool_duration_ = 2.0;
    double spool_peak_ratio_ = 0.90;
    double spool_throttle_min_ = 1000.0;
    double takeoff_duration_ = 3.0;
    bool   takeoff_lateral_ = false;

    // ---- 轨迹跟踪（最小 snap）成员 ----
    std::string feedback_mode_;
    Eigen::Vector3d target_;
    std::string duration_mode_;
    double fixed_duration_;
    double cruise_speed_, T_min_, T_max_;
    double arrive_tol_, takeoff_arrive_tol_, vel_tol_, arrive_hold_;
    double danger_tol_, timeout_traj_, odom_stale_timeout_;
    // ---- VINS 位置(XY)单拍跳变钳制（只作用位置反馈，不影响速度估计）----
    double vins_jump_max_speed_ = 2.0;      // 跳变判定速度阈值 (m/s)
    Eigen::Vector2d prev_p_xy_ = Eigen::Vector2d::Zero(); // 上一拍接受的位置(XY)
    ros::Time prev_p_stamp_;                // 上一拍接受位置的时间戳
    bool   prev_p_valid_ = false;
    int    ch6_mid_low_, ch6_mid_high_;
    double ch6_debounce_;
    bool   gimbal_lock_;
    ros::Time traj_start_time_;
    ros::Time trigger_since_;
    ros::Time arrived_since_;

    std::string height_source_;
    double hover_hold_before_traj_;
    int    override_threshold_;
    Eigen::Vector3d hover_target_;
    ros::Time hover_entered_since_;

    // ---- 悬停安全限幅 ----
    double hover_danger_tol_ = 0.5;
    double hover_danger_hold_ = 0.3;
    double hover_max_height_ = 1.5;
    double hover_min_height_ = 0.3;
    std::string hover_safety_action_ = "landing";
    ros::Time hover_danger_since_;

    // ---- 接管下降沿 + 油门偏置估计器 ----
    bool   override_prev_ = false;       // 上一拍是否接管（检测下降沿刷新 hover_target_）
    bool   hover_bias_enabled_ = true;   // 偏置估计器总开关
    double hover_bias_ki_ = 3.0;         // 积分增益 PWM/(m·s)
    double hover_bias_max_ = 80.0;       // bias 限幅 ±PWM
    double hover_bias_ep_max_ = 0.4;     // 仅 |ep_z| < 该值时积分
    double hover_bias_vz_max_ = 0.3;     // 仅 |v_z| < 该值时积分
    double hover_bias_ = 0.0;            // 学习到的悬停油门偏置 PWM
    ros::Time last_bias_time_;           // 上一拍偏置更新时间

    std::string target_mode_ = "relative";   // relative=相对增量 / absolute=固定绝对点
    std::string trigger_source_ = "ch6";     // ch6=遥控 CH6 边沿 / topic=/traj_trigger 话题
    Eigen::Vector3d traj_goal_ = Eigen::Vector3d::Zero(); // 本次轨迹终点（到达后作悬停目标）
    bool    ch6_prev_mid_ = false;   // CH6 上一 tick 是否中位（边沿检测）
    uint64_t trigger_prev_count_ = 0; // 话题触发上一次计数

    trajectory::CtrlParams ctrl_params_;
    trajectory::MinSnapTrajectory traj_;
    Eigen::Vector3d p0_W_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_WM_ = Eigen::Matrix3d::Identity();

    // ---- 输出变化率限幅（hover/trajectory 共用，跨 tick 保持状态）----
    double out_slew_max_ = 1000.0;      // PWM/s，0=不限幅
    trajectory::CtrlOutput last_out_;   // 上一拍控制器输出
    ros::Time last_ctl_time_;           // 上一拍输出时刻
    bool ctl_init_ = false;             // 首拍标志（首拍不限幅，直接以当前输出为基准）
};
