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
 *              检测 CH5 上升沿(>1750) 且锁存已清 → 建立任务系(起飞点机头系) → TAKEOFF
 *   TAKEOFF    起飞轨迹 csv 开环姿态(roll/pitch/yaw) + 油门 PID 定高修正；CH5 拨低 / 超时 / DANGER → abort()（上锁+回 IDLE）
 *   HOVER      上位机 ANGLE 定高定点（水平 VINS 闭环 + 垂直高度源闭环，CH6 恒 1000=ANGLE）；
 *              触发边沿（CH6 上升沿 或 /traj_trigger 话题，按 trigger_source）→ 去抖+悬停稳定
 *              → TRAJECTORY；CH5 拨低 → LANDING
 *   TRAJECTORY 最小 snap 轨迹 + 位置控制器；到达 → 回 HOVER；CH5 拨低 → LANDING；
 *              超时 / 位置误差超限 / VINS 失效 → abort()；飞行中 CH6 状态不影响轨迹
 *   LANDING    写 CH7=2000（RTH，无 GPS 时飞控自动降落）；检测着陆 → abort()（上锁+回 IDLE）
 *
 * 控制器分两套：起飞=csv 开环姿态 + 油门经典 PID(带 I)；悬停/轨迹=PositionController(差分平坦 PD)。
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
        , traj_index_(0), start_time_(ros::Time(0)), integral_(0.0), prev_error_(0.0)
        , state_(State::IDLE), enable_height_monitor_(false)
        , landing_height_threshold_(0.1), landing_hold_time_(2.0), hover_reach_hold_time_(0.0)
        , kp_(200.0), ki_(10.0), kd_(0.0), hover_reach_threshold_(0.9)
        , landed_since_(ros::Time(0))
        , reached_since_(ros::Time(0))
        , feedback_mode_("closed_loop")
        , target_(2.0, 0.0, 1.0)
        , duration_mode_("auto"), fixed_duration_(0.0)
        , cruise_speed_(0.5), T_min_(2.0), T_max_(8.0)
        , arrive_tol_(0.1), vel_tol_(0.1), arrive_hold_(1.0)
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
        bb->get("hover_reach_hold_time", hover_reach_hold_time_);
        bb->get("kp", kp_);
        bb->get("ki", ki_);
        bb->get("kd", kd_);
        bb->get("hover_reach_threshold", hover_reach_threshold_);

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
        bb->get("vel_tol", vel_tol_);
        bb->get("arrive_hold", arrive_hold_);
        bb->get("danger_tol", danger_tol_);
        bb->get("timeout_traj", timeout_traj_);
        bb->get("odom_stale_timeout", odom_stale_timeout_);
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

        // 路径约定: <skills_dir>/<skill_name>/trajectory.csv + meta.yaml
        std::string skills_dir;
        bb->get("skills_dir", skills_dir);
        std::string traj_path = skills_dir + "/" + skill_name + "/trajectory.csv";
        std::string meta_path = skills_dir + "/" + skill_name + "/meta.yaml";

        trajectory_ = SkillLoader::loadTrajectory(traj_path);
        meta_       = SkillLoader::loadMeta(meta_path);

        if (trajectory_.empty()) {
            ROS_ERROR("SkillExecutor: empty trajectory for %s", skill_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        traj_index_   = 0;
        start_time_   = ros::Time(0);
        integral_     = 0.0;
        prev_error_   = 0.0;
        state_        = State::IDLE;
        landed_since_ = ros::Time(0);
        reached_since_ = ros::Time(0);

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

        switch (state_) {
            case State::IDLE:    return runIdle(ch5_high, ch5_low, flags);
            case State::TAKEOFF: return runTakeoff(ch5_high, current_z);
            case State::HOVER:   return runHover(ch5_low, trigger_rising, override_active, current_z);
            case State::TRAJECTORY: return runTrajectory(ch5_low, override_active);
            case State::LANDING: return runLanding(current_z, flags);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_WARN("SkillExecutor: HALTED → 中止上锁");
        disarmAndIdle();  // 外部 halt（重新解锁/手动模式）：上锁+复位，但不置锁存
    }

private:
    enum class State { IDLE, TAKEOFF, HOVER, TRAJECTORY, LANDING };

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

        // CH5 上升沿（锁存已清）→ 起飞
        if (ch5_high && !latch) {
            ROS_INFO("SkillExecutor: CH5 上升沿 → 起飞");
            establishTaskFrame();   // 记录起飞点原点 + 起飞时机头朝向
            state_      = State::TAKEOFF;
            start_time_ = ros::Time::now();
            traj_index_ = 0;
            integral_   = 0.0;
            prev_error_ = 0.0;
            reached_since_ = ros::Time(0);
        }
        return BT::NodeStatus::RUNNING;
    }

    // ---- 起飞：轨迹 + PID + 异常 ----
    BT::NodeStatus runTakeoff(bool ch5_high, double current_z)
    {
        // 1. CH5 拨低 → 中止
        if (!ch5_high) {
            ROS_WARN("SkillExecutor: 起飞中 CH5 拨低 → 中止上锁");
            abort();
            return BT::NodeStatus::RUNNING;
        }

        double elapsed = (ros::Time::now() - start_time_).toSec();

        // 2. 超时 → 中止
        if (elapsed > meta_.timeout) {
            ROS_WARN("SkillExecutor: 超时 (%.1fs > %.1fs) → 中止上锁", elapsed, meta_.timeout);
            abort();
            return BT::NodeStatus::RUNNING;
        }

        // 2.5 提前到高度即悬停：current_z ≥ hover_reach_threshold 持续 hold_time 就提前切，
        //     不等轨迹跑满（治「时间驱动一路冲到 1.5m」的超调）
        double reach_threshold = hover_reach_threshold_;
        if (current_z >= reach_threshold) {
            if (reached_since_ == ros::Time(0)) reached_since_ = ros::Time::now();
            if ((ros::Time::now() - reached_since_).toSec() >= hover_reach_hold_time_) {
                ROS_INFO("SkillExecutor: 提前到达目标高度 (z=%.2fm, 阈值=%.2fm) → 悬停 (上位机 ANGLE 定高)",
                         current_z, reach_threshold);
                enterHover();
                return BT::NodeStatus::RUNNING;
            }
        } else {
            reached_since_ = ros::Time(0);
        }

        // 3. 轨迹完成 → 悬停（需实际高度到位，否则按超时中止）
        if (traj_index_ >= trajectory_.time.size()) {
            double final_error = std::abs(meta_.target_z - current_z);
            if (final_error <= meta_.tolerance_high) {
                ROS_INFO("SkillExecutor: 轨迹完成 → 悬停 (z=%.2fm, 上位机 ANGLE 定高)", current_z);
                enterHover();
            } else {
                // 轨迹时间走完但没爬到目标高度（拆桨/卡住）→ 等同超时，中止上锁
                ROS_WARN("SkillExecutor: 轨迹走完未达目标高度 (z=%.2fm, target=%.2fm) → 中止上锁",
                         current_z, meta_.target_z);
                abort();
            }
            return BT::NodeStatus::RUNNING;
        }

        // 4. 推进轨迹
        double t_target = trajectory_.time[traj_index_];
        if (elapsed >= t_target) {
            double base_throttle = trajectory_.throttle[traj_index_];
            // 期望高度：由油门归一化导出，与轨迹曲线同形。
            // 线性轨迹时等价于按时间线性；S 曲线轨迹时 PID 参考高度跟随 S 曲线，与开环油门一致。
            double thr_min  = 1000.0;
            double thr_peak = trajectory_.throttle.empty() ? 1000.0 : trajectory_.throttle.back();
            double norm     = std::max(0.0, (base_throttle - thr_min) / std::max(1.0, thr_peak - thr_min));
            double desired_z = meta_.target_z * norm;
            double error = desired_z - current_z;
            double abs_err = std::abs(error);

            // 三级判定（DANGER 由 enable_height_monitor 门控）
            if (enable_height_monitor_ && abs_err > meta_.tolerance_high) {
                ROS_ERROR("SkillExecutor: DANGER |error|=%.2fm > %.2fm (z=%.2f, desired=%.2f) → 中止上锁",
                          abs_err, meta_.tolerance_high, current_z, desired_z);
                abort();
                return BT::NodeStatus::RUNNING;
            }

            // PID 修正（仅油门通道）
            double derivative = (error - prev_error_) / trajectory_.interval;
            integral_ += error * trajectory_.interval;
            double correction = kp_ * error + ki_ * integral_ + kd_ * derivative;
            double output_throttle = std::max(1000.0, std::min(2000.0, base_throttle + correction));

            // 写通道
            setChannel(2, static_cast<int>(output_throttle));                    // CH3 油门
            setChannel(0, static_cast<int>(trajectory_.roll[traj_index_]));      // CH1 roll
            setChannel(1, static_cast<int>(trajectory_.pitch[traj_index_]));     // CH2 pitch
            setChannel(3, static_cast<int>(trajectory_.yaw[traj_index_]));       // CH4 yaw

            if (abs_err <= meta_.tolerance_low)
                ROS_DEBUG_THROTTLE(2.0, "SkillExecutor: NORMAL t=%.2f z=%.2f err=%.2f thr=%.0f",
                                   elapsed, current_z, error, output_throttle);
            else
                ROS_INFO_THROTTLE(1.0, "SkillExecutor: DEVIATED t=%.2f z=%.2f err=%.2f thr=%.0f",
                                  elapsed, current_z, error, output_throttle);

            prev_error_ = error;
            traj_index_++;
        }

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

        // 悬停安全限幅：跟踪误差/绝对高度越界 → 降落（或上锁）
        if (checkHoverSafety(p_M, current_z)) {
            return BT::NodeStatus::RUNNING;   // 已切换 LANDING / abort
        }

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

    // ---- 进入悬停：悬停目标 = 当前位置（起飞点机头系下的绝对点） ----
    void enterHover()
    {
        Eigen::Vector3d p_M, v_M;
        readState(p_M, v_M);   // 失败时 p_M 为 0，后续 runHover 会因闭环失效 abort
        hover_target_ = p_M;
        hover_entered_since_ = ros::Time::now();
        hover_danger_since_ = ros::Time(0);
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
        ROS_INFO("SkillExecutor: 任务系已建立 (原点=起飞点, x=起飞时机头, z=重力参考 roll/pitch, yaw0=%.2f rad)",
                 yaw0);
    }

    // ---- 读取任务系测量（水平 VINS + 垂直高度源相对值）；闭环失效返回 false ----
    bool readState(Eigen::Vector3d& p_M, Eigen::Vector3d& v_M)
    {
        const bool closed = (feedback_mode_ == "closed_loop");
        auto& rb = RosBackend::instance();
        p_M.setZero(); v_M.setZero();
        bool odom_ok = false; double odom_age = 1e9;

        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            if (rb.hasOdom()) {
                p_M = R_WM_.transpose() * (rb.odomPos() - p0_W_);
                // 水平速度：速度估计器（飞控加速度计积分 + VINS 位置差分去漂移），替代 odomVel(恒0)
                Eigen::Vector2d vxy = rb.estVelXY();
                v_M.x() = vxy.x();
                v_M.y() = vxy.y();
                odom_age = (ros::Time::now() - rb.odomStamp()).toSec();
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

        if (closed && (!odom_ok || odom_age > odom_stale_timeout_)) {
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
        integral_   = 0.0;
        prev_error_ = 0.0;
        landed_since_ = ros::Time(0);
        trigger_since_ = ros::Time(0);
        arrived_since_ = ros::Time(0);
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
    double   prev_error_, integral_;

    State    state_;
    bool     enable_height_monitor_;
    double   landing_height_threshold_;
    double   landing_hold_time_;
    double   hover_reach_hold_time_;
    double   kp_, ki_, kd_;
    double   hover_reach_threshold_;
    ros::Time landed_since_;
    ros::Time reached_since_;

    // ---- 轨迹跟踪（最小 snap）成员 ----
    std::string feedback_mode_;
    Eigen::Vector3d target_;
    std::string duration_mode_;
    double fixed_duration_;
    double cruise_speed_, T_min_, T_max_;
    double arrive_tol_, vel_tol_, arrive_hold_;
    double danger_tol_, timeout_traj_, odom_stale_timeout_;
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
