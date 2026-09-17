#pragma once
#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include "../skill_loader.h"
#include "../ros_backend.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "../trajectory/min_snap.h"
#include "../trajectory/position_controller.h"

/**
 * @brief 起飞技能执行器：IDLE → TAKEOFF → HOVER → LANDING 生命周期状态机
 *
 * 进自动模式即常驻（onStart 返回 RUNNING 进入 IDLE），自行拥有 CH5 的全部语义：
 *   IDLE    未解锁时什么都不写（等 arm_action）；已解锁写怠速通道；
 *           检测 CH5 上升沿(>1750) 且锁存已清 → TAKEOFF
 *   TAKEOFF 跑轨迹 + PID；CH5 拨低 / 超时 / DANGER → abort()（上锁+回 IDLE）
 *   HOVER   写 CH6=2000（POSHOLD+SURFACE）；CH5 拨低 → LANDING
 *   LANDING 写 CH7=2000（RTH，无 GPS 时飞控自动降落）；检测着陆 → abort()（上锁+回 IDLE）
 *
 * 锁存 ch5_release_required 由本节点统一拥有：abort() 置位，runIdle 检测 CH5 回落清零。
 * arm_action 只读该锁存（置位时跳过重新解锁），从而避免「中止→立即重解锁」的循环。
 *
 * Ports:
 *   skill (string):  技能名（对应 skills/ 子目录）
 *
 * Blackboard I/O:
 *   读: "channels"、参数 enable_height_monitor / landing_height_threshold / landing_hold_time
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
        , ch6_mid_since_(ros::Time(0))
        , arrived_since_(ros::Time(0))
        , height_source_("rangefinder"), h0_(0.0)
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
        // 快照遥测数据（读取输入 CH5、测距高度、armingFlags）
        auto& rb = RosBackend::instance();
        bool ch5_high = false, ch5_low = false, ch6_mid = false, override_active = false;
        double current_z = 0.0;
        uint32_t flags = 0;
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            const auto& ch = rb.remoteChannels();
            uint16_t ch5 = (ch.size() > 4) ? ch[4] : 0;
            uint16_t ch6 = (ch.size() > 5) ? ch[5] : 0;
            ch5_high = (ch5 > 1750);
            ch5_low  = (ch5 < 1750);
            ch6_mid  = (ch6 >= ch6_mid_low_ && ch6 <= ch6_mid_high_);
            override_active = lateralOverrideDetected(ch);
            current_z = rb.rangefinder();
            flags = rb.armingFlags();
        }

        switch (state_) {
            case State::IDLE:    return runIdle(ch5_high, ch5_low, flags);
            case State::TAKEOFF: return runTakeoff(ch5_high, current_z);
            case State::HOVER:   return runHover(ch5_low, ch6_mid, override_active);
            case State::TRAJECTORY: return runTrajectory(ch6_mid, override_active);
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

    // ---- 悬停：上位机 ANGLE 定高定点，检测降落触发 / CH6 中档轨迹触发 ----
    BT::NodeStatus runHover(bool ch5_low, bool ch6_mid, bool override_active)
    {
        // 遥控 CH6 中档 → 触发轨迹跟踪（去抖 + 需悬停稳定达标）
        if (ch6_mid) {
            if (ch6_mid_since_ == ros::Time(0)) ch6_mid_since_ = ros::Time::now();
            double stable = (ros::Time::now() - hover_entered_since_).toSec();
            if ((ros::Time::now() - ch6_mid_since_).toSec() >= ch6_debounce_
                && stable >= hover_hold_before_traj_) {
                if (enterTrajectory())
                    return BT::NodeStatus::RUNNING;
            }
        } else {
            ch6_mid_since_ = ros::Time(0);
        }

        // CH5 拨低 → 降落
        if (ch5_low) {
            ROS_INFO("SkillExecutor: 悬停后 CH5 拨低 → 降落 (CH7=2000 RTH)");
            state_ = State::LANDING;
            setChannel(2, 1000);  // CH3 油门最低（降落时不再保持悬停油门）
            setChannel(5, 1000);  // CH6 降到 ANGLE（RTH 未接管时的安全后备）
            setChannel(6, 2000);  // CH7 RTH（无 GPS 时飞控自动降落）
            landed_since_ = ros::Time(0);
            return BT::NodeStatus::RUNNING;
        }

        // 位置控制：悬停定高定点（目标 = hover_target_）
        Eigen::Vector3d p_M, v_M;
        if (!readState(p_M, v_M)) {
            abort();
            return BT::NodeStatus::RUNNING;
        }
        applyController(hover_target_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                        p_M, v_M, override_active);
        return BT::NodeStatus::RUNNING;
    }

    // ---- 轨迹跟踪：最小 snap + 位置控制器 ----
    BT::NodeStatus runTrajectory(bool ch6_mid, bool override_active)
    {
        const bool closed = (feedback_mode_ == "closed_loop");

        // 1. CH6 出中档带 → 中止（降落+上锁）
        if (!ch6_mid) {
            ROS_WARN("SkillExecutor: 轨迹跟踪中 CH6 出中档 → 中止上锁");
            abort();
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
            double to_target = (p_M - target_).norm();
            done = (elapsed >= traj_.totalTime()) && (to_target <= arrive_tol_) && (v_M.norm() <= vel_tol_);
        } else {
            done = (elapsed >= traj_.totalTime());
        }

        if (done) {
            if (arrived_since_ == ros::Time(0)) arrived_since_ = ros::Time::now();
            if ((ros::Time::now() - arrived_since_).toSec() >= arrive_hold_) {
                ROS_INFO("SkillExecutor: 到达目标 (%.2f,%.2f,%.2f) → 回上位机 ANGLE 悬停",
                         p_M.x(), p_M.y(), p_M.z());
                hover_target_ = target_;
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

    // ---- 进入悬停：记录任务系原点、高度源基准 ----
    void enterHover()
    {
        auto& rb = RosBackend::instance();
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            if (rb.hasOdom()) {
                p0_W_ = rb.odomPos();
                buildTaskFrame(rb.odomQuat());
            } else {
                p0_W_ = Eigen::Vector3d::Zero();
                R_WM_ = Eigen::Matrix3d::Identity();
            }
            // 高度源基准 h0（vins 源用任务系 z，无需额外基准）
            if (height_source_ == "barometer")      h0_ = rb.altitude();
            else if (height_source_ != "vins")      h0_ = rb.rangefinder();
            else                                     h0_ = 0.0;
        }
        hover_target_ = Eigen::Vector3d::Zero();
        hover_entered_since_ = ros::Time::now();
        state_ = State::HOVER;
        ROS_INFO("SkillExecutor: 进入悬停 height_source=%s h0=%.2fm", height_source_.c_str(), h0_);
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
                v_M = R_WM_.transpose() * rb.odomVel();
                odom_age = (ros::Time::now() - rb.odomStamp()).toSec();
                odom_ok = true;
            }
            // 垂直用高度源相对值（rangefinder/barometer 覆盖 VINS z）
            if (height_source_ == "barometer") {
                p_M.z() = rb.altitude() - h0_;
                v_M.z() = 0.0;
            } else if (height_source_ != "vins") {   // 默认 rangefinder
                p_M.z() = rb.rangefinder() - h0_;
                v_M.z() = 0.0;
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
        setChannel(0, out.roll_pwm);      // CH1 roll
        setChannel(1, out.pitch_pwm);     // CH2 pitch
        setChannel(2, out.throttle_pwm);  // CH3 油门
        setChannel(3, out.yaw_pwm);       // CH4 yaw
        setChannel(5, 1000);              // CH6 ANGLE（全程）
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

    // ---- 进入轨迹跟踪：以当前悬停点为起点生成轨迹 ----
    bool enterTrajectory()
    {
        const bool closed = (feedback_mode_ == "closed_loop");

        // 轨迹起点 = 当前任务系位置（无跳变）
        Eigen::Vector3d p_start = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_dummy;
        if (closed && !readState(p_start, v_dummy)) {
            ROS_WARN("SkillExecutor: 触发轨迹但 VINS 无位姿 → 忽略，保持悬停");
            ch6_mid_since_ = ros::Time(0);
            return false;
        }

        const double T = planDuration((target_ - p_start).norm());
        traj_ = trajectory::MinSnapTrajectory::singleSegment(p_start, target_, T);

        traj_start_time_ = ros::Time::now();
        arrived_since_ = ros::Time(0);
        state_ = State::TRAJECTORY;

        if (gimbal_lock_)
            ROS_INFO("SkillExecutor: 轨迹跟踪开始，云台锁定（不发送云台指令）");
        ROS_INFO("SkillExecutor: 进入轨迹跟踪 start=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) T=%.2fs %s",
                 p_start.x(), p_start.y(), p_start.z(),
                 target_.x(), target_.y(), target_.z(), T, closed ? "closed_loop" : "open_loop");
        return true;
    }

    // 由 VINS 四元数构建任务系（近水平：机体 z ≈ 重力向上）
    void buildTaskFrame(const Eigen::Quaterniond& q)
    {
        Eigen::Matrix3d R_WB = q.toRotationMatrix();
        Eigen::Vector3d z_W = R_WB * Eigen::Vector3d::UnitZ();   // 机体 z ≈ 上
        z_W.normalize();
        Eigen::Vector3d x_W = R_WB * Eigen::Vector3d::UnitX();   // 机头
        x_W -= (x_W.dot(z_W)) * z_W;                              // 投影到水平面
        if (x_W.norm() < 1e-6) {
            Eigen::Vector3d y_B = R_WB * Eigen::Vector3d::UnitY();
            x_W = y_B - (y_B.dot(z_W)) * z_W;
        }
        x_W.normalize();
        Eigen::Vector3d y_W = z_W.cross(x_W);                     // 左
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
        ch6_mid_since_ = ros::Time(0);
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
    ros::Time ch6_mid_since_;
    ros::Time arrived_since_;

    std::string height_source_;
    double h0_;
    double hover_hold_before_traj_;
    int    override_threshold_;
    Eigen::Vector3d hover_target_;
    ros::Time hover_entered_since_;

    trajectory::CtrlParams ctrl_params_;
    trajectory::MinSnapTrajectory traj_;
    Eigen::Vector3d p0_W_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_WM_ = Eigen::Matrix3d::Identity();
};
