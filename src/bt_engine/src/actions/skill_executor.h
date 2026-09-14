#pragma once
#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include "../skill_loader.h"
#include "../ros_backend.h"

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
        , hover_ch3_(1500), hover_ch6_(2000)
        , landed_since_(ros::Time(0))
        , reached_since_(ros::Time(0))
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
        bb->get("hover_ch3", hover_ch3_);
        bb->get("hover_ch6", hover_ch6_);

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
        bool ch5_high = false, ch5_low = false;
        double current_z = 0.0;
        uint32_t flags = 0;
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            const auto& ch = rb.remoteChannels();
            uint16_t ch5 = (ch.size() > 4) ? ch[4] : 0;
            ch5_high = (ch5 > 1750);
            ch5_low  = (ch5 < 1750);
            current_z = rb.rangefinder();
            flags = rb.armingFlags();
        }

        switch (state_) {
            case State::IDLE:    return runIdle(ch5_high, ch5_low, flags);
            case State::TAKEOFF: return runTakeoff(ch5_high, current_z);
            case State::HOVER:   return runHover(ch5_low);
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
    enum class State { IDLE, TAKEOFF, HOVER, LANDING };

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
                ROS_INFO("SkillExecutor: 提前到达目标高度 (z=%.2fm, 阈值=%.2fm) → 悬停 (CH6=2000)",
                         current_z, reach_threshold);
                state_ = State::HOVER;
                setChannel(2, hover_ch3_);  // CH3 油门 → SURFACE 中位
                setChannel(5, hover_ch6_);  // CH6 POSHOLD+SURFACE
                return BT::NodeStatus::RUNNING;
            }
        } else {
            reached_since_ = ros::Time(0);
        }

        // 3. 轨迹完成 → 悬停（需实际高度到位，否则按超时中止）
        if (traj_index_ >= trajectory_.time.size()) {
            double final_error = std::abs(meta_.target_z - current_z);
            if (final_error <= meta_.tolerance_high) {
                ROS_INFO("SkillExecutor: 轨迹完成 → 悬停 (z=%.2fm, CH6=2000)", current_z);
                state_ = State::HOVER;
                setChannel(2, hover_ch3_);  // CH3 油门 → SURFACE 中位（目标高度 1m）
                setChannel(5, hover_ch6_);  // CH6 POSHOLD+SURFACE
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

    // ---- 悬停：保持 CH6 高位，检测降落触发 ----
    BT::NodeStatus runHover(bool ch5_low)
    {
        setChannel(5, hover_ch6_);  // CH6 保持 POSHOLD+SURFACE

        if (ch5_low) {
            ROS_INFO("SkillExecutor: 悬停后 CH5 拨低 → 降落 (CH7=2000 RTH)");
            state_ = State::LANDING;
            setChannel(2, 1000);  // CH3 油门最低（降落时不再保持悬停油门）
            setChannel(5, 1000);  // CH6 降到 ANGLE（RTH 未接管时的安全后备）
            setChannel(6, 2000);  // CH7 RTH（无 GPS 时飞控自动降落）
            landed_since_ = ros::Time(0);
        }
        return BT::NodeStatus::RUNNING;
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
    int      hover_ch3_, hover_ch6_;
    ros::Time landed_since_;
    ros::Time reached_since_;
};
