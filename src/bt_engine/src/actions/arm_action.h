#pragma once
#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>
#include <vector>
#include "../ros_backend.h"

/**
 * @brief 解锁动作：发送解锁命令并等待解锁确认
 *
 * INAV armingFlags（MSP2_INAV_STATUS 0x2000，32-bit）位定义见
 *   src/main/fc/runtime_config.h 的 armingFlag_e：
 *     - ARMED           = (1 << 2) = 0x04   (bit2)
 *     - WAS_EVER_ARMED  = (1 << 3) = 0x08   (bit3)
 *     - 阻塞位从 bit6 起（ARMING_DISABLED_GEOZONE = 0x40），bit7 = FAILSAFE_SYSTEM …
 *
 * 解锁前置（GPS 无信号时）：
 *   navigation.c 的 navigationIsBlockingArming() 在「无健康位置估计」时，
 *   若 nav_extra_arming_safety == ALLOW_BYPASS 且 yaw 打满右（YAW_HI），
 *   则放行（返回 NONE）。因此解锁期间必须把 CH4 偏航打到 2000。
 *
 * 安全注意：解锁确认后必须立刻把偏航回到 1500（releaseYaw），
 *   否则飞控会把「偏航最大」当作满偏航速率指令执行，导致电机高速旋转（危险）。
 */
class ArmAction : public BT::StatefulActionNode
{
public:
    ArmAction(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config), start_time_(ros::Time(0)) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        // 中止后锁存：需 CH5 先回落（由 skill_executor 在 IDLE 态清零），否则跳过重新解锁，
        // 避免「中止→立即重解锁」的循环（超时/DANGER 时 CH5 仍高位）
        auto bb = config().blackboard;
        bool release_required = false;
        bb->get("ch5_release_required", release_required);
        if (release_required) {
            return BT::NodeStatus::SUCCESS;
        }

        uint32_t flags = readArmingFlags();
        if (flags & kArmingArmedMask) {
            return BT::NodeStatus::SUCCESS;  // 已解锁（稳态，不打日志避免刷屏）
        }

        start_time_ = ros::Time::now();
        ROS_INFO("ArmAction: 发送解锁命令 (flags=0x%x)", flags);
        applyArmCommand();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        uint32_t flags = readArmingFlags();
        if (flags & kArmingArmedMask) {
            ROS_INFO("ArmAction: 解锁已确认 (flags=0x%x)", flags);
            releaseYaw();  // 立刻释放偏航旁路，防止满偏航指令
            return BT::NodeStatus::SUCCESS;
        }

        if ((ros::Time::now() - start_time_).toSec() > kArmTimeout) {
            ROS_WARN("ArmAction: 解锁超时 (%.0fs, flags=0x%x)", kArmTimeout, flags);
            releaseYaw();  // 失败也释放偏航，避免残留
            return BT::NodeStatus::FAILURE;
        }

        applyArmCommand();  // 持续保持解锁命令 + 偏航最大右（旁路）
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ROS_WARN("ArmAction: HALTED → 上锁");
        setChannel(4, 1000);  // CH5 上锁
        setChannel(3, 1500);  // CH4 偏航回中
    }

private:
    // ARMED = bit2 = 0x04（runtime_config.h: ARMED = (1 << 2)）
    static constexpr uint32_t kArmingArmedMask = 0x04u;
    // 解锁超时 (s)，超时返回 FAILURE，由 ReactiveSequence 重试
    static constexpr double   kArmTimeout = 5.0;

    uint32_t readArmingFlags()
    {
        auto& rb = RosBackend::instance();
        std::lock_guard<std::mutex> lock(rb.mutex());
        return rb.armingFlags();
    }

    // 解锁命令：roll/pitch 中立 + 油门最低 + 偏航最大右（导航安全旁路）+ 解锁通道
    void applyArmCommand()
    {
        setChannel(0, 1500);  // CH1 roll 中立
        setChannel(1, 1500);  // CH2 pitch 中立
        setChannel(2, 1000);  // CH3 油门最低
        setChannel(3, 2000);  // CH4 偏航最大右（YAW_HI，旁路无 GPS 解锁）
        setChannel(4, 2000);  // CH5 解锁
        setChannel(5, 1000);  // CH6 飞行模式：低位 ANGLE
        setChannel(6, 1000);  // CH7 返航：关闭
    }

    // 解锁确认/失败后释放偏航，回到中立
    void releaseYaw()
    {
        setChannel(3, 1500);  // CH4 偏航回中
    }

    void setChannel(int index, uint16_t value)
    {
        std::vector<uint16_t> channels;
        auto bb = config().blackboard;
        if (bb->get("channels", channels)) {
            if (static_cast<size_t>(index) < channels.size())
                channels[index] = value;
            bb->set("channels", channels);
        }
    }

    ros::Time start_time_;
};
