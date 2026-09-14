#pragma once
#include <behaviortree_cpp/condition_node.h>
#include "../ros_backend.h"

class CanArm : public BT::ConditionNode
{
public:
    CanArm(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto& rb = RosBackend::instance();
        std::lock_guard<std::mutex> lock(rb.mutex());
        // 可解锁 = 无 ARMING_DISABLED_* 阻塞位（bit6+ 均为 0）
        // bit0-5 是状态位（ARMED=bit2, WAS_EVER_ARMED=bit3, SIMULATOR=bit4/5），不阻塞解锁。
        // 阻塞位从 bit6（GEOZONE）起，故掩码为 ~0x3F。
        return (rb.armingFlags() & ~0x3Fu) == 0
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
    }
};
