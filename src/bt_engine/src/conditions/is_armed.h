#pragma once
#include <behaviortree_cpp/condition_node.h>
#include "../ros_backend.h"

class IsArmed : public BT::ConditionNode
{
public:
    IsArmed(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto& rb = RosBackend::instance();
        std::lock_guard<std::mutex> lock(rb.mutex());
        // ARMED = bit2 = 0x04（INAV runtime_config.h: ARMED = (1 << 2)）
        return (rb.armingFlags() & 0x04u) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
