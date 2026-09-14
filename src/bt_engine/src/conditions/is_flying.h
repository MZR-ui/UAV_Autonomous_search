#pragma once
#include <behaviortree_cpp/condition_node.h>
#include "../ros_backend.h"

class IsFlying : public BT::ConditionNode
{
public:
    IsFlying(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto& rb = RosBackend::instance();
        std::lock_guard<std::mutex> lock(rb.mutex());
        // 高度 > 0.1m 且在自动模式
        return (rb.altitude() > 0.1) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
