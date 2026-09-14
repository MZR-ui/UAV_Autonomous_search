#pragma once
#include <behaviortree_cpp/condition_node.h>
#include "../ros_backend.h"

class InAutoMode : public BT::ConditionNode
{
public:
    InAutoMode(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto& rb = RosBackend::instance();
        std::lock_guard<std::mutex> lock(rb.mutex());
        const auto& ch = rb.remoteChannels();
        // CH8 >= 1350 → 自动模式
        return (ch.size() > 7 && ch[7] >= 1350) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};
