#pragma once
#include <behaviortree_cpp/action_node.h>
#include <vector>
#include <ros/ros.h>

/**
 * @brief 通用通道写入节点
 *
 * Ports:
 *   channel (int):    通道索引 (0-15)
 *   value   (int):    目标 PWM 值
 */
class SetChannel : public BT::SyncActionNode
{
public:
    SetChannel(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("channel"), BT::InputPort<int>("value") };
    }

    BT::NodeStatus tick() override
    {
        int channel = 0, value = 1500;
        if (!getInput("channel", channel) || !getInput("value", value))
            return BT::NodeStatus::FAILURE;

        if (channel < 0 || channel > 15)
            return BT::NodeStatus::FAILURE;

        // 从 Blackboard 读取 channels 数组，修改后写回
        std::vector<uint16_t> channels;
        auto bb = config().blackboard;
        if (bb->get("channels", channels)) {
            if (static_cast<size_t>(channel) < channels.size())
                channels[channel] = static_cast<uint16_t>(value);
            bb->set("channels", channels);
        }

        ROS_DEBUG("SetChannel: ch[%d] = %d", channel, value);
        return BT::NodeStatus::SUCCESS;
    }
};
