#pragma once
#include <behaviortree_cpp/condition_node.h>
#include "../ros_backend.h"

/**
 * @brief 起飞触发条件：输入 CH5 > 1750（上升沿 + 释放锁存）
 *
 * 自动模式下解锁完成后，CH5 由「解锁」语义切换为「起飞 & 降落触发」。
 * 为避免中止后（尤其超时/DANGER 时 CH5 仍高位）立刻重触发，引入锁存：
 *   中止动作会置 blackboard 的 ch5_release_required=true，
 *   本节点在 CH5>1750 且锁存时返回 FAILURE，要求 CH5 先回落（<1750）清锁存。
 *
 * Blackboard I/O:
 *   读/写: "ch5_release_required" (bool)
 */
class TakeoffTrigger : public BT::ConditionNode
{
public:
    TakeoffTrigger(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        auto& rb = RosBackend::instance();
        bool high = false;
        {
            std::lock_guard<std::mutex> lock(rb.mutex());
            const auto& ch = rb.remoteChannels();
            high = (ch.size() > 4 && ch[4] > 1750);
        }

        auto bb = config().blackboard;
        bool release_required = false;
        bb->get("ch5_release_required", release_required);

        if (!high) {
            // CH5 低位：清锁存
            bb->set("ch5_release_required", false);
            return BT::NodeStatus::FAILURE;
        }

        // CH5 高位
        if (release_required) {
            return BT::NodeStatus::FAILURE;  // 需先回落
        }
        return BT::NodeStatus::SUCCESS;
    }
};
