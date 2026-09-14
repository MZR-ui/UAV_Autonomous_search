#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>
#include "msp_interface/MspChannel.h"
#include "ros_backend.h"
#include "skill_loader.h"

// Condition nodes
#include "conditions/is_armed.h"
#include "conditions/can_arm.h"
#include "conditions/in_auto_mode.h"
#include "conditions/is_flying.h"

// Action nodes
#include "actions/set_channel.h"
#include "actions/skill_executor.h"
#include "actions/arm_action.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bt_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // 参数
    double tick_rate = 100.0;
    std::string xml_path, skills_dir;
    bool enable_height_monitor = false;
    double landing_height_threshold = 0.1;
    double landing_hold_time = 2.0;
    double hover_reach_hold_time = 0.0;
    double kp = 200.0, ki = 10.0, kd = 0.0;
    double hover_reach_threshold = 0.9;
    int hover_ch3 = 1500, hover_ch6 = 2000;
    nh_priv.param<double>("tick_rate", tick_rate, 100.0);
    nh_priv.param<std::string>("xml_path", xml_path, "");
    nh_priv.param<std::string>("skills_dir", skills_dir, "");
    nh_priv.param<bool>("enable_height_monitor", enable_height_monitor, false);
    nh_priv.param<double>("landing_height_threshold", landing_height_threshold, 0.1);
    nh_priv.param<double>("landing_hold_time", landing_hold_time, 2.0);
    nh_priv.param<double>("hover_reach_hold_time", hover_reach_hold_time, 0.0);
    nh_priv.param<double>("kp", kp, 200.0);
    nh_priv.param<double>("ki", ki, 10.0);
    nh_priv.param<double>("kd", kd, 0.0);
    nh_priv.param<double>("hover_reach_threshold", hover_reach_threshold, 0.9);
    nh_priv.param<int>("hover_ch3", hover_ch3, 1500);
    nh_priv.param<int>("hover_ch6", hover_ch6, 2000);

    // 初始化 Backend
    RosBackend::instance().init(nh);
    ROS_INFO("BT Server: backend initialized");

    // 创建 BehaviorTreeFactory，注册所有节点
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<IsArmed>("is_armed");
    factory.registerNodeType<CanArm>("can_arm");
    factory.registerNodeType<InAutoMode>("in_auto_mode");
    factory.registerNodeType<IsFlying>("is_flying");
    factory.registerNodeType<SetChannel>("set_channel");
    factory.registerNodeType<SkillExecutor>("skill_executor");
    factory.registerNodeType<ArmAction>("arm_action");
    ROS_INFO("BT Server: 7 node types registered");

    // 加载 XML
    if (xml_path.empty()) {
        ROS_FATAL("BT Server: xml_path param not set!");
        return 1;
    }
    BT::Tree tree;
    try {
        tree = factory.createTreeFromFile(xml_path);
        ROS_INFO("BT Server: loaded %s", xml_path.c_str());
    } catch (const std::exception& e) {
        ROS_FATAL("BT Server: failed to load tree: %s", e.what());
        return 1;
    }

    // 初始化 Blackboard
    tree.rootBlackboard()->set("skills_dir", skills_dir);
    tree.rootBlackboard()->set("ch5_release_required", false);
    tree.rootBlackboard()->set("enable_height_monitor", enable_height_monitor);
    tree.rootBlackboard()->set("landing_height_threshold", landing_height_threshold);
    tree.rootBlackboard()->set("landing_hold_time", landing_hold_time);
    tree.rootBlackboard()->set("hover_reach_hold_time", hover_reach_hold_time);
    tree.rootBlackboard()->set("kp", kp);
    tree.rootBlackboard()->set("ki", ki);
    tree.rootBlackboard()->set("kd", kd);
    tree.rootBlackboard()->set("hover_reach_threshold", hover_reach_threshold);
    tree.rootBlackboard()->set("hover_ch3", hover_ch3);
    tree.rootBlackboard()->set("hover_ch6", hover_ch6);
    std::vector<uint16_t> channels(16, 1500);
    channels[0] = 1500;  // CH1 roll 中立
    channels[1] = 1500;  // CH2 pitch 中立
    channels[2] = 1000;  // CH3 油门最低（安全）
    channels[3] = 1500;  // CH4 偏航中立
    channels[4] = 1000;  // CH5 上锁
    channels[5] = 1000;  // CH6 飞行模式：低位 ANGLE（无导航辅助）
    channels[6] = 1000;  // CH7 返航：关闭
    tree.rootBlackboard()->set("channels", channels);

    // 发布器
    ros::Publisher control_pub = nh.advertise<msp_interface::MspChannel>("/control_data", 1);

    ros::Rate rate(tick_rate);
    ROS_INFO("BT Server: running at %.1f Hz", tick_rate);

    while (ros::ok()) {
        RosBackend::instance().spinOnce();

        // Tick BT（单次 tick，频率由 rate.sleep() 控制）
        // 注意：不能用 tickWhileRunning()，它会在 RUNNING 时阻塞主循环，
        // 导致 /control_data 直到动作结束才发布一次
        tree.tickOnce();

        // 发布 channels
        std::vector<uint16_t> out_channels;
        tree.rootBlackboard()->get("channels", out_channels);

        msp_interface::MspChannel msg;
        msg.header.stamp = ros::Time::now();
        msg.channels = out_channels;
        control_pub.publish(msg);

        rate.sleep();
    }

    return 0;
}
