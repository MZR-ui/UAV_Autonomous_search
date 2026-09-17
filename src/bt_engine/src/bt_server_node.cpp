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

    // ---- 轨迹跟踪参数 ----
    std::string feedback_mode = "closed_loop";
    double target_x = 2.0, target_y = 0.0, target_z = 1.0;
    std::string duration_mode = "auto";
    double fixed_duration = 0.0;
    double cruise_speed = 0.5, T_min = 2.0, T_max = 8.0;
    double arrive_tol = 0.1, vel_tol = 0.1, arrive_hold = 1.0;
    double danger_tol = 0.8, timeout_traj = 12.0, odom_stale_timeout = 0.3;
    int ch6_mid_low = 1400, ch6_mid_high = 1600;
    double ch6_debounce = 0.3;
    bool gimbal_lock = true;
    std::string height_source = "rangefinder";
    double hover_hold_before_traj = 5.0;
    int override_threshold = 10;
    double kp_xy = 1.0, kp_z = 2.0, kd_xy = 0.5, kd_z = 0.8;
    double gravity = 9.81, angle_max = 0.35;
    double hover_throttle = 1420.0, k_thr = 30.0;
    double pitch_sign = -1.0, roll_sign = 1.0;
    bool yaw_enabled = false;
    double yaw_kp = 0.0;

    nh_priv.param<std::string>("feedback_mode", feedback_mode, "closed_loop");
    nh_priv.param<double>("target_x", target_x, 2.0);
    nh_priv.param<double>("target_y", target_y, 0.0);
    nh_priv.param<double>("target_z", target_z, 1.0);
    nh_priv.param<std::string>("duration_mode", duration_mode, "auto");
    nh_priv.param<double>("fixed_duration", fixed_duration, 0.0);
    nh_priv.param<double>("cruise_speed", cruise_speed, 0.5);
    nh_priv.param<double>("T_min", T_min, 2.0);
    nh_priv.param<double>("T_max", T_max, 8.0);
    nh_priv.param<double>("arrive_tol", arrive_tol, 0.1);
    nh_priv.param<double>("vel_tol", vel_tol, 0.1);
    nh_priv.param<double>("arrive_hold", arrive_hold, 1.0);
    nh_priv.param<double>("danger_tol", danger_tol, 0.8);
    nh_priv.param<double>("timeout_traj", timeout_traj, 12.0);
    nh_priv.param<double>("odom_stale_timeout", odom_stale_timeout, 0.3);
    nh_priv.param<int>("ch6_mid_low", ch6_mid_low, 1400);
    nh_priv.param<int>("ch6_mid_high", ch6_mid_high, 1600);
    nh_priv.param<double>("ch6_debounce", ch6_debounce, 0.3);
    nh_priv.param<bool>("gimbal_lock", gimbal_lock, true);
    nh_priv.param<std::string>("height_source", height_source, "rangefinder");
    nh_priv.param<double>("hover_hold_before_traj", hover_hold_before_traj, 5.0);
    nh_priv.param<int>("override_threshold", override_threshold, 10);
    nh_priv.param<double>("kp_xy", kp_xy, 1.0);
    nh_priv.param<double>("kp_z", kp_z, 2.0);
    nh_priv.param<double>("kd_xy", kd_xy, 0.5);
    nh_priv.param<double>("kd_z", kd_z, 0.8);
    nh_priv.param<double>("gravity", gravity, 9.81);
    nh_priv.param<double>("angle_max", angle_max, 0.35);
    nh_priv.param<double>("hover_throttle", hover_throttle, 1420.0);
    nh_priv.param<double>("k_thr", k_thr, 30.0);
    nh_priv.param<double>("pitch_sign", pitch_sign, -1.0);
    nh_priv.param<double>("roll_sign", roll_sign, 1.0);
    nh_priv.param<bool>("yaw_enabled", yaw_enabled, false);
    nh_priv.param<double>("yaw_kp", yaw_kp, 0.0);

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

    // ---- 轨迹跟踪参数 ----
    tree.rootBlackboard()->set("feedback_mode", feedback_mode);
    tree.rootBlackboard()->set("target_x", target_x);
    tree.rootBlackboard()->set("target_y", target_y);
    tree.rootBlackboard()->set("target_z", target_z);
    tree.rootBlackboard()->set("duration_mode", duration_mode);
    tree.rootBlackboard()->set("fixed_duration", fixed_duration);
    tree.rootBlackboard()->set("cruise_speed", cruise_speed);
    tree.rootBlackboard()->set("T_min", T_min);
    tree.rootBlackboard()->set("T_max", T_max);
    tree.rootBlackboard()->set("arrive_tol", arrive_tol);
    tree.rootBlackboard()->set("vel_tol", vel_tol);
    tree.rootBlackboard()->set("arrive_hold", arrive_hold);
    tree.rootBlackboard()->set("danger_tol", danger_tol);
    tree.rootBlackboard()->set("timeout_traj", timeout_traj);
    tree.rootBlackboard()->set("odom_stale_timeout", odom_stale_timeout);
    tree.rootBlackboard()->set("ch6_mid_low", ch6_mid_low);
    tree.rootBlackboard()->set("ch6_mid_high", ch6_mid_high);
    tree.rootBlackboard()->set("ch6_debounce", ch6_debounce);
    tree.rootBlackboard()->set("gimbal_lock", gimbal_lock);
    tree.rootBlackboard()->set("height_source", height_source);
    tree.rootBlackboard()->set("hover_hold_before_traj", hover_hold_before_traj);
    tree.rootBlackboard()->set("override_threshold", override_threshold);
    tree.rootBlackboard()->set("kp_xy", kp_xy);
    tree.rootBlackboard()->set("kp_z", kp_z);
    tree.rootBlackboard()->set("kd_xy", kd_xy);
    tree.rootBlackboard()->set("kd_z", kd_z);
    tree.rootBlackboard()->set("gravity", gravity);
    tree.rootBlackboard()->set("angle_max", angle_max);
    tree.rootBlackboard()->set("hover_throttle", hover_throttle);
    tree.rootBlackboard()->set("k_thr", k_thr);
    tree.rootBlackboard()->set("pitch_sign", pitch_sign);
    tree.rootBlackboard()->set("roll_sign", roll_sign);
    tree.rootBlackboard()->set("yaw_enabled", yaw_enabled);
    tree.rootBlackboard()->set("yaw_kp", yaw_kp);
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
