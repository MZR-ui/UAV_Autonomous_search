#pragma once

#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <remote_info/Remote.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Point.h>

/**
 * @brief 单例，为所有 BT 节点提供缓存的 ROS 遥测数据
 *
 * 在 ros_backend.cpp 中实现各话题订阅回调，
 * 数据受 mutex 保护。
 */
class RosBackend
{
public:
    static RosBackend& instance();

    /** 初始化所有订阅（在 ros::init 之后调用一次） */
    void init(ros::NodeHandle& nh);

    /** 在 BT tick 前调用一次，保证订阅回调被执行 */
    void spinOnce() { ros::spinOnce(); }

    // ---- 线程安全的数据读取接口 ----
    std::mutex& mutex() { return mtx_; }

    const std::vector<uint16_t>& remoteChannels() const { return remote_channels_; }
    uint32_t armingFlags() const { return arming_flags_; }
    double altitude() const { return altitude_; }
    double rangefinder() const { return rangefinder_alt_; }

private:
    RosBackend() = default;

    // 订阅回调
    void remoteCb(const remote_info::Remote::ConstPtr& msg);
    void statusCb(const std_msgs::UInt32MultiArray::ConstPtr& msg);
    void altitudeCb(const geometry_msgs::Point::ConstPtr& msg);
    void rangefinderCb(const geometry_msgs::Point::ConstPtr& msg);

    ros::Subscriber sub_remote_, sub_status_, sub_altitude_, sub_rangefinder_;

    std::mutex mtx_;
    std::vector<uint16_t> remote_channels_;
    uint32_t arming_flags_ = 0;
    double altitude_ = 0.0;
    double rangefinder_alt_ = 0.0;
};
