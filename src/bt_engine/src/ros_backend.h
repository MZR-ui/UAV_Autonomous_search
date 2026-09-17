#pragma once

#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <remote_info/Remote.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <msp_interface/Attitude.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

    // VINS odometry（无 IMU 纯双目）
    bool hasOdom() const { return odom_valid_; }
    const ros::Time& odomStamp() const { return odom_stamp_; }
    const Eigen::Vector3d& odomPos() const { return odom_pos_; }
    const Eigen::Vector3d& odomVel() const { return odom_vel_; }
    const Eigen::Quaterniond& odomQuat() const { return odom_quat_; }
    // 飞控姿态（重力参考）
    double attitudeRoll() const { return attitude_roll_; }
    double attitudePitch() const { return attitude_pitch_; }
    double attitudeYaw() const { return attitude_yaw_; }

private:
    RosBackend() = default;

    // 订阅回调
    void remoteCb(const remote_info::Remote::ConstPtr& msg);
    void statusCb(const std_msgs::UInt32MultiArray::ConstPtr& msg);
    void altitudeCb(const geometry_msgs::Point::ConstPtr& msg);
    void rangefinderCb(const geometry_msgs::Point::ConstPtr& msg);
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void attitudeCb(const msp_interface::Attitude::ConstPtr& msg);

    ros::Subscriber sub_remote_, sub_status_, sub_altitude_, sub_rangefinder_;
    ros::Subscriber sub_odom_, sub_attitude_;

    std::mutex mtx_;
    std::vector<uint16_t> remote_channels_;
    uint32_t arming_flags_ = 0;
    double altitude_ = 0.0;
    double rangefinder_alt_ = 0.0;

    bool odom_valid_ = false;
    ros::Time odom_stamp_;
    Eigen::Vector3d odom_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odom_vel_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond odom_quat_ = Eigen::Quaterniond::Identity();
    double attitude_roll_ = 0.0;
    double attitude_pitch_ = 0.0;
    double attitude_yaw_ = 0.0;
};
