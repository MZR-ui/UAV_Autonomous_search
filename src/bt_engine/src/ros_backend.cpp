#include "ros_backend.h"

RosBackend& RosBackend::instance()
{
    static RosBackend inst;
    return inst;
}

void RosBackend::init(ros::NodeHandle& nh)
{
    sub_remote_     = nh.subscribe("/remote_order", 1, &RosBackend::remoteCb, this);
    sub_status_     = nh.subscribe("/msp/status", 1, &RosBackend::statusCb, this);
    sub_altitude_   = nh.subscribe("/msp/altitude", 1, &RosBackend::altitudeCb, this);
    sub_rangefinder_ = nh.subscribe("/msp/rangefinder", 1, &RosBackend::rangefinderCb, this);
    sub_odom_       = nh.subscribe("/vins_estimator/odometry", 1, &RosBackend::odomCb, this);
    sub_attitude_   = nh.subscribe("/msp/attitude", 1, &RosBackend::attitudeCb, this);
}

void RosBackend::remoteCb(const remote_info::Remote::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    remote_channels_ = msg->channels;
}

void RosBackend::statusCb(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{
    // /msp/status 格式: [cycleTime, i2cErrors, sensorStatus, cpuLoad,
    //                    profileAndBatt, armingFlags]
    // 索引 5 = armingFlags (32-bit)
    std::lock_guard<std::mutex> lock(mtx_);
    if (msg->data.size() > 5)
        arming_flags_ = msg->data[5];
}

void RosBackend::altitudeCb(const geometry_msgs::Point::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    altitude_ = msg->z;
}

void RosBackend::rangefinderCb(const geometry_msgs::Point::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    rangefinder_alt_ = msg->z;
}

void RosBackend::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_pos_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
    odom_vel_ = Eigen::Vector3d(msg->twist.twist.linear.x,
                                msg->twist.twist.linear.y,
                                msg->twist.twist.linear.z);
    odom_quat_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);
    odom_stamp_ = ros::Time::now();   // 用接收时刻判 stale（VINS 消息戳可能延迟）
    odom_valid_ = true;
}

void RosBackend::attitudeCb(const msp_interface::Attitude::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    attitude_roll_  = msg->roll;
    attitude_pitch_ = msg->pitch;
    attitude_yaw_   = msg->yaw;
}
