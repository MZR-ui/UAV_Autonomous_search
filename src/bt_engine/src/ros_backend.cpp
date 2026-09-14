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
