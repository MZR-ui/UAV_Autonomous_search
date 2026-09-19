#pragma once

#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <remote_info/Remote.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <msp_interface/Attitude.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "trajectory/velocity_estimator.h"

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

    /** 初始化所有订阅（在 ros::init 之后调用一次）；nh_priv 读速度估计器参数 */
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

    /** 在 BT tick 前调用一次，保证订阅回调被执行 */
    void spinOnce() { ros::spinOnce(); }

    // ---- 线程安全的数据读取接口 ----
    std::mutex& mutex() { return mtx_; }

    const std::vector<uint16_t>& remoteChannels() const { return remote_channels_; }
    uint32_t armingFlags() const { return arming_flags_; }
    double altitude() const { return altitude_; }
    double rangefinder() const { return rangefinder_alt_; }
    uint64_t triggerCount() const { return trigger_count_; }   // 轨迹触发信号计数（topic 触发源）

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

    // 速度估计器（一阶互补：飞控加速度计积分 + 测距仪/VINS 去漂移）
    double estVelZ() const { return vel_z_.value(); }
    Eigen::Vector2d estVelXY() const { return Eigen::Vector2d(vel_x_.value(), vel_y_.value()); }
    Eigen::Vector3d estVel() const { return Eigen::Vector3d(vel_x_.value(), vel_y_.value(), vel_z_.value()); }
    bool velEstEnabled() const { return vel_est_enabled_; }

    // 由 skill_executor 在起飞建立任务系时调用，供 XY 速度估计做帧对齐（内部自加锁）
    void setTaskFrame(const Eigen::Matrix3d& R_WM, const Eigen::Vector3d& p0_W, double yaw0);

private:
    RosBackend() = default;

    // 订阅回调
    void remoteCb(const remote_info::Remote::ConstPtr& msg);
    void statusCb(const std_msgs::UInt32MultiArray::ConstPtr& msg);
    void altitudeCb(const geometry_msgs::Point::ConstPtr& msg);
    void rangefinderCb(const geometry_msgs::Point::ConstPtr& msg);
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void attitudeCb(const msp_interface::Attitude::ConstPtr& msg);
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
    void triggerCb(const std_msgs::Empty::ConstPtr& msg);

    // 飞控机体系(FRD) → 世界系(ENU) 旋转矩阵（INAV roll/pitch/yaw 约定，装机前地面倾斜验证）
    Eigen::Matrix3d buildRenuBody(double roll, double pitch, double yaw) const;
    // 世界系(ENU) → 任务系(M) 水平旋转（绕 z 旋 -yaw0，z 不变）
    Eigen::Vector3d rotateEnuToM(const Eigen::Vector3d& v) const;
    // 调试话题：发布 a_lin(ENU) 与 v_est(M)
    void publishDebug(const Eigen::Vector3d& a_enU);

    ros::Subscriber sub_remote_, sub_status_, sub_altitude_, sub_rangefinder_;
    ros::Subscriber sub_odom_, sub_attitude_, sub_imu_, sub_trigger_;
    ros::Publisher pub_accel_lin_, pub_vel_est_;

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
    uint64_t trigger_count_ = 0;

    // ---- 速度估计器（一阶互补，XYZ）----
    bool   vel_est_enabled_  = true;
    double vel_est_alpha_z_  = 0.98;    // Z 互补权重（IMU 占比）
    double vel_est_alpha_xy_ = 0.95;    // XY 互补权重
    double vel_est_gravity_  = 9.80665; // 重力加速度幅值 (m/s²)
    Eigen::Vector3d g_world_ = Eigen::Vector3d(0, 0, -9.80665); // 世界系(ENU)重力向量
    trajectory::ComplementaryVelocityFilter vel_x_, vel_y_, vel_z_;
    ros::Time imu_stamp_;

    // 测距仪慢通道（Z）
    double rf_z_ = 0.0;
    ros::Time rf_stamp_;
    bool rf_z_valid_ = false;

    // 任务系（XY 帧对齐，skill_executor 起飞时设置）
    Eigen::Matrix3d R_WM_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p0_W_ = Eigen::Vector3d::Zero();
    double yaw0_ = 0.0;
    bool task_frame_set_ = false;
    // VINS 位置慢通道（XY）
    Eigen::Vector3d last_p_M_ = Eigen::Vector3d::Zero();
    ros::Time last_p_M_stamp_;
    bool last_p_M_valid_ = false;
};
