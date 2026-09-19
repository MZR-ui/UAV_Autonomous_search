#include "ros_backend.h"

RosBackend& RosBackend::instance()
{
    static RosBackend inst;
    return inst;
}

void RosBackend::init(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
    nh_priv.param<bool>("vel_est_enabled", vel_est_enabled_, true);
    nh_priv.param<double>("vel_est_alpha_z", vel_est_alpha_z_, 0.98);
    nh_priv.param<double>("vel_est_alpha_xy", vel_est_alpha_xy_, 0.95);
    nh_priv.param<double>("vel_est_gravity", vel_est_gravity_, 9.80665);
    g_world_ = Eigen::Vector3d(0.0, 0.0, -vel_est_gravity_);

    sub_remote_     = nh.subscribe("/remote_order", 1, &RosBackend::remoteCb, this);
    sub_status_     = nh.subscribe("/msp/status", 1, &RosBackend::statusCb, this);
    sub_altitude_   = nh.subscribe("/msp/altitude", 1, &RosBackend::altitudeCb, this);
    sub_rangefinder_ = nh.subscribe("/msp/rangefinder", 1, &RosBackend::rangefinderCb, this);
    sub_odom_       = nh.subscribe("/vins_estimator/odometry", 1, &RosBackend::odomCb, this);
    sub_attitude_   = nh.subscribe("/msp/attitude", 1, &RosBackend::attitudeCb, this);
    sub_imu_        = nh.subscribe("/msp/imu", 1, &RosBackend::imuCb, this);
    sub_trigger_    = nh.subscribe("/traj_trigger", 1, &RosBackend::triggerCb, this);

    // 调试话题：地面倾斜/方向验证用
    pub_accel_lin_ = nh.advertise<geometry_msgs::Vector3Stamped>("/bt_engine/accel_lin", 1);
    pub_vel_est_   = nh.advertise<geometry_msgs::Vector3Stamped>("/bt_engine/vel_est", 1);
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
    double z = msg->z;
    // 慢通道：测距仪差分 → 一阶互补去漂移（Z 速度环）
    if (vel_est_enabled_ && rf_z_valid_) {
        double dt = (ros::Time::now() - rf_stamp_).toSec();
        if (dt > 0.0 && dt < 0.5) {
            double v_pos = (z - rf_z_) / dt;
            vel_z_.correct(v_pos, vel_est_alpha_z_);
        }
    }
    rf_z_ = z;
    rf_stamp_ = ros::Time::now();
    rf_z_valid_ = true;
    rangefinder_alt_ = z;
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

    // XY 慢通道去漂移：VINS 位置差分（任务系 M 建立后才做）
    if (vel_est_enabled_ && task_frame_set_) {
        Eigen::Vector3d p_W(odom_pos_.x(), odom_pos_.y(), odom_pos_.z());
        Eigen::Vector3d p_M = R_WM_.transpose() * (p_W - p0_W_);
        if (last_p_M_valid_) {
            double dt = (odom_stamp_ - last_p_M_stamp_).toSec();
            if (dt > 0.0 && dt < 0.5) {
                vel_x_.correct((p_M.x() - last_p_M_.x()) / dt, vel_est_alpha_xy_);
                vel_y_.correct((p_M.y() - last_p_M_.y()) / dt, vel_est_alpha_xy_);
            }
        }
        last_p_M_ = p_M;
        last_p_M_stamp_ = odom_stamp_;
        last_p_M_valid_ = true;
    }
}

void RosBackend::attitudeCb(const msp_interface::Attitude::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    attitude_roll_  = msg->roll;
    attitude_pitch_ = msg->pitch;
    attitude_yaw_   = msg->yaw;
}

void RosBackend::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (!vel_est_enabled_) {
        imu_stamp_ = msg->header.stamp;
        return;
    }

    double dt = 0.0;
    if (!imu_stamp_.isZero())
        dt = (msg->header.stamp - imu_stamp_).toSec();

    // 快通道：机体系比力 → 世界系(ENU)，去重力，转任务系(M)，积分 XYZ
    Eigen::Vector3d a_body(msg->linear_acceleration.x,
                           msg->linear_acceleration.y,
                           msg->linear_acceleration.z);
    Eigen::Matrix3d R = buildRenuBody(attitude_roll_, attitude_pitch_, attitude_yaw_);
    // /msp/imu 报的是机体系“重力向量”（静止时 ≈ +g 沿 z 下），故 a_lin = g_world − R·a_body
    Eigen::Vector3d a_enU = g_world_ - (R * a_body);
    Eigen::Vector3d a_M = rotateEnuToM(a_enU);

    if (dt > 0.0 && dt < 0.1) {
        vel_x_.integrate(a_M.x(), dt);
        vel_y_.integrate(a_M.y(), dt);
        vel_z_.integrate(a_M.z(), dt);
    }

    // 调试话题：a_lin(ENU) + v_est(M)
    publishDebug(a_enU);

    imu_stamp_ = msg->header.stamp;
}

// 世界系(ENU: x北/y东/z上) → 任务系(M: x机头/y左/z上) 水平旋转。
// 起飞时记录机头航向 yaw0，ENU 水平向量绕 z 旋 -yaw0 即对齐任务系。
// 方向（左/右、正/负）需装机前地面“前推→+x、左推→+y”验证，必要时翻号。
Eigen::Vector3d RosBackend::rotateEnuToM(const Eigen::Vector3d& v) const
{
    double c = std::cos(yaw0_), s = std::sin(yaw0_);
    return Eigen::Vector3d(c * v.x() + s * v.y(),
                          -s * v.x() + c * v.y(),
                           v.z());
}

void RosBackend::publishDebug(const Eigen::Vector3d& a_enU)
{
    geometry_msgs::Vector3Stamped am, vm;
    ros::Time now = ros::Time::now();
    am.header.stamp = now;
    am.header.frame_id = "map";
    am.vector.x = a_enU.x();
    am.vector.y = a_enU.y();
    am.vector.z = a_enU.z();
    vm.header.stamp = now;
    vm.header.frame_id = "map";
    vm.vector.x = vel_x_.value();
    vm.vector.y = vel_y_.value();
    vm.vector.z = vel_z_.value();
    pub_accel_lin_.publish(am);
    pub_vel_est_.publish(vm);
}

void RosBackend::setTaskFrame(const Eigen::Matrix3d& R_WM, const Eigen::Vector3d& p0_W, double yaw0)
{
    std::lock_guard<std::mutex> lock(mtx_);
    R_WM_ = R_WM;
    p0_W_ = p0_W;
    yaw0_ = yaw0;
    task_frame_set_ = true;
    // 起飞瞬间无人机静止在地面：速度积分清零，避免地面搬运积累的漂移带进飞行
    vel_x_.reset();
    vel_y_.reset();
    vel_z_.reset();
    last_p_M_valid_ = false;   // 重置慢通道，避免起飞瞬间 VINS 差分跳变
}

// 飞控机体系(FRD: x前/y右/z下) → 世界系(ENU: x北/y东/z上)。
// INAV roll/pitch/yaw(rad) → earth(NED)→body(FRD) 旋转矩阵 R_ned，再 NED→ENU 翻转。
// 方向需装机前地面倾斜验证（静止放平/前倾/左倾 → a_lin≈0）。
Eigen::Matrix3d RosBackend::buildRenuBody(double roll, double pitch, double yaw) const
{
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);

    Eigen::Matrix3d R_ned;   // earth(NED) → body(FRD)
    R_ned << cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
             -sp,    cp*sr,             cp*cr;

    Eigen::Matrix3d D;       // NED → ENU（y、z 翻号）
    D << 1.0, 0.0, 0.0,
         0.0,-1.0, 0.0,
         0.0, 0.0,-1.0;

    return D * R_ned.transpose();   // body(FRD) → earth(ENU)
}

void RosBackend::triggerCb(const std_msgs::Empty::ConstPtr&)
{
    std::lock_guard<std::mutex> lock(mtx_);
    ++trigger_count_;
}
