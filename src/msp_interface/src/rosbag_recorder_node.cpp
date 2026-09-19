#include <ros/ros.h>
#include <rosbag/bag.h>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt32MultiArray.h>
#include "msp_interface/MspChannel.h"
#include "msp_interface/EscTelem.h"
#include "msp_interface/Attitude.h"
#include "remote_info/Remote.h"
#include <boost/filesystem.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <csignal>

// SIGINT/SIGTERM 优雅退出标志：信号处理函数里只置位，主循环里检查后由析构 close() 收尾
static volatile sig_atomic_t g_shutdown_requested = 0;
static void handleShutdownSignal(int) { g_shutdown_requested = 1; }

class RosbagRecorderNode
{
public:
    RosbagRecorderNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), is_recording_(false), first_seen_(true),
          last_auto_mode_(false), last_should_record_(false),
          auto_restart_on_timeout_(false), record_realsense_(false), record_camera_infra_(true)
    {
        nh_priv_.param<std::string>("bag_save_dir", bag_save_dir_,
            std::string(getenv("HOME")) + "/lmw_catkin_ws/uav_ws/data/rosbag");
        nh_priv_.param<int>("max_bag_files", max_bag_files_, 100);
        nh_priv_.param<double>("max_record_duration", max_record_duration_, 600.0);
        nh_priv_.param<int>("ch5_threshold", ch5_threshold_, 1350);
        nh_priv_.param<int>("auto_mode_threshold", auto_mode_threshold_, 1350);
        nh_priv_.param<bool>("auto_restart_on_timeout", auto_restart_on_timeout_, true);
        nh_priv_.param<bool>("record_realsense", record_realsense_, true);
        nh_priv_.param<bool>("record_camera_infra", record_camera_infra_, true);

        // 遥控订阅（读出 CH5 触发录制）
        remote_sub_ = nh_.subscribe("/remote_order", 1, &RosbagRecorderNode::remoteCallback, this);

        // MSP 遥测订阅
        imu_sub_      = nh_.subscribe<sensor_msgs::Imu>("/msp/imu", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Imu>, this, "/msp/imu", _1));
        alt_sub_      = nh_.subscribe<geometry_msgs::Point>("/msp/altitude", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<geometry_msgs::Point>, this, "/msp/altitude", _1));
        range_sub_    = nh_.subscribe<geometry_msgs::Point>("/msp/rangefinder", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<geometry_msgs::Point>, this, "/msp/rangefinder", _1));
        gps_sub_      = nh_.subscribe<sensor_msgs::NavSatFix>("/msp/gps", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::NavSatFix>, this, "/msp/gps", _1));
        motor_sub_    = nh_.subscribe<sensor_msgs::JointState>("/msp/motor", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::JointState>, this, "/msp/motor", _1));
        status_sub_   = nh_.subscribe<std_msgs::UInt32MultiArray>("/msp/status", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<std_msgs::UInt32MultiArray>, this, "/msp/status", _1));
        battery_sub_  = nh_.subscribe<sensor_msgs::BatteryState>("/msp/battery", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::BatteryState>, this, "/msp/battery", _1));
        esc_sub_      = nh_.subscribe<msp_interface::EscTelem>("/msp/esc_telem", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<msp_interface::EscTelem>, this, "/msp/esc_telem", _1));
        attitude_sub_ = nh_.subscribe<msp_interface::Attitude>("/msp/attitude", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<msp_interface::Attitude>, this, "/msp/attitude", _1));
        ch_sub_       = nh_.subscribe<msp_interface::MspChannel>("/msp_channel", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<msp_interface::MspChannel>, this, "/msp_channel", _1));

        // VINS 位姿/路径 + Jetson 控制指令（轨迹分析用）
        vins_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vins_estimator/odometry", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<nav_msgs::Odometry>, this, "/vins_estimator/odometry", _1));
        vins_path_sub_ = nh_.subscribe<nav_msgs::Path>("/vins_estimator/path", 10,
            boost::bind(&RosbagRecorderNode::recordToBag<nav_msgs::Path>, this, "/vins_estimator/path", _1));
        control_sub_   = nh_.subscribe<msp_interface::MspChannel>("/control_data", 100,
            boost::bind(&RosbagRecorderNode::recordToBag<msp_interface::MspChannel>, this, "/control_data", _1));

        // RealSense 订阅（可选）
        if (record_realsense_) {
            sub_color_img_   = nh_.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Image>, this, "/camera/color/image_raw", _1));
            sub_color_info_  = nh_.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::CameraInfo>, this, "/camera/color/camera_info", _1));
            sub_depth_img_   = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Image>, this, "/camera/depth/image_rect_raw", _1));
            sub_depth_info_  = nh_.subscribe<sensor_msgs::CameraInfo>("/camera/depth/camera_info", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::CameraInfo>, this, "/camera/depth/camera_info", _1));
            sub_aligned_img_ = nh_.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Image>, this, "/camera/aligned_depth_to_color/image_raw", _1));
            sub_aligned_info_= nh_.subscribe<sensor_msgs::CameraInfo>("/camera/aligned_depth_to_color/camera_info", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::CameraInfo>, this, "/camera/aligned_depth_to_color/camera_info", _1));
        }

        // 红外双目 + IMU 订阅（可选，VIO 输入）
        if (record_camera_infra_) {
            sub_cam_imu_    = nh_.subscribe<sensor_msgs::Imu>("/camera/imu", 100,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Imu>, this, "/camera/imu", _1));
            sub_infra1_img_ = nh_.subscribe<sensor_msgs::Image>("/camera/infra1/image_rect_raw", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Image>, this, "/camera/infra1/image_rect_raw", _1));
            sub_infra2_img_ = nh_.subscribe<sensor_msgs::Image>("/camera/infra2/image_rect_raw", 10,
                boost::bind(&RosbagRecorderNode::recordToBag<sensor_msgs::Image>, this, "/camera/infra2/image_rect_raw", _1));
        }

        // 时长检查定时器
        check_timer_ = nh_.createTimer(ros::Duration(1.0), &RosbagRecorderNode::checkDuration, this);

        // 小 chunk 阈值：数据每 8KB 落成一个完整 chunk，硬 kill/断电也只丢最后 8KB，
        // 前面的 reindex 都能找回（默认 768KB 太大，中断即全丢）
        bag_.setChunkThreshold(8 * 1024);

        createBagDirectory();
        ROS_INFO("RosbagRecorder: dir=%s max_files=%d max_dur=%.0fs realsense=%d",
                 bag_save_dir_.c_str(), max_bag_files_, max_record_duration_, record_realsense_);
    }

    ~RosbagRecorderNode() { stopRecording(); }

private:
    // --- 遥控回调：录制状态机 ---
    // 手动模式(CH8<auto_mode_threshold_)：CH5 上升沿开始录、下降沿停
    // 自动模式(CH8>=auto_mode_threshold_)：进入即全程录，CH5 高低不影响（降落数据也在内）
    // 模式切换：先 close() 当前包（写索引截断），再按新模式规则决定开不开新包
    void remoteCallback(const remote_info::Remote::ConstPtr& msg)
    {
        bool start = false, stop = false;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (msg->channels.size() <= 7) return;  // 需要 CH5(4) 与 CH8(7)
            bool auto_mode = (msg->channels[7] >= auto_mode_threshold_);
            bool ch5_high  = (msg->channels[4] >= ch5_threshold_);
            bool should_record = auto_mode || ch5_high;  // 自动全程录；手动看 CH5

            if (first_seen_) {
                first_seen_ = false;
                last_auto_mode_      = auto_mode;
                last_should_record_  = should_record;
                if (should_record) start = true;  // 启动时已处于录制状态 → 立即开录
            } else {
                bool mode_changed = (auto_mode != last_auto_mode_);
                if (mode_changed) {
                    stop = true;                     // 模式切换 → 截断当前包
                    if (should_record) start = true; // 进入自动(或手动 CH5 仍高) → 开新包
                } else if (should_record != last_should_record_) {
                    if (should_record) start = true;
                    else               stop  = true;
                }
                last_auto_mode_      = auto_mode;
                last_should_record_  = should_record;
            }
        }
        if (stop)  stopRecording();   // 先停后开，保证切换时先 close 再开新包
        if (start) startRecording();
    }

    // --- 通用录制（显式传入 topic 名）---
    template<typename T>
    void recordToBag(const std::string& topic, const typename T::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(bag_mtx_);
        if (is_recording_ && bag_.isOpen())
            bag_.write(topic, ros::Time::now(), msg);
    }

    // --- 时长检查 ---
    void checkDuration(const ros::TimerEvent&)
    {
        bool restart = false;
        {
            std::lock_guard<std::mutex> lock(bag_mtx_);
            if (is_recording_) {
                // 用单调时钟测时长，避免系统时间跳变(NTP/开机未同步)导致误判超时、把连续录制劈开
                ros::WallDuration elapsed = ros::SteadyTime::now() - record_start_steady_;
                if (elapsed.toSec() >= max_record_duration_) {
                    stopRecordingLocked();
                    if (auto_restart_on_timeout_) {
                        std::lock_guard<std::mutex> lk(mtx_);
                        restart = last_should_record_;
                    }
                }
            }
        }
        if (restart) startRecording();
    }

    // --- 录制控制 ---
    void startRecording()
    {
        std::lock_guard<std::mutex> lock(bag_mtx_);
        if (is_recording_) return;
        try {
            bag_.open(generateFilename(), rosbag::bagmode::Write);
            is_recording_ = true;
            record_start_ = ros::Time::now();
            record_start_steady_ = ros::SteadyTime::now();
            ROS_INFO("Bag recording started");
        } catch (const std::exception& e) {
            ROS_ERROR("Bag open failed: %s", e.what());
        }
    }

    void stopRecording()
    {
        std::lock_guard<std::mutex> lock(bag_mtx_);
        stopRecordingLocked();
    }

    void stopRecordingLocked()
    {
        if (!is_recording_) return;
        try {
            if (bag_.isOpen()) bag_.close();
            is_recording_ = false;
            ROS_INFO("Bag recording stopped");
        } catch (const std::exception& e) {
            ROS_ERROR("Bag close failed: %s", e.what());
        }
    }

    // --- 文件名管理 ---
    void createBagDirectory()
    {
        try {
            boost::filesystem::path dir(bag_save_dir_);
            if (!boost::filesystem::exists(dir))
                boost::filesystem::create_directories(dir);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to create bag dir: %s", e.what());
        }
    }

    std::string generateFilename()
    {
        std::vector<std::string> files;
        try {
            boost::filesystem::path dir(bag_save_dir_);
            if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
                for (auto& e : boost::filesystem::directory_iterator(dir))
                    if (e.path().extension() == ".bag")
                        files.push_back(e.path().filename().string());
            }
        } catch (...) {}

        if (files.size() >= static_cast<size_t>(max_bag_files_)) {
            std::sort(files.begin(), files.end());
            size_t del = files.size() - max_bag_files_ + 1;
            for (size_t i = 0; i < del; ++i)
                boost::filesystem::remove(bag_save_dir_ + "/" + files[i]);
        }

        int idx = files.size() + 1;
        if (idx > max_bag_files_) idx = 1;

        auto now = std::chrono::system_clock::now();
        auto tt  = std::chrono::system_clock::to_time_t(now);
        std::tm tm;
        localtime_r(&tt, &tm);

        std::ostringstream oss;
        oss << bag_save_dir_ << "/"
            << std::setfill('0') << std::setw(3) << idx << "_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".bag";
        return oss.str();
    }

    // === 成员 ===
    ros::NodeHandle nh_, nh_priv_;
    ros::Subscriber remote_sub_, imu_sub_, alt_sub_, range_sub_, gps_sub_, motor_sub_;
    ros::Subscriber status_sub_, battery_sub_, esc_sub_, attitude_sub_, ch_sub_;
    ros::Subscriber vins_odom_sub_, vins_path_sub_, control_sub_;
    ros::Subscriber sub_color_img_, sub_color_info_, sub_depth_img_, sub_depth_info_;
    ros::Subscriber sub_aligned_img_, sub_aligned_info_, sub_cam_imu_;
    ros::Subscriber sub_infra1_img_, sub_infra2_img_;
    ros::Timer check_timer_;

    rosbag::Bag bag_;
    std::mutex bag_mtx_, mtx_;
    bool is_recording_, first_seen_, last_auto_mode_, last_should_record_;
    bool auto_restart_on_timeout_, record_realsense_, record_camera_infra_;
    ros::Time record_start_;
    ros::SteadyTime record_start_steady_;  // 单调时钟，测时长不受系统时间跳变(NTP/开机未同步)影响
    std::string bag_save_dir_;
    int max_bag_files_, ch5_threshold_, auto_mode_threshold_;
    double max_record_duration_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_recorder_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    RosbagRecorderNode node(nh, nh_priv);

    // 注册信号：SIGINT/SIGTERM 置位标志，主循环退出 → node 析构 → bag_.close() 正常收尾
    std::signal(SIGINT,  handleShutdownSignal);
    std::signal(SIGTERM, handleShutdownSignal);

    while (ros::ok() && !g_shutdown_requested) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    return 0;
}
