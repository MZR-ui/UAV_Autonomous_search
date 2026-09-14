#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt32MultiArray.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "msp_interface/MspChannel.h"
#include "msp_interface/EscTelem.h"
#include "msp_interface/Attitude.h"
#include "remote_info/Remote.h"
#include "subtask/ControlData.h"
#include "gimbal_control_serial/GimbalCmd.h"

namespace msp_interface
{

class ModeControllerNode
{
public:
    ModeControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), use_control_data_(false), use_remote_direct_(false),
          is_recording_(false), last_ch8_state_(false)
    {
        // 读取���数
        nh_priv_.param<double>("publish_rate", publish_rate_, 100.0);
        nh_priv_.param<int>("max_channels", max_channels_, 16);
        nh_priv_.param<double>("remote_timeout", remote_timeout_, 0.1);
        nh_priv_.param<double>("control_timeout", control_timeout_, 0.1);
        nh_priv_.param<double>("gimbal_angle_range", gimbal_angle_range_, 45.0);

        // rosbag录���参数
        nh_priv_.param<std::string>("bag_save_dir", bag_save_dir_, "/data/rosbag");
        nh_priv_.param<int>("max_bag_files", max_bag_files_, 100);
        nh_priv_.param<double>("max_record_duration", max_record_duration_, 600.0);

        // 发布���和订阅者
        channel_pub_ = nh_.advertise<msp_interface::MspChannel>("msp_channel", 1);
        remote_sub_ = nh_.subscribe("remote_order", 1, &ModeControllerNode::remoteCallback, this);
        control_sub_ = nh_.subscribe("/control_data", 1, &ModeControllerNode::controlCallback, this);
        gimbal_pub_ = nh_.advertise<gimbal_control_serial::GimbalCmd>("/gimbal/cmd", 1);

        // 订阅���有需要录制的/msp/话题
