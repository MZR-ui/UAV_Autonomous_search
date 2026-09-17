#include <ros/ros.h>
#include <mutex>
#include <vector>
#include <cstdlib>
#include "msp_interface/MspChannel.h"
#include "remote_info/Remote.h"
#include "gimbal_control_serial/GimbalCmd.h"

namespace msp_interface
{

class ModeControllerNode
{
public:
    ModeControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), use_control_data_(false), use_remote_direct_(false)
    {
        nh_priv_.param<double>("publish_rate", publish_rate_, 100.0);
        nh_priv_.param<int>("max_channels", max_channels_, 16);
        nh_priv_.param<double>("remote_timeout", remote_timeout_, 0.1);
        nh_priv_.param<double>("control_timeout", control_timeout_, 0.1);
        nh_priv_.param<double>("gimbal_angle_range", gimbal_angle_range_, 45.0);
        nh_priv_.param<int>("override_threshold", override_threshold_, 10);

        channel_pub_ = nh_.advertise<msp_interface::MspChannel>("msp_channel", 1);
        remote_sub_ = nh_.subscribe("remote_order", 1, &ModeControllerNode::remoteCallback, this);
        control_sub_ = nh_.subscribe("/control_data", 1, &ModeControllerNode::controlCallback, this);
        gimbal_pub_ = nh_.advertise<gimbal_control_serial::GimbalCmd>("/gimbal/cmd", 1);

        double period = 1.0 / publish_rate_;
        timer_ = nh_.createTimer(ros::Duration(period), &ModeControllerNode::timerCallback, this);

        last_remote_time_ = ros::Time(0);
        last_control_time_ = ros::Time(0);

        ROS_INFO("ModeControllerNode: rate=%.1fHz max_channels=%d remote_timeout=%.2fs control_timeout=%.2fs",
                 publish_rate_, max_channels_, remote_timeout_, control_timeout_);
    }

private:
    void remoteCallback(const remote_info::Remote::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_remote_channels_ = msg->channels;
        last_remote_time_ = ros::Time::now();

        if (last_remote_channels_.size() > 7) {
            use_remote_direct_ = (last_remote_channels_[7] < 1350);
            if (use_remote_direct_) {
                use_control_data_ = false;
            }
        } else {
            use_remote_direct_ = false;
        }
    }

    void controlCallback(const msp_interface::MspChannel::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!use_remote_direct_) {
            last_control_channels_ = msg->channels;
            last_control_time_ = ros::Time::now();
            use_control_data_ = true;
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        msp_interface::MspChannel cmd_msg;
        ros::Time now = ros::Time::now();

        {
            std::lock_guard<std::mutex> lock(mutex_);

            bool remote_valid = (last_remote_time_ != ros::Time(0)) &&
                                (now - last_remote_time_ < ros::Duration(remote_timeout_));
            bool control_valid = (last_control_time_ != ros::Time(0)) &&
                                 (now - last_control_time_ < ros::Duration(control_timeout_));

            if (!remote_valid) {
                use_remote_direct_ = false;
            }

            if (remote_valid && use_remote_direct_) {
                // 手动模式：RC 通道直通
                cmd_msg.channels.assign(max_channels_, 1500);
                size_t n = std::min(last_remote_channels_.size(), static_cast<size_t>(max_channels_));
                for (size_t i = 0; i < n; ++i)
                    cmd_msg.channels[i] = last_remote_channels_[i];

                // 手动模式下生成云台指令
                if (last_remote_channels_.size() >= 10) {
                    float roll = (last_remote_channels_[8] - 1500.0f) / 500.0f * gimbal_angle_range_;
                    float yaw  = (last_remote_channels_[9] - 1500.0f) / 500.0f * gimbal_angle_range_;
                    gimbal_control_serial::GimbalCmd gimbal_msg;
                    gimbal_msg.roll  = roll;
                    gimbal_msg.pitch = 0.0f;
                    gimbal_msg.yaw   = yaw;
                    gimbal_msg.mode  = 0;
                    gimbal_pub_.publish(gimbal_msg);
                }
            }
            else if (control_valid && use_control_data_) {
                // 自动模式：转发 BT 输出，姿态/云台通道支持手动接管 override
                cmd_msg.channels.assign(max_channels_, 1500);
                size_t n = std::min(last_control_channels_.size(), static_cast<size_t>(max_channels_));
                for (size_t i = 0; i < n; ++i)
                    cmd_msg.channels[i] = last_control_channels_[i];

                if (remote_valid) {
                    // 姿态通道 CH1/CH2/CH4：偏离中立超阈值 → 交还遥控
                    static const int axes[3] = {0, 1, 3};
                    for (int i : axes) {
                        if (last_remote_channels_.size() > static_cast<size_t>(i) &&
                            std::abs(static_cast<int>(last_remote_channels_[i]) - 1500) > override_threshold_)
                            cmd_msg.channels[i] = last_remote_channels_[i];
                    }
                    // 云台通道 CH9/CH10：偏离超阈值 → 发云台指令（复用同款公式）
                    if (last_remote_channels_.size() >= 10) {
                        bool gimbal_override =
                            (std::abs(static_cast<int>(last_remote_channels_[8]) - 1500) > override_threshold_) ||
                            (std::abs(static_cast<int>(last_remote_channels_[9]) - 1500) > override_threshold_);
                        if (gimbal_override) {
                            float roll = (last_remote_channels_[8] - 1500.0f) / 500.0f * gimbal_angle_range_;
                            float yaw  = (last_remote_channels_[9] - 1500.0f) / 500.0f * gimbal_angle_range_;
                            gimbal_control_serial::GimbalCmd gimbal_msg;
                            gimbal_msg.roll  = roll;
                            gimbal_msg.pitch = 0.0f;
                            gimbal_msg.yaw   = yaw;
                            gimbal_msg.mode  = 0;
                            gimbal_pub_.publish(gimbal_msg);
                        }
                    }
                }
            }
            else if (remote_valid) {
                // 回退：RC 直通
                cmd_msg.channels.assign(max_channels_, 1500);
                size_t n = std::min(last_remote_channels_.size(), static_cast<size_t>(max_channels_));
                for (size_t i = 0; i < n; ++i)
                    cmd_msg.channels[i] = last_remote_channels_[i];
            }
            else {
                // 安全值：全通道 1500，油门 1000
                cmd_msg.channels.assign(max_channels_, 1500);
                cmd_msg.channels[2] = 1000;
            }
        }

        channel_pub_.publish(cmd_msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    double publish_rate_;
    int max_channels_;
    double remote_timeout_;
    double control_timeout_;

    ros::Publisher channel_pub_;
    ros::Subscriber remote_sub_;
    ros::Subscriber control_sub_;
    ros::Timer timer_;

    std::vector<uint16_t> last_remote_channels_;
    std::vector<uint16_t> last_control_channels_;
    bool use_control_data_;
    bool use_remote_direct_;
    ros::Time last_remote_time_;
    ros::Time last_control_time_;
    std::mutex mutex_;

    ros::Publisher gimbal_pub_;
    double gimbal_angle_range_;
    int override_threshold_;
};

} // namespace msp_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mode_manage_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    msp_interface::ModeControllerNode node(nh, nh_priv);
    ros::spin();
    return 0;
}
