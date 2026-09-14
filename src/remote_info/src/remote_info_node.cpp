#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

// 自定义消息
#include "remote_info/Remote.h"
#include "msp_interface/MspChannel.h"

// 串口驱动和协议解析头文件
#include "remote_info/serial_driver.h"
#include "remote_info/ibus_parser.h"

namespace remote_info
{

class RemoteInfoNode
{
public:
    RemoteInfoNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh)
        , nh_priv_(nh_priv)
        , running_(true)
    {
        // 1. 读取 ROS 参数
        nh_priv_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_priv_.param<int>("baud", baud_, 115200);  // IBUS 固定 115200
        nh_priv_.param<double>("send_rate", send_rate_, 0);  // 发送频率，0表示禁用


        // 2. 初始化发布者
        remote_pub_ = nh_.advertise<remote_info::Remote>("remote_order", 1);

        channel_sub_ = nh_.subscribe("/msp_channel", 10, &RemoteInfoNode::channelCallback, this);

        // 3. 创建串口驱动对象并打开串口
        serial_.reset(new SerialDriver());
        if (!serial_->open(port_, baud_))
        {
            ROS_FATAL("Failed to open serial port %s", port_.c_str());
            ros::shutdown();
            return;
        }
        ROS_INFO("Remote: Serial port %s opened (baudrate %d)", port_.c_str(), baud_);

        // 4. 启动异步读取线程
        serial_->startAsyncRead(std::bind(&RemoteInfoNode::onSerialData, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));
                     
        if (send_rate_ > 0)
        {
            double send_period = 1.0 / send_rate_;
            send_timer_ = nh_.createTimer(ros::Duration(send_period),
                                        &RemoteInfoNode::sendTimerCallback, this);
            ROS_INFO("Remote: Send timer enabled at %.1f Hz", send_rate_);
        }

        ROS_INFO("Remote info node started successfully");
    }

    ~RemoteInfoNode()
    {
        running_ = false;
        if (serial_)
            serial_->close();
    }

private:

    /**
     * @brief 订阅话题 /msp_channel 的回调，将控制数据发送给飞控
     */
    void channelCallback(const msp_interface::MspChannel::ConstPtr& msg)
    {
        sendChannels(msg->channels);
    }

    /**
     * @brief 将通道数据打包并通过串口发送给飞控 (使用IBUS协议)
     * @param channels 通道值数组
     */
    void sendChannels(const std::vector<uint16_t>& channels)
    {
        // 1. Initialize a 32-byte frame with zeros
        std::vector<uint8_t> ibus_frame(32, 0);

        // 2. Set the Length and Command headers
        ibus_frame[0] = 0x20;
        ibus_frame[1] = 0x40;

        // 3. Pack up to 14 channels (Little-Endian)
        for (size_t i = 0; i < 14; ++i)
        {
            // If the incoming /msp_channel has fewer than 14 channels, default to 1500
            uint16_t ch = (i < channels.size()) ? channels[i] : 1500;

            ibus_frame[2 + i * 2] = ch & 0xFF;           // Low byte
            ibus_frame[3 + i * 2] = (ch >> 8) & 0xFF;    // High byte
        }

        // 4. Calculate the Checksum
        uint16_t sum = 0;
        for (int i = 0; i < 30; ++i)
        {
            sum += ibus_frame[i];
        }
        uint16_t checksum = 0xFFFF - sum;

        // 5. Append the Checksum (Little-Endian)
        ibus_frame[30] = checksum & 0xFF;
        ibus_frame[31] = (checksum >> 8) & 0xFF;

        // 6. Send over Serial
        std::lock_guard<std::mutex> lock(serial_write_mutex_);
        if (!serial_->write(ibus_frame.data(), ibus_frame.size()))
        {
            ROS_ERROR("Failed to send IBUS channel data");
        }
    }

    void onSerialData(const uint8_t* data, size_t len)
    {
        // 可选：打印原始数据用于调试
        // ROS_INFO( "Remote RX: %zu bytes", len);

        std::lock_guard<std::mutex> lock(parser_mutex_);
        for (size_t i = 0; i < len; ++i)
        {
            std::vector<uint16_t> channels;
            if (parser_.parseByte(data[i], channels))
            {
                // 解析出一帧完整数据
                remote_info::Remote msg;
                msg.header.stamp = ros::Time::now();
                msg.channels = channels;
                remote_pub_.publish(msg);
                // printf("success!\r\n");
                // ROS_INFO("Published remote: CH1=%d, CH2=%d, CH3=%d", 
                                //    channels[0], channels[1], channels[2]);
            }
        }
    }

    // 添加回调函数：
    void sendTimerCallback(const ros::TimerEvent&)
    {
        // Construct a test frame containing 14 channels (default to 1500)
        std::vector<uint16_t> test_channels(14, 1500);
        sendChannels(test_channels);
    }



private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::string port_;
    int baud_;

    std::unique_ptr<SerialDriver> serial_;
    std::atomic<bool> running_;

    IbusParser parser_;
    std::mutex parser_mutex_;
    std::mutex serial_write_mutex_;

    ros::Publisher remote_pub_;
    ros::Subscriber channel_sub_;
    
    ros::Timer send_timer_;
    double send_rate_;
};

} // namespace remote_info

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remote_info_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    remote_info::RemoteInfoNode node(nh, nh_priv);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}