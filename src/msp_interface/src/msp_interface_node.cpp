#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <cmath>

// 自定义消息
#include "msp_interface/MspSensor.h"
#include "msp_interface/MspChannel.h"
#include "msp_interface/EscTelem.h"
#include "msp_interface/Attitude.h"

// 标准消息
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt32MultiArray.h>
#include <sensor_msgs/BatteryState.h>
#include <tf2/LinearMath/Quaternion.h>

// 串口驱动和协议解析头文件
#include "msp_interface/serial_driver.h"
#include "msp_interface/msp_protocol.h"
#include "msp_interface/msp_parser.h"

namespace msp_interface
{

class MspInterfaceNode
{
public:
    MspInterfaceNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh)
        , nh_priv_(nh_priv)
        , running_(true)
        , counter_(0)
    {
        // 1. 读取 ROS 参数
        nh_priv_.param<std::string>("port", port_, "/dev/ttyACM0");
        nh_priv_.param<int>("baud", baud_, 115200);
        nh_priv_.param<double>("loop_rate", loop_rate_, 1000.0);
        nh_priv_.param<bool>("simulate_channels", simulate_channels_, false); // 是否模拟通道数据
        nh_priv_.param<double>("channel_rate", channel_rate_, 50.0);     // 模拟通道发送频率 (Hz)

        // 读取各指令频率
        nh_priv_.param<double>("rate_102", rate_102_, 50.0);
        nh_priv_.param<double>("rate_109", rate_109_, 10.0);
        nh_priv_.param<double>("rate_106", rate_106_, 5.0);
        nh_priv_.param<double>("rate_104", rate_104_, 20.0);
        nh_priv_.param<double>("rate_0x2000", rate_0x2000_, 5.0);
        nh_priv_.param<double>("rate_0x2002", rate_0x2002_, 2.0);
        nh_priv_.param<double>("rate_0x2220", rate_0x2220_, 50.0);
        nh_priv_.param<double>("rate_0x2041", rate_0x2041_, 20.0);
        nh_priv_.param<double>("rate_0x3A", rate_0x3A_, 10.0);

        auto calc_interval = [&](double rate) -> int {
            if (loop_rate_ > 0 && rate > 0) {
                int interval = static_cast<int>(loop_rate_ / rate);
                return interval < 1 ? 1 : interval;
            }
            return 0;
        };

        interval_102_ = calc_interval(rate_102_);
        interval_109_ = calc_interval(rate_109_);
        interval_106_ = calc_interval(rate_106_);
        interval_104_ = calc_interval(rate_104_);
        interval_0x2000_ = calc_interval(rate_0x2000_);
        interval_0x2002_ = calc_interval(rate_0x2002_);
        interval_0x2220_ = calc_interval(rate_0x2220_);
        interval_0x2041_ = calc_interval(rate_0x2041_);
        interval_0x3A_  = calc_interval(rate_0x3A_);

        if (loop_rate_ > 0 && channel_rate_ > 0 && simulate_channels_) {
            channel_interval_ = static_cast<int>(loop_rate_ / channel_rate_);
            if (channel_interval_ < 1) channel_interval_ = 1;
        } else {
            channel_interval_ = 0;  // 不发送
        }

        ROS_INFO("Loop rate: %.1f Hz", loop_rate_);

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/msp/imu", 10);
        alt_pub_ = nh_.advertise<geometry_msgs::Point>("/msp/altitude", 10);
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/msp/gps", 10);
        motor_pub_ = nh_.advertise<sensor_msgs::JointState>("/msp/motor", 10);
        status_pub_ = nh_.advertise<std_msgs::UInt32MultiArray>("/msp/status", 10);
        battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/msp/battery", 10);
        esc_pub_ = nh_.advertise<msp_interface::EscTelem>("/msp/esc_telem", 10);
        attitude_pub_ = nh_.advertise<msp_interface::Attitude>("/msp/attitude", 10);
        rangefinder_pub_ = nh_.advertise<geometry_msgs::Point>("/msp/rangefinder", 10);

        // 初始化姿态四元数
        q_attitude_.setRPY(0, 0, 0);

        // 3. 创建串口驱动对象并打开串口
        serial_.reset(new SerialDriver());
        if (!serial_->open(port_, baud_))
        {
            ROS_FATAL("Failed to open serial port %s", port_.c_str());
            ros::shutdown();
            return;
        }
        ROS_INFO("Serial port %s opened (baudrate %d)", port_.c_str(), baud_);

        // 4. 启动异步读取线程
        serial_->startAsyncRead(std::bind(&MspInterfaceNode::onSerialData, this,
                                          std::placeholders::_1,
                                          std::placeholders::_2));

        // 5. 创建主循环定时器
        double loop_period = 1.0 / loop_rate_;
        loop_timer_ = nh_.createTimer(ros::Duration(loop_period),
                                      &MspInterfaceNode::loopCallback, this);
        ROS_INFO("Main loop timer started at %.1f Hz", loop_rate_);
        ROS_INFO("MSP interface node started successfully");
    }

    ~MspInterfaceNode()
    {
        running_ = false;
        if (serial_)
            serial_->close();
    }

private:
    tf2::Quaternion q_attitude_;

    void onSerialData(const uint8_t* data, size_t len)
    {
        std::lock_guard<std::mutex> lock(parser_mutex_);
        for (size_t i = 0; i < len; ++i)
        {
            std::vector<uint8_t> packet;
            if (parser_.parseByte(data[i], packet))
            {
                uint16_t cmd = parser_.getLastCommand();
                handleSensorPacket(cmd, packet);
            }
        }
    }

     void handleSensorPacket(uint16_t cmd, const std::vector<uint8_t>& payload) {
        switch(cmd) {
            case 102: handleRawImu(payload); break;
            case 109: handleAltitude(payload); break;
            case 106: handleRawGps(payload); break;
            case 104: handleMotor(payload); break;
            case 0x2000: handleStatus(payload); break;
            case 0x2002: handleAnalog(payload); break;
            case 0x2220: handleLocalPose(payload); break;
            case 0x2041: handleEscTelem(payload); break;
            case 0x3A: handleRangefinder(payload); break;
            case MSP2_RC: break;
            default: ROS_WARN_THROTTLE(1.0, "Unknown command: %d", cmd); break;
        }
    }

    void handleLocalPose(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 24) return;

        int16_t roll_raw  = static_cast<int16_t>(payload[0] | (payload[1] << 8));
        int16_t pitch_raw = static_cast<int16_t>(payload[2] | (payload[3] << 8));
        int16_t yaw_raw   = static_cast<int16_t>(payload[4] | (payload[5] << 8));

        double roll  = (roll_raw  / 10.0) * M_PI / 180.0;
        double pitch = (pitch_raw / 10.0) * M_PI / 180.0;
        double yaw   = (yaw_raw   / 10.0) * M_PI / 180.0;

        q_attitude_.setRPY(roll, pitch, yaw);

        msp_interface::Attitude msg;
        msg.header.stamp = ros::Time::now();
        msg.roll = roll;
        msg.pitch = pitch;
        msg.yaw = yaw;
        msg.orientation.x = q_attitude_.x();
        msg.orientation.y = q_attitude_.y();
        msg.orientation.z = q_attitude_.z();
        msg.orientation.w = q_attitude_.w();

        attitude_pub_.publish(msg);
    }

    void handleRawImu(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 18) return;

        int16_t accX  = static_cast<int16_t>(payload[0] | (payload[1] << 8));
        int16_t accY  = static_cast<int16_t>(payload[2] | (payload[3] << 8));
        int16_t accZ  = static_cast<int16_t>(payload[4] | (payload[5] << 8));
        int16_t gyroX = static_cast<int16_t>(payload[6] | (payload[7] << 8));
        int16_t gyroY = static_cast<int16_t>(payload[8] | (payload[9] << 8));
        int16_t gyroZ = static_cast<int16_t>(payload[10] | (payload[11] << 8));

        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "imu_link";

        double acc_scale = 9.80665 / 512.0;
        msg.linear_acceleration.x = accX * acc_scale;
        msg.linear_acceleration.y = accY * acc_scale;
        msg.linear_acceleration.z = accZ * acc_scale;

        double gyro_scale = M_PI / 180.0;
        msg.angular_velocity.x = gyroX * gyro_scale;
        msg.angular_velocity.y = gyroY * gyro_scale;
        msg.angular_velocity.z = gyroZ * gyro_scale;

        imu_pub_.publish(msg);
    }

    void handleAltitude(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 6) return;

        int32_t estimatedAltitude = static_cast<int32_t>(payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));

        geometry_msgs::Point msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = estimatedAltitude / 100.0; // cm to m

        alt_pub_.publish(msg);
    }

    void handleRangefinder(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 4) return;

        int32_t rangefinderAltitude = static_cast<int32_t>(
            payload[0] | (payload[1] << 8) | (payload[2] << 16) | (payload[3] << 24));

        geometry_msgs::Point msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = rangefinderAltitude / 100.0;  // cm → m
        rangefinder_pub_.publish(msg);
    }

    void handleRawGps(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 16) return;

        uint8_t fixType = payload[0];
        int32_t lat = static_cast<int32_t>(payload[2] | (payload[3] << 8) | (payload[4] << 16) | (payload[5] << 24));
        int32_t lon = static_cast<int32_t>(payload[6] | (payload[7] << 8) | (payload[8] << 16) | (payload[9] << 24));
        int16_t alt = static_cast<int16_t>(payload[10] | (payload[11] << 8));

        sensor_msgs::NavSatFix msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "gps_link";

        msg.status.status = (fixType >= 1) ? sensor_msgs::NavSatStatus::STATUS_FIX : sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        msg.latitude = lat / 1e7;
        msg.longitude = lon / 1e7;
        msg.altitude = alt;

        gps_pub_.publish(msg);
    }

    void handleMotor(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 16) return;

        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();

        for (int i = 0; i < 8; ++i) {
            int16_t pwm = static_cast<int16_t>(payload[i*2] | (payload[i*2+1] << 8));
            msg.name.push_back("motor" + std::to_string(i+1));
            msg.position.push_back(pwm);
        }

        motor_pub_.publish(msg);
    }

    void handleStatus(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 13) return;

        uint16_t cycleTime = static_cast<uint16_t>(payload[0] | (payload[1] << 8));
        uint16_t i2cErrors = static_cast<uint16_t>(payload[2] | (payload[3] << 8));
        uint16_t sensorStatus = static_cast<uint16_t>(payload[4] | (payload[5] << 8));
        uint16_t cpuLoad = static_cast<uint16_t>(payload[6] | (payload[7] << 8));
        uint8_t profileAndBatt = payload[8];
        uint32_t armingFlags = static_cast<uint32_t>(payload[9] | (payload[10] << 8) | (payload[11] << 16) | (payload[12] << 24));

        std_msgs::UInt32MultiArray msg;
        msg.data.push_back(cycleTime);
        msg.data.push_back(i2cErrors);
        msg.data.push_back(sensorStatus);
        msg.data.push_back(cpuLoad);
        msg.data.push_back(profileAndBatt);
        msg.data.push_back(armingFlags);

        status_pub_.publish(msg);
    }

    void handleAnalog(const std::vector<uint8_t>& payload)
    {
        if (payload.size() < 5) return;

        uint8_t batteryFlags = payload[0];
        uint16_t vbat = static_cast<uint16_t>(payload[1] | (payload[2] << 8));
        int16_t amperage = static_cast<int16_t>(payload[3] | (payload[4] << 8));

        sensor_msgs::BatteryState msg;
        msg.header.stamp = ros::Time::now();
        msg.voltage = vbat * 0.01f;
        msg.current = amperage * 0.01f;
        msg.power_supply_status = batteryFlags;

        battery_pub_.publish(msg);
    }

    void handleEscTelem(const std::vector<uint8_t>& payload)
    {
        if (payload.empty()) return;
        uint8_t motorCount = payload[0];

        // INAV 9.x uses 16 bytes per ESC (13 bytes data + 3 bytes padding for alignment)
        const size_t ESC_DATA_SIZE = 16;
        if (payload.size() < 1 + motorCount * ESC_DATA_SIZE) return;

        msp_interface::EscTelem msg;
        msg.header.stamp = ros::Time::now();
        msg.motor_count = motorCount;

        for (int i = 0; i < motorCount; ++i) {
            size_t offset = 1 + i * ESC_DATA_SIZE;

            // Structure: dataAge(1) + padding(1) + temp(2) + volt(2) + padding(2) + current(4) + rpm(4)
            msg.data_age.push_back(payload[offset]);
            msg.temperature.push_back(static_cast<int16_t>(payload[offset+2] | (payload[offset+3] << 8)));
            msg.voltage.push_back(static_cast<int16_t>(payload[offset+4] | (payload[offset+5] << 8)));

            int32_t current = static_cast<int32_t>(payload[offset+8] | (payload[offset+9] << 8) | (payload[offset+10] << 16) | (payload[offset+11] << 24));
            uint32_t rpm = static_cast<uint32_t>(payload[offset+12] | (payload[offset+13] << 8) | (payload[offset+14] << 16) | (payload[offset+15] << 24));

            msg.current.push_back(current);
            msg.rpm.push_back(rpm);
        }
        esc_pub_.publish(msg);
    }

    void simple_cal_send_ID(void)
    {
        for (int i = 0; i < 9; ++i)
        {
            uint8_t id = (now_send_ID + i) % 9;
            if (sign_send_enable & (1u << id))
            {
                now_send_ID = id;
                break;
            }
        }
    }

    void loopCallback(const ros::TimerEvent&)
    {
        counter_++;

        if (interval_102_ > 0 && (counter_ % interval_102_ == 0)) sign_send_enable|=0x00000001;
        if (interval_109_ > 0 && (counter_ % interval_109_ == 0)) sign_send_enable|=0x00000002;
        if (interval_106_ > 0 && (counter_ % interval_106_ == 0)) sign_send_enable|=0x00000004;
        if (interval_104_ > 0 && (counter_ % interval_104_ == 0)) sign_send_enable|=0x00000008;
        if (interval_0x2000_ > 0 && (counter_ % interval_0x2000_ == 0)) sign_send_enable|=0x00000010;
        if (interval_0x2002_ > 0 && (counter_ % interval_0x2002_ == 0)) sign_send_enable|=0x00000020;
        if (interval_0x2220_ > 0 && (counter_ % interval_0x2220_ == 0)) sign_send_enable|=0x00000040;
        if (interval_0x2041_ > 0 && (counter_ % interval_0x2041_ == 0)) sign_send_enable|=0x00000080;
        if (interval_0x3A_  > 0 && (counter_ % interval_0x3A_  == 0)) sign_send_enable|=0x00000100;


        if(counter_%9==0)//msp带宽为100HZ
        {
            if((sign_send_enable & 0x00000001)&&(now_send_ID==0))
            {
                sendRequest(102);sign_send_enable&=0xfffffffe;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000002)&&(now_send_ID==1))
            {
                sendRequest(109);sign_send_enable&=0xfffffffd;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000004)&&(now_send_ID==2))
            {
                sendRequest(106);sign_send_enable&=0xfffffffb;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000008)&&(now_send_ID==3))
            {
                sendRequest(104);sign_send_enable&=0xfffffff7;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000010)&&(now_send_ID==4))
            {
                sendRequest(0x2000);sign_send_enable&=0xffffffef;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000020)&&(now_send_ID==5))
            {
                sendRequest(0x2002);sign_send_enable&=0xffffffdf;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000040)&&(now_send_ID==6))
            {
                sendRequest(0x2220);sign_send_enable&=0xffffffbf;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000080)&&(now_send_ID==7))
            {
                sendRequest(0x2041);sign_send_enable&=0xffffff7f;simple_cal_send_ID();
            }
            else if((sign_send_enable & 0x00000100)&&(now_send_ID==8))
            {
                sendRequest(0x3A);sign_send_enable&=0xfffffeff;simple_cal_send_ID();
            }
        }
    }

    void sendRequest(uint16_t cmd)
    {
        std::vector<uint8_t> request = packMspV2Request(cmd, nullptr, 0);
        if (!request.empty())
        {
            std::lock_guard<std::mutex> lock(serial_write_mutex_);
            if (!serial_->write(request.data(), request.size()))
            {
                ROS_ERROR_THROTTLE(1.0, "Failed to send request (cmd=%u)", cmd);
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    std::string port_;
    int baud_;
    double loop_rate_;
    bool simulate_channels_;
    double channel_rate_;

    double rate_102_;
    double rate_109_;
    double rate_106_;
    double rate_104_;
    double rate_0x2000_;
    double rate_0x2002_;
    double rate_0x2220_;
    double rate_0x2041_;
    double rate_0x3A_;

    int interval_102_;
    int interval_109_;
    int interval_106_;
    int interval_104_;
    int interval_0x2000_;
    int interval_0x2002_;
    int interval_0x2220_;
    int interval_0x2041_;
    int interval_0x3A_;

    int channel_interval_;
    uint64_t counter_;
    uint64_t sign_send_enable = 0;
    uint8_t now_send_ID = 0;

    std::unique_ptr<SerialDriver> serial_;
    std::atomic<bool> running_;

    MspV2Parser parser_;
    std::mutex parser_mutex_;
    std::mutex serial_write_mutex_;
    std::mutex pose_mutex_;

    ros::Publisher sensor_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher alt_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher motor_pub_;
    ros::Publisher status_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher esc_pub_;
    ros::Publisher attitude_pub_;
    ros::Publisher rangefinder_pub_;
    ros::Timer request_timer_;
    ros::Timer channel_sim_timer_;

    ros::Timer loop_timer_;

};

} // namespace msp_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "msp_interface_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    msp_interface::MspInterfaceNode node(nh, nh_priv);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}