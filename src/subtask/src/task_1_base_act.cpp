#include <ros/ros.h>
#include <mutex>
#include <task_manager/OrderTask.h>
#include <subtask/ControlData.h>

namespace subtask
{

class Task1BaseAct
{
public:
    Task1BaseAct(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), current_task_id_(0)
    {
        // 读取参数：发布频率，默认50Hz
        nh_priv_.param<double>("publish_rate", publish_rate_, 100.0);
        // 读取任务对应的指令参数（可选，也可硬编码）
        nh_priv_.param<float>("throttle_idle", throttle_idle_, 1000.0f);
        nh_priv_.param<float>("throttle_auto", throttle_auto_, 1500.0f);
        // ... 其他参数可类似添加

        // 订阅任务ID
        task_sub_ = nh_.subscribe("/order_task", 10, &Task1BaseAct::taskCallback, this);

        // 发布控制数据
        control_pub_ = nh_.advertise<subtask::ControlData>("/control_data", 10);

        // 定时器定期发布
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &Task1BaseAct::timerCallback, this);

        ROS_INFO("Task1BaseAct started, publishing at %.1f Hz", publish_rate_);
    }

private:
    void taskCallback(const task_manager::OrderTask::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_task_id_ = msg->task_id;
        ROS_DEBUG_THROTTLE(1.0, "Received task_id: %d", current_task_id_);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        uint8_t task_id_copy;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            task_id_copy = current_task_id_;
        }

        subtask::ControlData cmd;
        // 根据任务ID设置控制指令（示例值，请根据实际需要调整）
        switch (task_id_copy)
        {
        case 0:  // 空闲/安全模式
            cmd.throttle = throttle_idle_;
            cmd.pitch = 1000.0f;
            cmd.roll = 1000.0f;
            cmd.yaw = 1000.0f;
            cmd.mode_gim = 0;       // 跟随
            cmd.pitch_gim = 0.0f;
            cmd.roll_gim = 0.0f;
            cmd.yaw_gim = 0.0f;
            break;
        case 1:  // 任务1: 起飞/悬停
            cmd.throttle = throttle_auto_;
            cmd.pitch = 1000.0f;
            cmd.roll = 1000.0f;
            cmd.yaw = 1000.0f;
            cmd.mode_gim = 1;       // 锁定
            cmd.pitch_gim = -30.0f;  // 云台向下30度
            cmd.roll_gim = 0.0f;
            cmd.yaw_gim = 0.0f;
            break;
        case 2:  // 任务2: 前进/扫描
            cmd.throttle = throttle_auto_;
            cmd.pitch = 1000.0f;        // 俯仰5度前进
            cmd.roll = 1000.0f;
            cmd.yaw = 1000.0f;
            cmd.mode_gim = 0;        // 跟随
            cmd.pitch_gim = -45.0f;
            cmd.roll_gim = 0.0f;
            cmd.yaw_gim = 0.0f;
            break;
        default:
            cmd.throttle = throttle_idle_;
            cmd.pitch = 1000.0f;
            cmd.roll = 1000.0f;
            cmd.yaw = 1000.0f;
            cmd.mode_gim = 0;
            cmd.pitch_gim = 0.0f;
            cmd.roll_gim = 0.0f;
            cmd.yaw_gim = 0.0f;
            break;
        }

        control_pub_.publish(cmd);
        ROS_DEBUG_THROTTLE(1.0, "Published control data for task %d", task_id_copy);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    double publish_rate_;
    uint8_t current_task_id_;

    // 可配置参数（示例）
    float throttle_idle_;
    float throttle_auto_;

    ros::Subscriber task_sub_;
    ros::Publisher control_pub_;
    ros::Timer timer_;
    std::mutex mutex_;
};

} // namespace subtask

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_1_base_act");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    subtask::Task1BaseAct node(nh, nh_priv);
    ros::spin();
    return 0;
}