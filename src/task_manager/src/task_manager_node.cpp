#include <ros/ros.h>
#include <mutex>
#include <remote_info/Remote.h>
#include <task_manager/OrderTask.h>

namespace task_manager
{

class TaskManagerNode
{
public:
    TaskManagerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
        : nh_(nh), nh_priv_(nh_priv), current_channel_value_(1500)
    {
        // 参数：通道索引、阈值、发布频率
        nh_priv_.param<int>("channel_index", channel_index_, 4);    // 默认通道5（索引4）
        nh_priv_.param<int>("threshold_low", threshold_low_, 1300);
        nh_priv_.param<int>("threshold_high", threshold_high_, 1700);
        nh_priv_.param<double>("publish_rate", publish_rate_, 10.0);

        // 订阅遥控器数据（话题 /remote_order）
        remote_sub_ = nh_.subscribe("/remote_order", 10, &TaskManagerNode::remoteCallback, this);

        // 发布任务命令
        task_pub_ = nh_.advertise<task_manager::OrderTask>("/order_task", 10);

        // 定时器定期发布
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                 &TaskManagerNode::timerCallback, this);

        ROS_INFO("TaskManagerNode started, publishing at %.1f Hz", publish_rate_);
    }

private:
    void remoteCallback(const remote_info::Remote::ConstPtr& msg)
    {
        if (msg->channels.size() > channel_index_)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_channel_value_ = msg->channels[channel_index_];
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        uint8_t task_id = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (current_channel_value_ < threshold_low_)
                task_id = 0;
            else if (current_channel_value_ > threshold_high_)
                task_id = 2;
            else
                task_id = 1;
        }

        task_manager::OrderTask msg;
        msg.task_id = task_id;
        task_pub_.publish(msg);
        ROS_DEBUG_THROTTLE(1.0, "Published task_id: %d", task_id);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    int channel_index_;
    int threshold_low_;
    int threshold_high_;
    double publish_rate_;

    ros::Subscriber remote_sub_;
    ros::Publisher task_pub_;
    ros::Timer timer_;

    std::mutex mutex_;
    uint16_t current_channel_value_;
};

} // namespace task_manager

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    task_manager::TaskManagerNode node(nh, nh_priv);
    ros::spin();
    return 0;
}