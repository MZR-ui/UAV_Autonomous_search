#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 10);

  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok()) {
    std_msgs::String msg;
    msg.data = "hello world " + std::to_string(count++);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
