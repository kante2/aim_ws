#include "ros/ros.h"
#include "roscpp_wk1/Greeting.h"

void cb(const roscpp_wk1::Greeting::ConstPtr& msg) {
  ROS_INFO("I heard: name=%s count=%u", msg->name.c_str(), msg->count);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "greet_sub");
  ros::NodeHandle nh;
  auto sub = nh.subscribe("greeting", 10, cb);
  ros::spin();
  return 0;
}
