#include "ros/ros.h"
#include "roscpp_wk1/Greeting.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "greet_pub");
  ros::NodeHandle nh;

  auto pub = nh.advertise<roscpp_wk1::Greeting>("greeting", 10);
  ros::Rate rate(5);
  uint32_t cnt = 0;

  while (ros::ok()) {
    roscpp_wk1::Greeting msg;
    msg.name  = "kante";
    msg.count = cnt++;

    ROS_INFO("publish: name=%s count=%u", msg.name.c_str(), msg.count);
    pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
