#include "ros/ros.h"
#include "roscpp_wk1/AddTwoInts.h"

// req부분에 a,b가 들어오고
// res부분에 result부분이 들어온다.
bool add(roscpp_wk1::AddTwoInts::Request &req,
         roscpp_wk1::AddTwoInts::Response &res) {
  res.sum = req.a + req.b;
  ROS_INFO("request: a=%ld b=%ld -> sum=%ld",
           (long)req.a, (long)req.b, (long)res.sum);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "adder_server");
  ros::NodeHandle nh;
  auto srv = nh.advertiseService("add_two_ints", add);
  ROS_INFO("ready: /add_two_ints");
  ros::spin();
  return 0;
}
