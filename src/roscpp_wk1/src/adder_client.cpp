#include "ros/ros.h"
#include "roscpp_wk1/AddTwoInts.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "adder_client");
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<roscpp_wk1::AddTwoInts>("add_two_ints");

  roscpp_wk1::AddTwoInts srv;
  srv.request.a = 7;
  srv.request.b = 5;

  if (client.call(srv)) {
    ROS_INFO("sum=%ld", (long)srv.response.sum);
  } else {
    ROS_ERROR("service call failed");
    return 1;
  }
  return 0;
}
