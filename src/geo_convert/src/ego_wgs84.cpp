#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// geograhiclib을 이용해서 좌표변환을 수행.

CB_

int main(int argc, char** argv){
    ros::init(argc, argv, "ego_wgs84_node");
    ros::NodeHandle nodehandler("~");

    ros::Publisher pub = nodehandler.advertise<nav_msgs::Path>("/Ego_path", 1, true);
    ros::Subscriber sub_1 = nodehandler.subscribe<>("/Ego_topic",1, CB_ego)
    ros::Subscriber sub_2 = nodehandler.subscribe<>("/Ego_topic",1, CB_ego)

    ROS_INFO_STREAM("[gb_pose] =" << path);
    pub.publish(path);
    ROS_INFO("[global_path_pub_node] published %zu poses from %s",
            path.poses.size(), csv_file.c_str());

    ros::spin(); // latched 유지
    return 0;
}