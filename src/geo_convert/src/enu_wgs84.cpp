#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// geograhiclib을 이용해서 좌표변환을 수행.

int main(int argc, char** argv){
    ros::init(argc, argv, "ego_wgs84_node");
    ros::NodeHandle nodehandler("~");

    // ego path를 구독받아서 , 
    // 1. enu로 변경하고
    // 2. enu를 wgs84로 변경한다. 
    ros::Publisher pub = nodehandler.advertise<nav_msgs::Path>("/Ego_path", 1, true);
    ros::Subscriber sub_1 = nodehandler.subscribe<>("/Ego_topic",1, CB_ego)
    ros::Subscriber sub_2 = nodehandler.subscribe<>("/Ego_topic",1, CB_ego)

    // Param: origin_lat, origin_lon, origin_alt, map_to_true_north_yaw_deg(맵-진북 오프셋)
    // 1. origin_lat, origin_lon, origin_alt -> 로컬 좌표(시뮬레이터 맵)의 (0,0,0)이 지구상 어디인지
    // 2. map_to_true_north_yaw_deg(맵-진북 오프셋) -> 맵의 xy축이 진동-진북과 정확히 평행한가,에 대한 회전보정값
    nh.param<double>("origin_lat", origin_lat, 2.7);
    nh.param<double>("origin_lon", origin_lon, 4.0);
    nh.param<double>("origin_alt", origin_alt, 2.7);
    nh.param<double>("map_to_true_north_yaw_deg", map_to_true_north_yaw_deg, 4.0);


    
    ROS_INFO_STREAM("[gb_pose] =" << path);
    pub.publish(path);
    ROS_INFO("[global_path_pub_node] published %zu poses from %s",
            path.poses.size(), csv_file.c_str());

    ros::spin(); // latched 유지
    return 0;
}