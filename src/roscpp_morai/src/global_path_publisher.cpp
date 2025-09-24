#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>

class GlobalPathPublisher{
  ros::Publisher pub_;
  nav_msgs::Path path_;
  double pub_rate_;

public:
  GlobalPathPublisher(ros::NodeHandle& nh){
    std::string file; nh.param<std::string>("file", file, "/tmp/ego_path.csv");
    nh.param<double>("pub_rate", pub_rate_, 5.0);
    pub_ = nh.advertise<nav_msgs::Path>("/global_path",1,true);
    path_.header.frame_id="map";

    std::ifstream ifs(file);
    if(!ifs){ ROS_ERROR("Cannot open %s", file.c_str()); return; }
    std::string line; std::getline(ifs,line); // header
    while(std::getline(ifs,line)){
      std::stringstream ss(line);
      std::string sx,sy,syaw;
      if(!std::getline(ss,sx,',')) break;
      if(!std::getline(ss,sy,',')) break;
      if(!std::getline(ss,syaw,',')) syaw="0";
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id="map";
      ps.pose.position.x = std::stod(sx);
      ps.pose.position.y = std::stod(sy);
      ps.pose.orientation.w = 1.0; // 시각화용(필요시 yaw->quat)
      path_.poses.push_back(ps);
    }
    pub_.publish(path_);
    ROS_INFO("[global_path] loaded %zu poses", path_.poses.size());
  }
};

int main(int argc,char**argv){
  ros::init(argc,argv,"global_path_publisher");
  ros::NodeHandle nh("~");
  GlobalPathPublisher node(nh);
  ros::spin();
  return 0;
}
