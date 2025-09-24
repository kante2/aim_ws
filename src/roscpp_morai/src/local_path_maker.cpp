#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <cmath>

class LocalPathMaker{
  ros::Subscriber sub_path_, sub_ego_;
  ros::Publisher pub_;
  nav_msgs::Path gpath_;
  bool have_path_=false;
  double forward_len_m_;

public:
  LocalPathMaker(ros::NodeHandle& nh){
    nh.param<double>("forward_len_m", forward_len_m_, 30.0); // 15~30m 추천
    sub_path_ = nh.subscribe("/global_path",1,&LocalPathMaker::cbPath,this);
    sub_ego_  = nh.subscribe("/Ego_topic",10,&LocalPathMaker::cbEgo,this);
    pub_ = nh.advertise<nav_msgs::Path>("/local_path",1);
  }
  void cbPath(const nav_msgs::Path::ConstPtr& p){ gpath_=*p; have_path_=true; }

  static int nearestIndex(const nav_msgs::Path& P, double x, double y){
    int best=-1; double bd=1e18;
    for(size_t i=0;i<P.poses.size();++i){
      double dx=P.poses[i].pose.position.x - x;
      double dy=P.poses[i].pose.position.y - y;
      double d=dx*dx+dy*dy;
      if(d<bd){ bd=d; best=i;}
    }
    return best;
  }

  void cbEgo(const morai_msgs::EgoVehicleStatus::ConstPtr& m){
    if(!have_path_ || gpath_.poses.empty()) return;
    int idx = nearestIndex(gpath_, m->position.x, m->position.y);
    if(idx<0) return;

    nav_msgs::Path L; L.header.frame_id="map"; L.header.stamp=ros::Time::now();
    double accum=0.0;
    for(size_t i=idx;i+1<gpath_.poses.size();++i){
      L.poses.push_back(gpath_.poses[i]);
      double x1=gpath_.poses[i].pose.position.x, y1=gpath_.poses[i].pose.position.y;
      double x2=gpath_.poses[i+1].pose.position.x, y2=gpath_.poses[i+1].pose.position.y;
      accum += std::hypot(x2-x1,y2-y1);
      if(accum>=forward_len_m_){ L.poses.push_back(gpath_.poses[i+1]); break; }
    }
    if(L.poses.size()<2) L.poses.push_back(gpath_.poses.back());
    pub_.publish(L);
  }
};

int main(int argc,char**argv){
  ros::init(argc,argv,"local_path_maker");
  ros::NodeHandle nh("~");
  LocalPathMaker node(nh);
  ros::spin();
  return 0;
}
