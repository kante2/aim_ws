#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <cmath>

struct Pt{ double x,y; };

class PurePursuit {
  ros::Subscriber sub_path_, sub_ego_;
  ros::Publisher pub_cmd_;
  nav_msgs::Path lpath_;
  morai_msgs::EgoVehicleStatus ego_;
  bool have_path_=false, have_ego_=false;
  double L_;      // wheelbase
  double Ld_min_, Ld_max_; // lookahead bounds
  double v_set_;  // target speed

public:
  PurePursuit(ros::NodeHandle& nh){
    nh.param<double>("wheelbase", L_, 2.7);
    nh.param<double>("Ld_min", Ld_min_, 6.0);
    nh.param<double>("Ld_max", Ld_max_, 16.0);
    nh.param<double>("v_set", v_set_, 8.0); // m/s
    sub_path_ = nh.subscribe("/local_path",1,&PurePursuit::cbPath,this);
    sub_ego_  = nh.subscribe("/Ego_topic",10,&PurePursuit::cbEgo,this);
    pub_cmd_  = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd",1);
  }
  void cbPath(const nav_msgs::Path::ConstPtr& p){ lpath_=*p; have_path_=true; }
  void cbEgo(const morai_msgs::EgoVehicleStatus::ConstPtr& m){ ego_=*m; have_ego_=true; control(); }

  static double clamp(double a,double lo,double hi){ return std::max(lo,std::min(hi,a)); }

  // 경로를 따라 Ld 위치 찾기(선형보간)
  bool findLookahead(const nav_msgs::Path& P, const Pt& ego_map, double Ld, Pt& tgt_map){
    if(P.poses.size()<2) return false;
    // 가장 가까운 인덱스
    int best=0; double bd=1e18;
    for(size_t i=0;i<P.poses.size();++i){
      double dx=P.poses[i].pose.position.x-ego_map.x;
      double dy=P.poses[i].pose.position.y-ego_map.y;
      double d=dx*dx+dy*dy; if(d<bd){bd=d; best=i;}
    }
    double s_prev=0.0;
    for(size_t i=best;i+1<P.poses.size();++i){
      double x1=P.poses[i].pose.position.x, y1=P.poses[i].pose.position.y;
      double x2=P.poses[i+1].pose.position.x, y2=P.poses[i+1].pose.position.y;
      double seg=std::hypot(x2-x1,y2-y1);
      if(s_prev+seg >= Ld){
        double t = (Ld - s_prev)/seg;
        tgt_map.x = x1 + t*(x2-x1);
        tgt_map.y = y1 + t*(y2-y1);
        return true;
      }
      s_prev += seg;
    }
    // 경로가 Ld보다 짧으면 끝점
    tgt_map.x = P.poses.back().pose.position.x;
    tgt_map.y = P.poses.back().pose.position.y;
    return true;
  }

  void control(){
    if(!have_path_||!have_ego_) return;
    // 속도 기반 Ld 스케일(간단): v in [0, v_set] → Ld in [Ld_min, Ld_max]
    double v = ego_.velocity; // m/s
    double Ld = Ld_min_ + (Ld_max_-Ld_min_)*clamp(v/v_set_,0.0,1.0);

    Pt ego_map{ego_.position.x, ego_.position.y};
    Pt tgt_map;
    if(!findLookahead(lpath_, ego_map, Ld, tgt_map)) return;

    // map → ego frame 변환 (yaw=heading)
    double yaw = ego_.heading;
    double dx = tgt_map.x - ego_map.x;
    double dy = tgt_map.y - ego_map.y;
    double x_e =  cos(yaw)*dx + sin(yaw)*dy;
    double y_e = -sin(yaw)*dx + cos(yaw)*dy;

    // Pure Pursuit
    double alpha = std::atan2(y_e, x_e);
    double delta = std::atan( 2.0 * L_ * std::sin(alpha) / Ld );

    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;   // 1: velocity cmd (MORAI 설정에 맞게 조정)
    cmd.velocity = v_set_;  // 간단 정속. (곡률기반 속도계획 넣어도 됨)
    cmd.steering = delta;   // MORAI 규약: 라디안형/노말라이즈 여부 확인 필요
    cmd.accel = 0.0; cmd.brake = 0.0;
    pub_cmd_.publish(cmd);
  }
};

int main(int argc,char**argv){
  ros::init(argc,argv,"pure_pursuit_controller");
  ros::NodeHandle nh("~");
  PurePursuit node(nh);
  ros::spin();
  return 0;
}
