// src/local_path_pub_fn.cpp
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <cmath>

// ---- 전역 상태 ----
nav_msgs::Path get_global;     // 최신 전역 경로 저장
bool global_pose_flag = false;  // 전역 경로 유무
double gb_forward_len = 10.0; // 앞으로 자를 길이 [m]
ros::Publisher g_pub_local;  // 로컬 경로 퍼블리셔

// 가장 가까운 인덱스 찾기
int nearestIndex(const nav_msgs::Path& P, double x, double y){
  if(P.poses.empty()) return -1;
  int best = 0; double bd = 1e18;
  for(size_t i=0;i<P.poses.size();++i){
    double dx = P.poses[i].pose.position.x - x;
    double dy = P.poses[i].pose.position.y - y;
    double d2 = dx*dx + dy*dy;
    if(d2 < bd){ bd = d2; best = (int)i; }
  }
  return best;
}

// 전역경로에서 현재 위치 기준으로 앞 gb_forward_len m만 잘라 로컬경로 생성
// 함수이름앞에 nav_msgs::Path는 어떤걸 리턴한다는지 작성할 수 있다.
nav_msgs::Path makeLocalPath(const nav_msgs::Path& gb_pose, int start_idx, double forward_len)
{
  nav_msgs::Path Local_Pth;
  // ROS_INFO_STREAM("[gb_pose] =" << gb_pose);
  printf("[makeLocalPath func] global poses = %zu\n", gb_pose.poses.size());

  //gb_pose에는 waypoint가 리스트형태로 들어가있다.
  Local_Pth.header.frame_id = gb_pose.header.frame_id;
  Local_Pth.header.stamp    = ros::Time::now();

  // 사이즈나 idx에 문제가 있으면 -- 예외처리
  if(gb_pose.poses.size() < 2 || start_idx < 0 || (size_t)start_idx >= gb_pose.poses.size()){
    return Local_Pth; //빈경로를 반환한다.
  }
  
  // accum은 누적되는 거리 
  double accum = 0.0;
  for(size_t i=start_idx; i+1<gb_pose.poses.size(); ++i){
    Local_Pth.poses.push_back(gb_pose.poses[i]);
    const auto &p1 = gb_pose.poses[i].pose.position;
    const auto &p2 = gb_pose.poses[i+1].pose.position;
    accum += std::hypot(p2.x - p1.x, p2.y - p1.y);
    if(accum >= forward_len){
      Local_Pth.poses.push_back(gb_pose.poses[i+1]); // 끝점 하나 더
      break;
    }
  }

  // Local_Pth에 대한 예외 처리 --
  if(Local_Pth.poses.size() < 2) Local_Pth.poses.push_back(gb_pose.poses.back());
  return Local_Pth;
}

// 콜백: 전역 경로 수신
void CB_Global(const nav_msgs::Path::ConstPtr& msg){
  get_global = *msg;
  global_pose_flag = !get_global.poses.empty(); // bool
}

// 콜백: EGO 상태 수신 → 로컬 경로 계산/퍼블리시
void CB_Ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego){
  if(!global_pose_flag || get_global.poses.size() < 2) return;
  const double ego_x = ego->position.x;
  const double ego_y = ego->position.y;
  int idx = nearestIndex(get_global, ego_x, ego_y);
  if(idx < 0) return;
  // makeLocalPath에 gloabl, idx, forward_length를 인자로 넘겨서 local_path를 생성한다. 
  //  만들어진 local_path를 퍼블리시한다. 
  nav_msgs::Path Local_Path = makeLocalPath(get_global, idx, gb_forward_len);
  g_pub_local.publish(Local_Path);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "local_path_pub_node");
  ros::NodeHandle nodehandler("~");

  // 파라미터
  nodehandler.param<double>("forward_len_m", gb_forward_len, 10.0);

  // 퍼블리셔/구독자
  g_pub_local = nodehandler.advertise<nav_msgs::Path>("/local_path", 1, false);
  ros::Subscriber sub_g = nodehandler.subscribe("/global_path", 1, CB_Global);
  ros::Subscriber sub_e = nodehandler.subscribe("/Ego_topic", 10, CB_Ego);

  ROS_INFO_STREAM("[local_path_pub_node] forward_len_m=" << gb_forward_len);
  ros::spin();
  return 0;
}


'''
nav_msgs::Path path;
// path.header ...
path.poses = std::vector<geometry_msgs::PoseStamped>{
  // [0]
  {
    .header = { .frame_id="map", .stamp=... },
    .pose = {
      .position    = { .x=0.0,  .y=0.0,  .z=0.0 },
      .orientation = { .x=0.0,  .y=0.0,  .z=0.0, .w=1.0 }
    }
  },
  // [1]
  {
    .header = { .frame_id="map", .stamp=... },
    .pose = {
      .position    = { .x=10.0, .y=0.0,  .z=0.0 },
      .orientation = { .x=0.0,  .y=0.0,  .z=0.0, .w=1.0 }
    }
  },
'''