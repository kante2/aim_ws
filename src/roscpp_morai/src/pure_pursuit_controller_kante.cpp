#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <algorithm>   // std::min/max

// ----------------- 전역 상태 -----------------
nav_msgs::Path g_local;
morai_msgs::EgoVehicleStatus g_ego;
bool g_have_local = false;
bool g_have_ego   = false;
ros::Publisher g_pub_cmd;

// ----------------- 파라미터 ------------------
double g_wheelbase = 2.7;   // [m]
double g_v_set     = 4.0;   // [m/s]; 간단 정속

// ----------------- 유틸 ----------------------
// static inline double clamp(double x, double lo, double hi){
//   if (x < lo) return lo;
//   if (x > hi) return hi;
//   return x;
// }

static int nearestIndex(const nav_msgs::Path& Pose, double x, double y){
  if (Pose.poses.empty()) return -1;
  int best = 0; double bd = 1e18;
  for (size_t i=0; i<Pose.poses.size(); ++i){
    double dx = Pose.poses[i].pose.position.x - x;
    double dy = Pose.poses[i].pose.position.y - y;
    double d2 = dx*dx + dy*dy;
    if (d2 < bd){ bd = d2; best = (int)i; }
  }
  return best;
}

// ----------------- 핵심 로직 ------------------
static void runPP(){
  if (!(g_have_local && g_have_ego)) return;
  if (g_local.poses.size() < 2) return;

  // 1) EGO와 가장 가까운 점 idx, 타깃은 idx+1
  const double ex = g_ego.position.x;
  const double ey = g_ego.position.y;
  int idx = nearestIndex(g_local, ex, ey);
  if (idx < 0) return;

  // <int>로 템플릿인자 지정 -> 두인자를 int로 모두 비교하라고 강제할수있음,
  // idx+1,즉 다음점을 타겟으로 한다. 
  int tgt_idx = std::min<int>(idx + 1, (int)g_local.poses.size() - 1);

  // 2) map → ego 좌표 변환
  const auto& tgt = g_local.poses[tgt_idx].pose.position;
  double yaw = g_ego.heading; // 라디안 가정(MORAI 프로젝트 세팅 확인)
  double dx = tgt.x - ex;
  double dy = tgt.y - ey;
  double x_e =  std::cos(yaw)*dx + std::sin(yaw)*dy;
  double y_e = -std::sin(yaw)*dx + std::cos(yaw)*dy;

  // 3) Pure Pursuit (간단: 한 점 타깃)
  double alpha = std::atan2(y_e, x_e);
  double Ld    = std::max(1.0, std::hypot(x_e, y_e)); // 너무 작을 때 폭주 방지
  double delta = std::atan( 2.0 * g_wheelbase * std::sin(alpha) / Ld ); // [rad]

  // 4) 명령 퍼블리시
  morai_msgs::CtrlCmd cmd;
  cmd.longlCmdType = 2;      // 1: velocity control (프로젝트 규격 확인)
  cmd.velocity     = g_v_set;
  cmd.steering     = delta;  // MORAI가 라디안/정규화 중 무엇인지 확인 필요
  cmd.accel = 0.0; 
  cmd.brake = 0.0;

  g_pub_cmd.publish(cmd);
}

// ----------------- 콜백 ----------------------
void CB_local(const nav_msgs::Path::ConstPtr& msg){
  g_local = *msg;
  g_have_local = !g_local.poses.empty();

}

void CB_ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego){
  g_ego = *ego;
  g_have_ego = true;

  // 시작점(로컬 첫 점)과의 거리 모니터링 위해서 생성, 
  if (g_have_local){
    const auto& p_local = g_local.poses.front().pose.position;
    double dx = p_local.x - g_ego.position.x;
    double dy = p_local.y - g_ego.position.y;
    double d  = std::hypot(dx, dy);
    ROS_INFO(0.2, "[start-err] d=%.3f m (dx=%.3f, dy=%.3f)", d, dx, dy);
  }

  runPP(); // pp은 ego주기로만 한다. 
}

// ----------------- main ----------------------
int main(int argc, char** argv){
  ros::init(argc, argv, "pure_pursuit_from_local_fn");
  ros::NodeHandle nh("~");

  // 파라미터
  nh.param<double>("wheelbase", g_wheelbase, 2.7);
  nh.param<double>("v_set",     g_v_set,     4.0);

  // 토픽
  g_pub_cmd = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
  ros::Subscriber sub_local = nh.subscribe("/local_path", 1,  CB_local);
  ros::Subscriber sub_ego   = nh.subscribe("/Ego_topic", 10, CB_ego);

  ROS_INFO("[pp_local] wheelbase=%.2f, v_set=%.2f", g_wheelbase, g_v_set);
  
  //ros::spin()은 싱글 스레드라 콜백이 동시에 돌진 않음,
  ros::spin();
  return 0;
}
