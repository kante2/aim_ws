#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <tf/transform_datatypes.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

// -------------------- 전역 상태 --------------------
ros::Publisher cmd_pub, path_pub;
morai_msgs::CtrlCmd g_cmd;

GeographicLib::LocalCartesian g_lc(GeographicLib::Geocentric::WGS84());
bool g_have_origin = false;
bool g_have_gps = false, g_have_yaw = false;

double g_enu_x = 0.0, g_enu_y = 0.0;
double g_yaw = 0.0;               // [rad]
double g_ego_speed_ms = 0.0;      // [m/s] (Stanley 분모에 사용)

std::vector<std::pair<double,double>> g_path_xy;
std::string g_ref_file, g_path_file;

// 제어 파라미터
double g_wheelbase_L = 3.0;       // [m] (조향 각도 클램프 등 쓸 때 유용, 여기선 직접 사용 X)
double g_target_vel = 20.0;       // (여기선 그대로 km/h 필드 사용)
double g_k_stanley  =1.0;        // Stanley gain -> 여기는 선언부, 튜닝 자리가 아님, 메인에서 파라미터로 설정. --> 이런거 함수안에서 처리.. 

// -------------------- 유틸/로더 --------------------
bool loadOrigin(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) return false;
  double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
  g_lc.Reset(lat0, lon0, alt0);
  g_have_origin = true;
  ROS_INFO("[stanley] ENU origin: lat=%.15f lon=%.15f alt=%.3f", lat0, lon0, alt0);
  return true;
}

bool loadPath(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) return false;
  g_path_xy.clear();
  std::string line; double x, y, z;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    if (!(iss >> x >> y >> z)) continue;
    g_path_xy.emplace_back(x, y);
  }
  ROS_INFO("[stanley] Path points loaded: %zu", g_path_xy.size());
  return !g_path_xy.empty();
}

void publishPathOnce() {
  nav_msgs::Path p; 
  p.header.stamp = ros::Time::now();
  p.header.frame_id = "map";
  for (auto &pt : g_path_xy) {
    geometry_msgs::PoseStamped ps;
    ps.header = p.header;
    ps.pose.position.x = pt.first;
    ps.pose.position.y = pt.second;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  path_pub.publish(p);
}

int findNearestIdx(double x, double y) {
  if (g_path_xy.empty()) return -1;
  int best_idx = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < g_path_xy.size(); ++i) {
    const double dx = g_path_xy[i].first  - x;
    const double dy = g_path_xy[i].second - y;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) { best_d2 = d2; best_idx = static_cast<int>(i); }
  }
  return best_idx;
}

void publishStop() {
  g_cmd.steering = 0.0;
  g_cmd.velocity = 0.0;
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;
  cmd_pub.publish(g_cmd);
}

// 각도 정규화 [-pi, pi]
inline double wrapAngle(double a) {
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

// -------------------- 콜백 --------------------
void gpsCB(const morai_msgs::GPSMessage &msg) {
  if (!g_have_origin) return; // 이거 필요한건가 ?? 
  double x, y, z;
  try {
    g_lc.Forward(msg.latitude, msg.longitude,
                 std::isfinite(msg.altitude) ? msg.altitude : 0.0,
                 x, y, z);
  } catch (...) {
    ROS_WARN_THROTTLE(1.0, "[stanley] LocalCartesian.Forward failed");
    return;
  }
  g_enu_x = x;
  g_enu_y = y;
  g_have_gps = true;
}

void imuCB(const sensor_msgs::Imu &msg) {
  const geometry_msgs::Quaternion &q = msg.orientation;
  tf::Quaternion tfq(q.x, q.y, q.z, q.w);
  g_yaw = tf::getYaw(tfq);
  g_have_yaw = true;
}

void egoCB(const morai_msgs::EgoVehicleStatus &msg) {
  g_ego_speed_ms = msg.velocity.x; // [m/s]
}

// -------------------- 메인 루프 --------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "stanley_fixed_speed");
  ros::NodeHandle nh;          // global
  // 파라미터 --> 함수화, 
  nh.param<std::string>("ref_file",   g_ref_file,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
  nh.param<std::string>("path_file",  g_path_file, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
  nh.param<double>("wheelbase", g_wheelbase_L, 3.0);
  nh.param<double>("target_kmh", g_target_vel, 20.0);
  // nh.param<double>("stanley_k",  g_k_stanley,  5.0); // 튜닝 , Stanley gain , - 곡선 추종은 잘 하지만, 직선 심하게 진동.
  nh.param<double>("stanley_k",  g_k_stanley,  3.0); // 튜닝 , Stanley gain , - 직선/곡선 모두 무난한 편.- 곡선은 개선 필요,,
  // nh.param<double>("stanley_k",  g_k_stanley,  1.0); // 튜닝 , Stanley gain , - 직선 추종은 잘 하지만, 곡선 심하게 진동.

  if (!loadOrigin(g_ref_file)) {
    ROS_FATAL("[stanley] Failed to load ENU origin from %s", g_ref_file.c_str());
    return 1;
  }
  if (!loadPath(g_path_file)) {
    ROS_FATAL("[stanley] Failed to load path from %s", g_path_file.c_str());
    return 1;
  }

  // Pub/Sub
  path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1, true);
  publishPathOnce();

  ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCB);
  ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCB);
  ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, egoCB);

  cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

  // 제어 모드 설정
  g_cmd.longlCmdType = 2;  // 2: velocity
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;

  ros::Rate rate(15); // 최대 50hz, - 15로 하면 센서 데이터도 15hz로 받는 상태가 되는..
  while (ros::ok()) {
    ros::spinOnce();

    if (!g_have_gps) {
      ROS_INFO_THROTTLE(1.0, "[stanley] waiting /gps ...");
      rate.sleep(); continue;
    }
    if (!g_have_yaw) {
      ROS_INFO_THROTTLE(1.0, "[stanley] waiting /imu (yaw) ...");
      rate.sleep(); continue;
    }

    const int nearest_idx = findNearestIdx(g_enu_x, g_enu_y);
    if (nearest_idx < 0) {
      ROS_WARN_THROTTLE(1.0, "[stanley] nearest index not found");
      publishStop();
      rate.sleep(); continue;
    }

    // ---------- Stanley Controller ----------
    const double c = std::cos(g_yaw), s = std::sin(g_yaw);
    const int N = static_cast<int>(g_path_xy.size());

    // 1) 최근접 점의 차량좌표 (e_y)
    const double dx_n = g_path_xy[nearest_idx].first  - g_enu_x;
    const double dy_n = g_path_xy[nearest_idx].second - g_enu_y;
    const double x_local_n =  c*dx_n + s*dy_n;     // 전방(+x)
    const double y_local_n = -s*dx_n + c*dy_n;     // 좌(+y)
    const double e_y = y_local_n;                  // 횡방향 편차 (좌측이 +)

    // 2) 경로 진행방향 psi_path (최근접-이웃 간 기울기)
    int idx2; /// 변수명  / ??? 
    if (nearest_idx + 1 < N)      idx2 = nearest_idx + 1;
    else if (nearest_idx - 1 >= 0) idx2 = nearest_idx - 1;
    else                           idx2 = nearest_idx;

    double dx_tan = g_path_xy[idx2].first  - g_path_xy[nearest_idx].first;
    double dy_tan = g_path_xy[idx2].second - g_path_xy[nearest_idx].second;
    double psi_path = std::atan2(dy_tan, dx_tan);

    // 3) 헤딩 오차
    double psi_err = wrapAngle(psi_path - g_yaw);

    // 4) Stanley 조향 (소프트닝 제거: atan2(k*e_y, v))
    double v = std::max(0.0, g_ego_speed_ms); // [m/s] // ???
    double delta = psi_err + std::atan2(g_k_stanley * e_y, v); //CONST도 굳이. ?? 
    // 함수안 일회용 변수 --> 함수 내부에서 선언,, // ??? 

    g_cmd.steering = delta;

    // 속도: 고정 속도 명령 (기존과 동일, km/h 필드 사용)
    g_cmd.velocity = g_target_vel;
    g_cmd.accel = 0.0;
    g_cmd.brake = 0.0;
    cmd_pub.publish(g_cmd);

    ROS_INFO_THROTTLE(0.5,
      "[stanley] ENU(%.2f, %.2f) yaw=%.1fdeg psi_err=%.3frad e_y=%.2f m v=%.2f m/s steer=%.3frad (near=%d, xloc=%.2f)",
      g_enu_x, g_enu_y, g_yaw*180.0/M_PI, psi_err, e_y, v, g_cmd.steering, nearest_idx, x_local_n);

    rate.sleep();
  }
  return 0;
}

// 리펙토링
// cosnt제거
// 전역, 지역 변수 처리 


// ld는 1차로 튜닝 <-- 속도로 제어,, 
// 속도는 곡률로 튜닝, 

// 