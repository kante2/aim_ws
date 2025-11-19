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

// 곡률이 큰 상태에서는 -> stanley추종
// 곡률이 작은 상태에서는 -> pure pursuit추종 (지금은 stanley만 실제 사용)

// -------------------- 전역 상태 --------------------
ros::Publisher cmd_pub;

morai_msgs::CtrlCmd g_cmd;

GeographicLib::LocalCartesian g_lc(GeographicLib::Geocentric::WGS84()); // gps -> enu(x,y,z)
bool g_have_origin = false;
bool g_have_gps   = false;
bool g_have_yaw   = false;

double g_enu_x = 0.0, g_enu_y = 0.0;
double g_yaw   = 0.0;               // [rad]
double g_ego_speed_ms = 0.0;        // [m/s] (Stanley용)

std::vector<std::pair<double,double>> g_path_xy;
std::string g_ref_file, g_path_file;

// 제어 파라미터
double g_wheelbase_L = 3.0;       // [m]
double g_lfd         = 4.5;       // [m] (기본 LD, 동적 LD 계산에 참고로 쓸 수 있음)
double g_target_vel  = 20.0;      // MORAI에 그대로 보낼 속도 (km/h라고 가정)
double g_k_stanley   = 3.0;       // Stanley gain
double g_kappa_low   = 0.005;     // 사용은 안 하지만, 나중에 weight 바꿀 때 사용 가능
double g_kappa_high  = 0.01;

double lx = 0.0;
double ly = 0.0;

// -------------------- 로더 --------------------
bool loadOrigin(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) {
    ROS_ERROR("[pp_fixed] Failed to open origin file: %s", file.c_str());
    return false;
  }
  double lat0, lon0, alt0;
  in >> lat0 >> lon0 >> alt0;
  g_lc.Reset(lat0, lon0, alt0);
  g_have_origin = true;
  ROS_INFO("[pp_fixed] ENU origin: lat=%.15f lon=%.15f alt=%.3f", lat0, lon0, alt0);
  return true;
}

bool loadPath(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) {
    ROS_ERROR("[pp_fixed] Failed to open path file: %s", file.c_str());
    return false;
  }
  g_path_xy.clear();
  std::string line;
  double x, y, z;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    if (!(iss >> x >> y >> z)) continue;
    g_path_xy.emplace_back(x, y);
  }
  ROS_INFO("[pp_fixed] Path points loaded: %zu", g_path_xy.size());
  return !g_path_xy.empty();
}

// -------------------- 퍼블리셔 --------------------
void publishStop() {
  g_cmd.steering = 0.0;
  g_cmd.velocity = 0.0;
  g_cmd.accel    = 0.0;
  g_cmd.brake    = 0.0;
  cmd_pub.publish(g_cmd);
}

// ------------------------ 1. 유틸 함수들 ------------------------
double compute_WrapAngle(double a) { // 각도 정규화 [-pi, pi]
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// 현재 ENU (x,y) 기준으로 최근접 path index
int compute_findNearestIdx(double x, double y) {
  if (g_path_xy.empty()) return -1;
  int best_idx = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < g_path_xy.size(); ++i) {
    double dx = g_path_xy[i].first  - x;
    double dy = g_path_xy[i].second - y;
    double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

// 곡률 계산 (path 상의 세 점 사용)
double compute_CurvatureAtIndex(int i) {
  int N = static_cast<int>(g_path_xy.size());
  // i, i+5, i+10 을 쓰므로 범위 체크
  if (i < 0 || i + 10 >= N) return 0.0;

  double x1 = g_path_xy[i].first;
  double y1 = g_path_xy[i].second;
  double x2 = g_path_xy[i + 5].first;
  double y2 = g_path_xy[i + 5].second;
  double x3 = g_path_xy[i + 10].first;
  double y3 = g_path_xy[i + 10].second;

  double a = std::hypot(x2 - x1, y2 - y1);
  double b = std::hypot(x3 - x2, y3 - y2);
  double c = std::hypot(x3 - x1, y3 - y1);

  const double eps_len  = 1e-6;
  const double eps_area = 1e-9;
  if (a <= eps_len || b <= eps_len || c <= eps_len) return 0.0;

  double cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
  double A = 0.5 * std::fabs(cross);
  if (A <= eps_area) return 0.0;

  double R = (a * b * c) / (4.0 * A); // 곡률반경
  if (R <= 1e-6) return 0.0;

  double kappa = 1.0 / R;             // 곡률
  double sign  = (cross >= 0.0) ? 1.0 : -1.0;
  return sign * kappa;
}

// 곡률 기반 동적 LD 계산
double compute_dynamic_ld_with_curvature(double curvature_abs)
{
  // 곡률이 클수록 LD 작게, 직선에 가까울수록 LD 크게
  const double ld_min = 3.0;   // 급커브일 때 LD
  const double ld_max = 8.0;   // 직선일 때 LD

  const double c_min = 0.0009;
  const double c_max = 0.05;

  if (curvature_abs <= c_min) return ld_max;
  if (curvature_abs >= c_max) return ld_min;

  double alpha = (curvature_abs - c_min) / (c_max - c_min); // 0~1
  return ld_max - alpha * (ld_max - ld_min);
}

// 현재 yaw/위치 기준으로, "거리 >= ld" 가 되는 path index 찾기
int findIndexWithLD(int nearest_idx, double ld)
{
  if (g_path_xy.empty()) return -1;

  const double c = std::cos(g_yaw);
  const double s = std::sin(g_yaw);
  const int N = static_cast<int>(g_path_xy.size());

  for (int i = nearest_idx; i < N; ++i)
  {
    double dx = g_path_xy[i].first  - g_enu_x;
    double dy = g_path_xy[i].second - g_enu_y;

    // ENU -> 차량 좌표계 (x: 전방, y: 좌측)
    double x_local =  c * dx + s * dy;
    double y_local = -s * dx + c * dy;

    if (x_local <= 0.0) continue;  // 뒤쪽 점은 무시

    double d = std::hypot(x_local, y_local);
    if (d >= ld)
      return i;
  }

  // 못 찾으면 최근접 인덱스 사용
  return nearest_idx;
}

// ----------------- 제어부 compute 함수 -----------------
double compute_PurePursuitSteering(double lx, double ly, double ld) {
  const double theta = std::atan2(ly, lx);
  const double delta = std::atan2(2.0 * g_wheelbase_L * std::sin(theta), ld);
  return delta;
}

double compute_StanleySteering(int idx) {
  int N = static_cast<int>(g_path_xy.size());
  if (N < 2 || idx < 0 || idx >= N) {
    return 0.0;
  }

  // === 1) idx번째 점의 차량 좌표계 위치 → 횡방향 오차 e_y ===
  const double cos_y = std::cos(g_yaw);
  const double sin_y = std::sin(g_yaw);

  double dx_n = g_path_xy[idx].first  - g_enu_x;
  double dy_n = g_path_xy[idx].second - g_enu_y;

  double x_local_n =  cos_y * dx_n + sin_y * dy_n;  // 전방 (+x)
  double y_local_n = -sin_y * dx_n + cos_y * dy_n;  // 좌측 (+y)

  double e_y = y_local_n;

  // === 2) 경로 진행 방향 psi_path 계산 ===
  int idx2;
  if (idx + 1 < N) {
    idx2 = idx + 1;
  } else {
    idx2 = idx - 1;
  }

  double dx_tan = g_path_xy[idx2].first  - g_path_xy[idx].first;
  double dy_tan = g_path_xy[idx2].second - g_path_xy[idx].second;

  double psi_path = std::atan2(dy_tan, dx_tan);

  // === 3) 헤딩 오차 psi_err ===
  double psi_err = compute_WrapAngle(psi_path - g_yaw);

  // === 4) Stanley 조향각 ===
  double v = g_ego_speed_ms;      // [m/s]
  if (std::fabs(v) < 0.1) v = 0.1;

  double delta = psi_err + std::atan2(g_k_stanley * e_y, v);
  return delta;
}

// -------------------- 콜백 --------------------
void gpsCB(const morai_msgs::GPSMessage &msg) {
  if (!g_have_origin) return;
  double x, y, z;
  try {
    g_lc.Forward(msg.latitude, msg.longitude,
                 std::isfinite(msg.altitude) ? msg.altitude : 0.0,
                 x, y, z);
  } catch (...) {
    ROS_WARN_THROTTLE(1.0, "[pp_fixed] LocalCartesian.Forward failed");
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
  g_ego_speed_ms = msg.velocity.x;  // MORAI가 m/s로 준다고 가정
}

// -------------------- 메인 --------------------
int main(int argc, char **argv) {
  ros::init(argc, argv, "pp_fixed_speed");
  ros::NodeHandle nh;

  // 파라미터
  nh.param<std::string>("ref_file",   g_ref_file,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
  nh.param<std::string>("path_file",  g_path_file, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
  nh.param<double>("wheelbase",  g_wheelbase_L, 3.0);
  nh.param<double>("lookahead",  g_lfd,        4.5);
  nh.param<double>("target_kmh", g_target_vel, 16.0);

  // 경로/원점 로딩
  if (!loadOrigin(g_ref_file)) {
    ROS_WARN("[pp_fixed] Origin not loaded. GPS -> ENU 변환 불가.");
  }
  if (!loadPath(g_path_file)) {
    ROS_WARN("[pp_fixed] Path not loaded.");
  }

  // Sub
  ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, egoCB);
  ros::Subscriber gps_sub = nh.subscribe("/gps",       1, gpsCB);
  ros::Subscriber imu_sub = nh.subscribe("/imu",       1, imuCB);

  // Pub
  cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

  // 제어 모드 설정
  g_cmd.longlCmdType = 2;  // 2: velocity
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;

  ros::Rate rate(50); // 50 Hz
  while (ros::ok()) {
    ros::spinOnce();

    // 준비 안 됐으면 정지
    if (!g_have_origin || !g_have_gps || !g_have_yaw || g_path_xy.empty()) {
      publishStop();
      rate.sleep();
      continue;
    }

    // 1) 최근접 인덱스
    int nearest_idx = compute_findNearestIdx(g_enu_x, g_enu_y);
    if (nearest_idx < 0) {
      publishStop();
      rate.sleep();
      continue;
    }

    // 2) 곡률 계산
    double curvature     = compute_CurvatureAtIndex(nearest_idx);
    double curvature_abs = std::fabs(curvature);

    // 3) 곡률 기반 동적 LD 계산
    double dynamic_ld = compute_dynamic_ld_with_curvature(curvature_abs);
    ROS_INFO_THROTTLE(1.0, "[pp_fixed] kappa=%.5f | ld=%.2f", curvature, dynamic_ld);

    // 4) LD를 만족하는 인덱스 선택
    int ld_idx = findIndexWithLD(nearest_idx, dynamic_ld);

    // 5) Pure Pursuit용 타겟점 (차량 좌표계)
    double cos = std::cos(g_yaw);
    double sin = std::sin(g_yaw);

    double dx = g_path_xy[ld_idx].first  - g_enu_x;
    double dy = g_path_xy[ld_idx].second - g_enu_y;

    lx =  cos * dx + sin * dy;   // 전방(+x)
    ly = -sin * dx + cos * dy;   // 좌측(+y)

    if (lx <= 0.0) {
      // 타겟이 뒤쪽이면 정지
      publishStop();
      rate.sleep();
      continue;
    }

    // 6) 스티어링 계산
    double delta_pp      = compute_PurePursuitSteering(lx, ly, dynamic_ld);
    double delta_stanley = compute_StanleySteering(ld_idx);

    // 지금은 Stanley만 사용 (나중에 weight_steering으로 fusion 가능)
    double delta = delta_stanley;

    g_cmd.steering = delta;
    g_cmd.velocity = g_target_vel;   // km/h라고 가정

    g_cmd.accel = 0.0;
    g_cmd.brake = 0.0;

    cmd_pub.publish(g_cmd);
    rate.sleep();
  }

  return 0;
}
