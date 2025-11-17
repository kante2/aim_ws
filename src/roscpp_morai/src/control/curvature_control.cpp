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
// 곡률이 작은 상태에서는 -> pure pursuit추종

// -------------------- 전역 상태 --------------------
ros::Publisher cmd_pub;

morai_msgs::CtrlCmd g_cmd;

GeographicLib::LocalCartesian g_lc(GeographicLib::Geocentric::WGS84()); // gps -> enu(x,y,z)
bool g_have_origin = false;
bool g_have_gps = false, g_have_yaw = false;
int idx2;
double weight_steering = 0.5;
double delta_fusion = 0.0;

double g_enu_x = 0.0, g_enu_y = 0.0;
double g_yaw = 0.0;               // [rad]
double g_ego_speed_ms = 0.0;      // 로그용

std::vector<std::pair<double,double>> g_path_xy;
std::string g_ref_file, g_path_file;

// 제어 파라미터
double g_wheelbase_L = 3.0;       // [m]
double g_lfd = 4.5;               // [m]
double g_target_vel = 20.0;       // (여기선 그대로 km/h로 사용)
double g_k_stanley  = 3.0;        // Stanley gain
double g_kappa_low = 0.005;
double g_kappa_high = 0.01;


double lx = 0;
double ly = 0;

// -------------------- 로더 --------------------
bool loadOrigin(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) return false;
  double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
  g_lc.Reset(lat0, lon0, alt0);
  g_have_origin = true;
  ROS_INFO("[pp_fixed] ENU origin: lat=%.15f lon=%.15f alt=%.3f", lat0, lon0, alt0);
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
  ROS_INFO("[pp_fixed] Path points loaded: %zu", g_path_xy.size());
  return !g_path_xy.empty();
}
// -------------------- 퍼블리셔 --------------------
void publishStop() {
  g_cmd.steering = 0.0;
  g_cmd.velocity = 0.0;
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;
  cmd_pub.publish(g_cmd);
}

// --------------Ccompute functions--------------------------------

// d 부분이 계속 업데이트 **수정중
// double compute_LD_points(double x_local, double y_local, double d){ // 내 포인트 ~ lookahead 점 거리를 계산하여, 임계를 넘으면 해당 좌표를 출력하도록 한다.
//     if (d <= g_lfd) {
//         return x_local, y_local; // 그대로 반환
//     }
//     // LD 기반 거리가 넘을때까지 계산 후, LX, LY 재계산

//     lx = x_local;
//     ly = y_local;
//     return lx, ly
// }

int compute_findNearestIdx(double x, double y) {
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

double compute_WrapAngle(double a) { // 각도 정규화 [-pi, pi]
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

double compute_CurvatureAtIndex(int i) {
    int N = static_cast<int>(g_path_xy.size());
    if (i <= 0 || i >= N - 1) return 0.0;

    // double x1 = g_path_xy[i - 1].first;
    // double y1 = g_path_xy[i - 1].second;
    // double x2 = g_path_xy[i].first;
    // double y2 = g_path_xy[i].second;
    // double x3 = g_path_xy[i + 1].first;
    // double y3 = g_path_xy[i + 1].second;

    double x1 = g_path_xy[i].first;
    double y1 = g_path_xy[i].second;
    double x2 = g_path_xy[i + 5].first;
    double y2 = g_path_xy[i + 5].second;
    double x3 = g_path_xy[i + 10].first;
    double y3 = g_path_xy[i + 10].second;
    // 여기를 LD기반으로 바꾸도록,, 설정,

    double a = std::hypot(x2 - x1, y2 - y1);
    double b = std::hypot(x3 - x2, y3 - y2);
    double c = std::hypot(x3 - x1, y3 - y1);

    const double eps_len  = 1e-6;
    const double eps_area = 1e-9;
    if (a <= eps_len || b <= eps_len || c <= eps_len) return 0.0;

    double cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    double A = 0.5 * std::fabs(cross);
    if (A <= eps_area) return 0.0;

    double R = (a * b * c) / (4.0 * A); // 곡률반경, 
    if (R <= 1e-6) return 0.0;

    double kappa = 1.0 / R; // 곡률,
    double sign = (cross >= 0.0) ? 1.0 : -1.0;
    return sign * kappa;
}

double compute_PurePursuitSteering(double lx, double ly) {
    const double theta = std::atan2(ly, lx);
    const double delta = std::atan2(2.0 * g_wheelbase_L * std::sin(theta), g_lfd);
    return delta; // g_cmd.steering = delta;
}

double compute_StanleySteering(int nearest_idx) {
    // ---------- Stanley Controller ----------
    double c = std::cos(g_yaw), s = std::sin(g_yaw);
    int N = static_cast<int>(g_path_xy.size()); //이런 값들은 전역으로 뺴는게 나아보임,,

    // 1) 최근접 점의 차량좌표 (e_y)
    const double dx_n = g_path_xy[nearest_idx].first  - g_enu_x;
    const double dy_n = g_path_xy[nearest_idx].second - g_enu_y;
    const double x_local_n =  c*dx_n + s*dy_n;     // 전방(+x)
    const double y_local_n = -s*dx_n + c*dy_n;     // 좌(+y)
    const double e_y = y_local_n;                  // 횡방향 편차 (좌측이 +)

    // 2) 경로 진행방향 psi_path (최근접-이웃 간 기울기)
    
    if (nearest_idx + 1 < N) idx2 = nearest_idx + 1;
    else if (nearest_idx - 1 >= 0) idx2 = nearest_idx - 1;
    else idx2 = nearest_idx;

    double dx_tan = g_path_xy[idx2].first  - g_path_xy[nearest_idx].first;
    double dy_tan = g_path_xy[idx2].second - g_path_xy[nearest_idx].second;
    double psi_path = std::atan2(dy_tan, dx_tan);

    // 3) 헤딩 오차
    double psi_err = compute_WrapAngle(psi_path - g_yaw);

    // 4) Stanley 조향 (소프트닝 제거: atan2(k*e_y, v))
    double v = g_ego_speed_ms; 
    double delta = psi_err + std::atan2(g_k_stanley * e_y, v); //CONST도 굳이. ?? g_k_stanley-> 3// 횡오차 , 헤딩오차를 이용,
    return delta; // g_cmd.steering = delta;
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
  g_ego_speed_ms = msg.velocity.x;
}

// -------------------- 메인 루프 --------------------
// 메인을 가볍게 만들어야 함.. ??
int main(int argc, char **argv) {
  ros::init(argc, argv, "pp_fixed_speed");
  ros::NodeHandle nh;  

  // 파라미터
  nh.param<std::string>("ref_file",   g_ref_file,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
  nh.param<std::string>("path_file",  g_path_file, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
  nh.param<double>("wheelbase", g_wheelbase_L, 3.0);
  nh.param<double>("lookahead", g_lfd, 4.5);
  nh.param<double>("target_kmh", g_target_vel, 16.0);

  ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 20, egoCB);
  
  // Pub/Sub

  ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCB);
  ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCB);

  cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

  // 제어 모드 설정
  g_cmd.longlCmdType = 2;  // 2: velocity
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;

  ros::Rate rate(50); // 최대 50hz, - 15로 하면 센서 데이터도 15hz로 받는 상태가 되는..
  while (ros::ok()) {
    ros::spinOnce();

    // check nearest index
    const int nearest_idx = compute_findNearestIdx(g_enu_x, g_enu_y);

    // 전방에서 lookahead 이상 첫 점 선택
    double lx = 0.0, ly = 0.0; bool found = false;
    double c = std::cos(g_yaw), s = std::sin(g_yaw);
    int N = static_cast<int>(g_path_xy.size());
    for (int i = nearest_idx; i < N; ++i) {
      double dx = g_path_xy[i].first  - g_enu_x;
      double dy = g_path_xy[i].second - g_enu_y;
      double x_local =  c*dx + s*dy;   // 전방(+x)
      double y_local = -s*dx + c*dy;   // 좌(+y)
      if (x_local > 0.0) { // 전방,,
        double d = std::hypot(x_local, y_local);
        // 함수로 파기,,
        lx, ly = compute_LD_points(x_local, y_local, d);
        // --> if (d >= g_lfd) { lx = x_local; ly = y_local; found = true; break; }
      }
    }
    if (!found) {
      const double dx = g_path_xy.back().first  - g_enu_x;
      const double dy = g_path_xy.back().second - g_enu_y;
      lx =  c*dx + s*dy;
      ly = -s*dx + c*dy;
      found = (lx > 0.0);
    }
    if (!found) {
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] forward point not found (lfd=%.1f)", g_lfd);
      publishStop();
      rate.sleep(); continue;
    }
    // --------------------------------------------------------------
    // 경로의 곡률을 계산한다.
    double curvature = compute_CurvatureAtIndex(nearest_idx);
    double curvature_abs = std::fabs(curvature);

    double delta_pp = compute_PurePursuitSteering(lx, ly);
    double delta_stanley = compute_StanleySteering(nearest_idx);
    double t = 0.0;

    // 1.곡률이 작다 --> Pure Pursuit 조향 가중치를 늘린다.
    // 2.곡률이 크다 --> Stanley 조향 가중치를 늘린다.
    // 곡률에 따라 동적으로 늘릴수 있도록 한다.

    if (curvature_abs <= g_kappa_low) {
        weight_steering = 1.0; // Pure Pursuit만,
        ROS_INFO_THROTTLE(1.0, "[fusion][pp] curvature=%.4f weight_steering=%.2f --> pp WEIGHT!", curvature, weight_steering);
        g_cmd.velocity = g_target_vel;
    } else if (curvature_abs >= g_kappa_high) {
        weight_steering = 0.0; // Stanley만,
        g_cmd.velocity = g_target_vel - 3.5; // 곡률 클땐 속도감소
        ROS_INFO_THROTTLE(1.0, "[fusion][stanley] curvature=%.4f weight_steering=%.2f --> st WEIGHT!", curvature, weight_steering);
    } else {
        // 선형 보간
        // 곡률이 커질수록, (high - curv_abs)의 차이가 줄어들기에, T 가 작아진다. --> 곡선구간, --> pp가 줄어든다.

        // (g_kappa_high - curvature_abs) 이값이 줄어들면 --> 곡선에 가까워진다. --> pp가 줄어든다., 
        weight_steering = (g_kappa_high - curvature_abs) / (g_kappa_high - g_kappa_low); 
        ROS_INFO_THROTTLE(1.0, "[fusion][fusion] curvature=%.4f weight_steering=%.2f --> FUSION WEIGHT!", curvature, weight_steering);
        g_cmd.velocity = g_target_vel;
    }

    delta_fusion = delta_pp * (weight_steering) + delta_stanley * (1.0 - weight_steering);

    // 스티어 제어
    g_cmd.steering = delta_fusion;
    // 속도: 파라미터 값 사용 (여기서는 그대로 km/h 필드에 넣던 기존 코드 유지)
    // g_cmd.velocity = g_target_vel;  // 필요 시 (g_target_vel/3.6) 로 바꿔 m/s 제어

    g_cmd.accel = 0.0;
    g_cmd.brake = 0.0;
    cmd_pub.publish(g_cmd);

    // ROS_INFO_THROTTLE(0.5,
    //   "[pp_fixed] ENU(%.2f, %.2f) yaw=%.1fdeg steer=%.3frad v_set=%.2f(km/h) (near=%d)",
    //   g_enu_x, g_enu_y, g_yaw*180.0/M_PI, g_cmd.steering, g_cmd.velocity, nearest_idx);

    rate.sleep();
  }
  return 0;
}

