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

#include "roscpp_morai_2/control_file.hpp"
#include "roscpp_morai_2/control_compute.hpp"

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
double g_kappa_high = 0.001;

// -------------------- 유틸/로더 --------------------

void publishStop() {
  g_cmd.steering = 0.0;
  g_cmd.velocity = 0.0;
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;
  cmd_pub.publish(g_cmd);
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
int main(int argc, char **argv) {
  ros::init(argc, argv, "curvature_control_clear");
  ros::NodeHandle nh; 
  // 파라미터 -> init
  nh.param<std::string>("ref_file",   g_ref_file,  std::string("/root/ws/src/roscpp_morai_2/map/ref.txt"));
  nh.param<std::string>("path_file",  g_path_file, std::string("/root/ws/src/roscpp_morai_2/map/Path.txt"));
  nh.param<double>("wheelbase", g_wheelbase_L, 3.0);
  nh.param<double>("lookahead", g_lfd, 4.5);
  nh.param<double>("target_kmh", g_target_vel, 16.0);

  ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 20, egoCB);
  
  // === 파일 로딩 ===
  if (!roscpp_morai_2::loadOrigin(g_ref_file, g_lc, g_have_origin)) { // [FIX: 인자 추가 (lc, have_origin)]
    ROS_FATAL("[pp_fixed] Failed to load ENU origin from %s", g_ref_file.c_str());
    return 1;
  }
  if (!roscpp_morai_2::loadPath(g_path_file, g_path_xy)) { // [FIX: 인자 추가 (path_xy)]
    ROS_FATAL("[pp_fixed] Failed to load path from %s", g_path_file.c_str());
    return 1;
  }

  // Pub/Sub
  ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCB);
  ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCB);
  cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

  // 제어 모드 설정
  g_cmd.longlCmdType = 2;  // 2: velocity
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;

  ros::Rate rate(50); // 최대 50hz
  while (ros::ok()) {
    ros::spinOnce();


    // === 최근접 인덱스 ===
    const int nearest_idx = roscpp_morai_2::compute_NearestIdx(g_path_xy, g_enu_x, g_enu_y); // [FIX: path 전달]

    // 전방에서 lookahead 이상 첫 점 선택
    double lx = 0.0, ly = 0.0; bool found = false;
    double c = std::cos(g_yaw), s = std::sin(g_yaw); // ?? 
    int N = static_cast<int>(g_path_xy.size());
    for (int i = nearest_idx; i < N; ++i) {
      const double dx = g_path_xy[i].first  - g_enu_x;
      const double dy = g_path_xy[i].second - g_enu_y;
      const double x_local =  c*dx + s*dy;   // 전방(+x)
      const double y_local = -s*dx + c*dy;   // 좌(+y)
      if (x_local > 0.0) {
        const double d = std::hypot(x_local, y_local);
        if (d >= g_lfd) { lx = x_local; ly = y_local; found = true; break; }
      }
    }
    if (!found) {
      const double dx = g_path_xy.back().first  - g_enu_x;
      const double dy = g_path_xy.back().second - g_enu_y;
      lx =  c*dx + s*dy;
      ly = -s*dx + c*dy;
      found = (lx > 0.0);
    }

    // === 곡률/조향 계산 ===
    double curvature = roscpp_morai_2::compute_CurvatureAtIndex(g_path_xy, nearest_idx); // [FIX: path 전달]
    double curvature_abs = std::fabs(curvature);

    double delta_pp = roscpp_morai_2::compute_PurePursuitSteering(lx, ly, g_wheelbase_L, g_lfd); // [FIX: wheelbase, lfd 전달]
    double delta_stanley = roscpp_morai_2::compute_StanleySteering( // [FIX: 모든 필요한 인자 전달]
        nearest_idx, g_yaw, g_ego_speed_ms, g_k_stanley, g_enu_x, g_enu_y, g_path_xy);

    double t = 0.0; // (선형 보간시 내부 계산용)

    // 1.곡률이 작다 --> Pure Pursuit 조향 가중치를 늘린다.
    // 2.곡률이 크다 --> Stanley 조향 가중치를 늘린다.

    // 스탠리에 LD
    if (curvature_abs <= g_kappa_low) {
        weight_steering = 1.0; // Pure Pursuit만,
        ROS_INFO_THROTTLE(1.0, "[fusion][pp] curvature=%.4f weight_steering=%.2f --> pp WEIGHT!", curvature, weight_steering);
    } else if (curvature_abs >= g_kappa_high) {
        weight_steering = 0.0; // Stanley만,
        ROS_INFO_THROTTLE(1.0, "[fusion][stanley] curvature=%.4f weight_steering=%.2f --> st WEIGHT!", curvature, weight_steering);
    } else {
        // 선형 보간
        weight_steering = (g_kappa_high - curvature_abs) / (g_kappa_high - g_kappa_low);
        ROS_INFO_THROTTLE(1.0, "[fusion][fusion] curvature=%.4f weight_steering=%.2f --> FUSION WEIGHT!", curvature, weight_steering);
    }
    delta_fusion = delta_pp * (weight_steering) + delta_stanley * (1.0 - weight_steering);

    // 스티어/속도 퍼블리시
    g_cmd.steering = delta_fusion;
    g_cmd.velocity = g_target_vel;  // (필요 시 m/s면 g_target_vel/3.6)
    g_cmd.accel = 0.0;
    g_cmd.brake = 0.0;
    cmd_pub.publish(g_cmd);
    // pub_cmd();
    // 정리,,
    // 속도, PID,, 모드1 // ld기반,,
    // 

    rate.sleep();
  }
  return 0;
}
