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
double g_ego_speed_ms = 0.0;      // 로그용

std::vector<std::pair<double,double>> g_path_xy;
std::string g_ref_file, g_path_file;

// 제어 파라미터
double g_wheelbase_L = 3.0;       // [m]
double g_lfd = 4.5;               // [m]
double g_target_vel = 20.0;       // (여기선 그대로 km/h로 사용)

// -------------------- 유틸/로더 --------------------
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
  ros::init(argc, argv, "pp_fixed_speed");
  ros::NodeHandle nh;          // global
  ros::NodeHandle pnh("~");    // private

  // 파라미터
  pnh.param<std::string>("ref_file",   g_ref_file,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
  pnh.param<std::string>("path_file",  g_path_file, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
  pnh.param<double>("wheelbase", g_wheelbase_L, 3.0);
  pnh.param<double>("lookahead", g_lfd, 4.5);
  pnh.param<double>("target_kmh", g_target_vel, 20.0);

  if (!loadOrigin(g_ref_file)) {
    ROS_FATAL("[pp_fixed] Failed to load ENU origin from %s", g_ref_file.c_str());
    return 1;
  }
  if (!loadPath(g_path_file)) {
    ROS_FATAL("[pp_fixed] Failed to load path from %s", g_path_file.c_str());
    return 1;
  }

  // Pub/Sub
  path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1, true);
  publishPathOnce();

  ros::Subscriber gps_sub = nh.subscribe("/gps", 20, gpsCB);
  ros::Subscriber imu_sub = nh.subscribe("/imu", 50, imuCB);
  ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 20, egoCB);

  cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

  // 제어 모드 설정
  g_cmd.longlCmdType = 2;  // 2: velocity
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;

  ros::Rate rate(15);
  while (ros::ok()) {
    ros::spinOnce();

    if (!g_have_gps) {
      ROS_INFO_THROTTLE(1.0, "[pp_fixed] waiting /gps ...");
      rate.sleep(); continue;
    }
    if (!g_have_yaw) {
      ROS_INFO_THROTTLE(1.0, "[pp_fixed] waiting /imu/data (yaw) ...");
      rate.sleep(); continue;
    }

    const int nearest_idx = findNearestIdx(g_enu_x, g_enu_y);
    if (nearest_idx < 0) {
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] nearest index not found");
      publishStop();
      rate.sleep(); continue;
    }

    // 전방에서 lookahead 이상 첫 점 선택
    double lx = 0.0, ly = 0.0; bool found = false;
    const double c = std::cos(g_yaw), s = std::sin(g_yaw);
    const int N = static_cast<int>(g_path_xy.size());
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
    if (!found) {
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] forward point not found (lfd=%.1f)", g_lfd);
      publishStop();
      rate.sleep(); continue;
    }

    // Pure Pursuit 조향
    const double theta = std::atan2(ly, lx);
    const double delta = std::atan2(2.0 * g_wheelbase_L * std::sin(theta), g_lfd);
    g_cmd.steering = delta;

    // 속도: 파라미터 값 사용 (여기서는 그대로 km/h 필드에 넣던 기존 코드 유지)
    g_cmd.velocity = g_target_vel;  // 필요 시 (g_target_vel/3.6) 로 바꿔 m/s 제어

    g_cmd.accel = 0.0;
    g_cmd.brake = 0.0;
    cmd_pub.publish(g_cmd);

    ROS_INFO_THROTTLE(0.5,
      "[pp_fixed] ENU(%.2f, %.2f) yaw=%.1fdeg steer=%.3frad v_set=%.2f(km/h) (near=%d)",
      g_enu_x, g_enu_y, g_yaw*180.0/M_PI, g_cmd.steering, g_cmd.velocity, nearest_idx);

    rate.sleep();
  }
  return 0;
}
