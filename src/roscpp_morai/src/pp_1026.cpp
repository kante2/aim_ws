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

// Pure Pursuit + 고정 속도(10 km/h) 노드
class PPFixedSpeedNode {
public:
  PPFixedSpeedNode()
  : nh_(""),
    lc_(GeographicLib::Geocentric::WGS84()),
    have_origin_(false), have_gps_(false), have_yaw_(false) {

    // ---- 파라미터 ----
    nh_.param<std::string>("ref_file",  ref_file_,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
    nh_.param<std::string>("path_file", path_file_, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
    nh_.param<double>("wheelbase", L_, 4.6);         // 차량 휠베이스 [m]
    nh_.param<double>("lookahead", lfd_, 15.0);      // look-ahead 거리 [m]
    nh_.param<double>("target_kmh", target_vel_kmh_, 10.0); // 10km/h (고정)

    // ---- ENU 원점/경로 로드 ----
    if (!loadOrigin(ref_file_)) {
      ROS_FATAL("[pp_fixed] Failed to load ENU origin from %s", ref_file_.c_str());
      ros::shutdown();
    }
    if (!loadPath(path_file_)) {
      ROS_FATAL("[pp_fixed] Failed to load path from %s", path_file_.c_str());
      ros::shutdown();
    }

    // ---- 퍼블리셔/서브스크라이버 ----
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    publishPath();

    gps_sub_ = nh_.subscribe("/gps", 20, &PPFixedSpeedNode::gpsCB, this);
    imu_sub_ = nh_.subscribe("/imu", 50, &PPFixedSpeedNode::imuCB, this);
    ego_sub_ = nh_.subscribe("/Ego_topic", 20, &PPFixedSpeedNode::egoCB, this);

    cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

    // 제어 모드: velocity 제어
    cmd_.longlCmdType = 2;
    cmd_.accel  = 0.0;
    cmd_.brake  = 0.0;
  }

  void spin() {
    ros::Rate rate(15);
    while (ros::ok()) {
      ros::spinOnce();

      if (!have_gps_) {
        ROS_INFO_THROTTLE(1.0, "[pp_fixed] waiting /gps ...");
        rate.sleep();
        continue;
      }
      if (!have_yaw_) {
        ROS_INFO_THROTTLE(1.0, "[pp_fixed] waiting /imu/data (yaw) ...");
        rate.sleep();
        continue;
      }

      // 1) 현재 위치에서 경로 최근접 인덱스
      const int nearest_idx = findNearestIdx(enu_x_, enu_y_);
      if (nearest_idx < 0) {
        ROS_WARN_THROTTLE(1.0, "[pp_fixed] nearest index not found");
        publishStop();
        rate.sleep();
        continue;
      }

      // 2) 전방(+x)에서 lookahead 이상 첫 점 선택
      double lx = 0.0, ly = 0.0; bool found = false;
      {
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        for (int i = nearest_idx; i < static_cast<int>(path_xy_.size()); ++i) {
          const double dx = path_xy_[i].first  - enu_x_;
          const double dy = path_xy_[i].second - enu_y_;
          const double x_local =  c*dx + s*dy;  // 전방(+)
          const double y_local = -s*dx + c*dy;  // 좌(+)
          if (x_local > 0.0) {
            const double d = std::hypot(x_local, y_local);
            if (d >= lfd_) { lx = x_local; ly = y_local; found = true; break; }
          }
        }
        // 못 찾으면: 경로 끝점으로 폴백 (단, 앞쪽일 때만)
        if (!found) {
          const double dx = path_xy_.back().first  - enu_x_;
          const double dy = path_xy_.back().second - enu_y_;
          lx =  c*dx + s*dy;  ly = -s*dx + c*dy;
          found = (lx > 0.0);
        }
      }

      if (!found) {
        ROS_WARN_THROTTLE(1.0, "[pp_fixed] forward point not found (lfd=%.1f)", lfd_);
        publishStop();
        rate.sleep();
        continue;
      }

      // 3) Pure Pursuit 조향
      const double theta = std::atan2(ly, lx);                // 바디 프레임에서 목표점 각도
      const double delta = std::atan2(2.0 * L_ * std::sin(theta), lfd_);
      cmd_.steering = delta;                                  // [rad]

      // 4) 속도: 5 km/h 고정 (파라미터로 바꾸고 싶으면 target_kmh 수정)
      const double target_ms = target_vel_kmh_ / 3.6;         // 5 km/h => 1.388...
      // cmd_.velocity = target_ms;
      cmd_.velocity = target_vel_kmh_.; //임시 높은 속도
      cmd_.accel    = 0.0;
      cmd_.brake    = 0.0;

      cmd_pub_.publish(cmd_);

      ROS_INFO_THROTTLE(0.5,
        "[pp_fixed] ENU(%.2f, %.2f) yaw=%.1fdeg steer=%.3frad v_set=%.2fm/s (near=%d)",
        enu_x_, enu_y_, yaw_*180.0/M_PI, cmd_.steering, cmd_.velocity, nearest_idx);

      rate.sleep();
    }
  }

private:
  // ---------- IO ----------
  bool loadOrigin(const std::string &file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
    lc_.Reset(lat0, lon0, alt0);
    have_origin_ = true;
    ROS_INFO("[pp_fixed] ENU origin: lat=%.15f lon=%.15f alt=%.3f", lat0, lon0, alt0);
    return true;
  }

  bool loadPath(const std::string &file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    path_xy_.clear();
    std::string line; double x, y, z;
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      if (!(iss >> x >> y >> z)) continue;
      path_xy_.emplace_back(x, y);
    }
    ROS_INFO("[pp_fixed] Path points loaded: %zu", path_xy_.size());
    // publishPath();
    return !path_xy_.empty();
  }

  void publishPath() {
    nav_msgs::Path p; p.header.stamp = ros::Time::now(); 
    p.header.frame_id = "map";
    // p.header.frame_id = "base_link";
    for (auto &pt : path_xy_) {
      geometry_msgs::PoseStamped ps; ps.header = p.header;
      ps.pose.position.x = pt.first;
      ps.pose.position.y = pt.second;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps);
    }
    path_pub_.publish(p);
    path_msg_ = p;
  }

  // ---------- Callbacks ----------
  void gpsCB(const morai_msgs::GPSMessage &msg) {
    if (!have_origin_) return;
    double x, y, z;
    try {
      lc_.Forward(msg.latitude, msg.longitude,
                  std::isfinite(msg.altitude) ? msg.altitude : 0.0,
                  x, y, z);
    } catch (...) {
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] LocalCartesian.Forward failed");
      return;
    }
    enu_x_ = x; enu_y_ = y; have_gps_ = true;
  }

  void imuCB(const sensor_msgs::Imu &msg) {
    const geometry_msgs::Quaternion &q = msg.orientation;
    tf::Quaternion tfq(q.x, q.y, q.z, q.w);
    yaw_ = tf::getYaw(tfq); // [rad]
    have_yaw_ = true;
  }

  void egoCB(const morai_msgs::EgoVehicleStatus &msg) {
    ego_speed_ms_ = msg.velocity.x; // m/s (로그용)
  }

  // ---------- Utils ----------
  int findNearestIdx(double x, double y) const {
    if (path_xy_.empty()) return -1;
    int best = -1; double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_xy_.size(); ++i) {
      const double dx = path_xy_[i].first  - x;
      const double dy = path_xy_[i].second - y;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(i); }
    }
    return best;
  }

  void publishStop() {
    cmd_.steering = 0.0;
    cmd_.velocity = 0.0;  // velocity 제어 모드에서 정지
    cmd_.accel = 0.0;
    cmd_.brake = 0.0;
    cmd_pub_.publish(cmd_);
  }

private:
  ros::NodeHandle nh_;

  // pubs/subs
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber ego_sub_;
  ros::Publisher  cmd_pub_;
  ros::Publisher  path_pub_;

  nav_msgs::Path path_msg_;
  morai_msgs::CtrlCmd cmd_;

  // ENU origin
  GeographicLib::LocalCartesian lc_;
  bool have_origin_;

  // state
  bool   have_gps_, have_yaw_;
  double enu_x_{0.0}, enu_y_{0.0};
  double yaw_{0.0};            // rad (from IMU)

  double ego_speed_ms_{0.0};   // from /Ego_topic (로그)

  // path (ENU)
  std::vector<std::pair<double,double>> path_xy_;
  std::string ref_file_, path_file_;

  // control params
  double L_;                 // wheelbase [m]
  double lfd_;               // lookahead [m]
  double target_vel_kmh_;    // 기본 5 km/h
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pp_fixed_speed");
  PPFixedSpeedNode node;
  node.spin();
  return 0;
}
