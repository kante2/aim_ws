#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <morai_msgs/GPSMessage.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

struct Pt {
  double x, y;  // ENU [m]
};

class PathVisualizer {
public:
  PathVisualizer()
  : nh_("~"),
    lc_(GeographicLib::Geocentric::WGS84()),
    have_origin_(false),
    have_gps_(false)
  {
    // ---- params ----
    nh_.param<std::string>("ref_file",  ref_file_,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
    nh_.param<std::string>("path_file", path_file_, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
    nh_.param<std::string>("frame_id",  frame_id_,  std::string("map"));
    nh_.param<double>("highlight_dist", highlight_dist_, 30.0); // 빨간 강조 길이 [m]
    nh_.param<double>("line_width",     line_width_,     0.2);  // 라인 두께 [m]
    nh_.param<bool>("show_current",     show_current_,   true); // 현재 위치 마커 표시

    if (!loadOrigin(ref_file_)) {
      ROS_FATAL("[path_vis] failed to load ENU origin from %s", ref_file_.c_str());
      ros::shutdown();
      return;
    }
    if (!loadPath(path_file_)) {
      ROS_FATAL("[path_vis] failed to load path from %s", path_file_.c_str());
      ros::shutdown();
      return;
    }

    // pubs/subs
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    path_pub_   = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    gps_sub_    = nh_.subscribe("/gps", 20, &PathVisualizer::gpsCB, this);

    // 초기 전체 경로와 회색 마커 1회 송신
    publishPathMsg();
    publishFullPathMarker();

    // 타이머(또는 GPS 콜백에서 그때그때 그려도 됨)
    timer_ = nh_.createTimer(ros::Duration(0.1), &PathVisualizer::timerCB, this);
  }

private:
  // --------- IO ----------
  bool loadOrigin(const std::string& file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
    lc_.Reset(lat0, lon0, alt0);
    have_origin_ = true;
    ROS_INFO("[path_vis] ENU origin: lat=%.9f lon=%.9f alt=%.3f", lat0, lon0, alt0);
    return true;
  }

  bool loadPath(const std::string& file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    path_.clear();
    std::string line; double x, y, z;
    while (std::getline(in, line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      if (!(iss >> x >> y >> z)) continue;
      path_.push_back({x, y});
    }
    ROS_INFO("[path_vis] loaded %zu path points", path_.size());
    return !path_.empty();
  }

  // --------- Callbacks ----------
  void gpsCB(const morai_msgs::GPSMessage& msg) {
    if (!have_origin_) return;
    double x, y, z;
    try {
      lc_.Forward(msg.latitude, msg.longitude,
                  std::isfinite(msg.altitude) ? msg.altitude : 0.0,
                  x, y, z);
    } catch (...) {
      ROS_WARN_THROTTLE(1.0, "[path_vis] LocalCartesian.Forward failed");
      return;
    }
    enu_x_ = x; enu_y_ = y; have_gps_ = true;
  }

  void timerCB(const ros::TimerEvent&) {
    // 전체 경로는 1회 발행해도 되지만, RViz 재시작 대비 가끔 재송신해도 무방
    static int cnt = 0;
    if ((cnt++ % 50) == 0) {
      publishFullPathMarker();
      publishPathMsg();
    }

    if (!have_gps_) return;

    // 현재 위치에서 최근접 인덱스 찾기
    const int nearest_idx = findNearestIdx(enu_x_, enu_y_);
    if (nearest_idx < 0) return;

    // 최근접부터 앞으로 highlight_dist_ 누적
    std::vector<geometry_msgs::Point> ahead_pts;
    ahead_pts.reserve(128);

    geometry_msgs::Point p;
    p.x = path_[nearest_idx].x; p.y = path_[nearest_idx].y; p.z = 0.0;
    ahead_pts.push_back(p);

    double acc = 0.0;
    for (size_t i = nearest_idx + 1; i < path_.size(); ++i) {
      const double dx = path_[i].x - path_[i-1].x;
      const double dy = path_[i].y - path_[i-1].y;
      const double ds = std::hypot(dx, dy);
      acc += ds;
      geometry_msgs::Point q;
      q.x = path_[i].x; q.y = path_[i].y; q.z = 0.0;
      ahead_pts.push_back(q);
      if (acc >= highlight_dist_) break;
    }

    // 빨간 라인 마커
    visualization_msgs::Marker red;
    red.header.frame_id = frame_id_;
    red.header.stamp = ros::Time::now();
    red.ns = "path_vis";
    red.id = 2;
    red.type = visualization_msgs::Marker::LINE_STRIP;
    red.action = visualization_msgs::Marker::ADD;
    red.scale.x = line_width_;
    red.color.r = 1.0;
    red.color.g = 0.0;
    red.color.b = 0.0;
    red.color.a = 1.0;
    red.pose.orientation.w = 1.0;
    red.points = ahead_pts;
    marker_pub_.publish(red);

    // 현재 위치 마커(옵션)
    if (show_current_) {
      visualization_msgs::Marker cur;
      cur.header.frame_id = frame_id_;
      cur.header.stamp = ros::Time::now();
      cur.ns = "path_vis";
      cur.id = 3;
      cur.type = visualization_msgs::Marker::SPHERE;
      cur.action = visualization_msgs::Marker::ADD;
      cur.scale.x = cur.scale.y = cur.scale.z = std::max(1.5*line_width_, 0.3); // 보기 좋은 점 크기
      cur.color.r = 1.0; cur.color.g = 1.0; cur.color.b = 0.0; cur.color.a = 1.0; // 노랑
      cur.pose.position.x = enu_x_;
      cur.pose.position.y = enu_y_;
      cur.pose.position.z = 0.2;
      cur.pose.orientation.w = 1.0;
      marker_pub_.publish(cur);
    }
  }

  // --------- Publishers ----------
  void publishPathMsg() {
    nav_msgs::Path p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = frame_id_;
    p.poses.reserve(path_.size());
    for (const auto& pt : path_) {
      geometry_msgs::PoseStamped ps;
      ps.header = p.header;
      ps.pose.position.x = pt.x;
      ps.pose.position.y = pt.y;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps);
    }
    path_pub_.publish(p);
  }

  void publishFullPathMarker() {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = ros::Time::now();
    m.ns = "path_vis";
    m.id = 1;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = line_width_;
    m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.6; m.color.a = 1.0; // 회색
    m.pose.orientation.w = 1.0;
    m.points.reserve(path_.size());
    for (const auto& pt : path_) {
      geometry_msgs::Point gp;
      gp.x = pt.x; gp.y = pt.y; gp.z = 0.0;
      m.points.push_back(gp);
    }
    marker_pub_.publish(m);
  }

  // --------- Utils ----------
  int findNearestIdx(double x, double y) const {
    if (path_.empty()) return -1;
    int best = -1; double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_.size(); ++i) {
      const double dx = path_[i].x - x;
      const double dy = path_[i].y - y;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(i); }
    }
    return best;
  }

private:
  ros::NodeHandle nh_;

  // pubs/subs
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Subscriber gps_sub_;
  ros::Timer timer_;

  // ENU origin
  GeographicLib::LocalCartesian lc_;
  bool have_origin_;

  // state
  bool   have_gps_;
  double enu_x_{0.0}, enu_y_{0.0};

  // path
  std::vector<Pt> path_;
  std::string ref_file_, path_file_, frame_id_;

  // params
  double highlight_dist_;
  double line_width_;
  bool show_current_;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_visualizer");
  PathVisualizer node;
  ros::spin();
  return 0;
}
