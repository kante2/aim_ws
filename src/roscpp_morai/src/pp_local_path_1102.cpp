#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
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

class PPFixedSpeedNode {
public:
  PPFixedSpeedNode()
  : nh_(""),
    lc_(GeographicLib::Geocentric::WGS84()),
    have_origin_(false), have_gps_(false), have_yaw_(false),
    last_scan_stamp_(0.0)
  {
    // ----- params -----
    nh_.param<std::string>("ref_file", ref_file_, std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
    nh_.param<std::string>("path_file", path_file_, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
    nh_.param<std::string>("scan_topic", scan_topic_, std::string("/scan"));

    nh_.param<double>("wheelbase", wheelbase_L, 3.0);
    nh_.param<double>("lookahead", lfd_, 4.5);
    nh_.param<double>("target_kmh", target_vel_kmh, 15.0);

    // ROI & 회피 파라미터
    nh_.param<double>("obstacle_dist_thresh", obstacle_dist_thresh_, 2.5); // m
    nh_.param<int>("min_points_to_block", min_points_to_block_, 3);        // ROI 내 최소 포인트 수
    nh_.param<int>("clear_frames_needed", clear_frames_needed_, 4);        // 연속 클리어 프레임
    nh_.param<double>("avoid_steer_rad", avoid_steer_rad_, 0.35);          // 회피 고정 조향(라디안) ~20도
    nh_.param<double>("avoid_kmh", avoid_kmh_, 8.0);                       // 회피시 속도(km/h)

    nh_.param<double>("roi_front_x", roi_front_x_, 4.0);   // 정면 ROI 길이
    nh_.param<double>("roi_front_w", roi_front_w_, 2.0);   // 정면 ROI 폭(좌우 합)
    nh_.param<double>("roi_side_x",  roi_side_x_,  3.0);   // 좌/우 전방 ROI 길이
    nh_.param<double>("roi_side_w",  roi_side_w_,  1.2);   // 좌/우 전방 ROI 폭(한쪽)

    // ENU 원점/경로
    if (!loadOrigin(ref_file_)) { ROS_FATAL("Failed to load ENU origin"); ros::shutdown(); }
    if (!loadPath(path_file_))  { ROS_FATAL("Failed to load path");       ros::shutdown(); }

    // pubs/subs
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    publishPath();

    gps_sub_ = nh_.subscribe("/gps", 20, &PPFixedSpeedNode::gpsCB, this);
    imu_sub_ = nh_.subscribe("/imu", 50, &PPFixedSpeedNode::imuCB, this);
    ego_sub_ = nh_.subscribe("/Ego_topic", 20, &PPFixedSpeedNode::egoCB, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 20, &PPFixedSpeedNode::scanCB, this);

    cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

    // Ctrl type: velocity
    cmd_.longlCmdType = 2;
    cmd_.accel = cmd_.brake = 0.0;

    mode_ = Mode::TRACK_WAYPOINT;
    clear_frames_ = 0;
  }

  void spin() {
    ros::Rate rate(15);
    while (ros::ok()) {
      ros::spinOnce();

      if (!have_gps_) { ROS_INFO_THROTTLE(1.0, "[pp] waiting /gps ..."); rate.sleep(); continue; }
      if (!have_yaw_) { ROS_INFO_THROTTLE(1.0, "[pp] waiting /imu ..."); rate.sleep(); continue; }

      // 최신 스캔 정보로 장애물 플래그 갱신
      ObstacleFlags flags = evaluateObstacles();

      // ---- 상태머신 ----
      switch (mode_) {
        case Mode::TRACK_WAYPOINT: {
          if (flags.block_front || flags.block_left || flags.block_right) {
            // 회피 방향 결정: 더 한가한 쪽으로
            if (flags.block_front) {
              if (flags.count_left > flags.count_right) mode_ = Mode::AVOID_RIGHT;
              else                                      mode_ = Mode::AVOID_LEFT;
            } else if (flags.block_left)  mode_ = Mode::AVOID_RIGHT;
            else if (flags.block_right)   mode_ = Mode::AVOID_LEFT;
            clear_frames_ = 0;
            ROS_INFO("[pp] -> AVOID (%s)", mode_==Mode::AVOID_LEFT?"LEFT":"RIGHT");
            doAvoidStep(); // 즉시 1스텝 회피 커맨드
          } else {
            doPurePursuitStep(); // 정상 추종
          }
          break;
        }
        case Mode::AVOID_LEFT:
        case Mode::AVOID_RIGHT: {
          // ROI가 연속 clear_frames_needed 프레임 동안 비면 복귀
          if (!flags.block_front && !flags.block_left && !flags.block_right) {
            clear_frames_++;
            if (clear_frames_ >= clear_frames_needed_) {
              mode_ = Mode::TRACK_WAYPOINT;
              ROS_INFO("[pp] AVOID cleared -> TRACK_WAYPOINT");
              doPurePursuitStep();
            } else {
              doAvoidStep();
            }
          } else {
            clear_frames_ = 0;
            doAvoidStep();
          }
          break;
        }
      }

      rate.sleep();
    }
  }

private:
  // ---------------- IO/Path ----------------
  bool loadOrigin(const std::string &file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
    lc_.Reset(lat0, lon0, alt0);
    have_origin_ = true;
    ROS_INFO("[pp] ENU origin: %.9f, %.9f, alt=%.2f", lat0, lon0, alt0);
    return true;
  }

  bool loadPath(const std::string &file) {
    std::ifstream in(file);
    if (!in.is_open()) return false;
    path_xy_.clear();
    std::string line; double x,y,z;
    while (std::getline(in,line)) {
      if (line.empty()) continue;
      std::istringstream iss(line);
      if (!(iss>>x>>y>>z)) continue;
      path_xy_.emplace_back(x,y);
    }
    ROS_INFO("[pp] Path points: %zu", path_xy_.size());
    return !path_xy_.empty();
  }

  void publishPath() {
    nav_msgs::Path p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "map";
    for (auto &pt: path_xy_) {
      geometry_msgs::PoseStamped ps; ps.header=p.header;
      ps.pose.position.x=pt.first; ps.pose.position.y=pt.second;
      ps.pose.orientation.w=1.0; p.poses.push_back(ps);
    }
    path_pub_.publish(p);
    path_msg_ = p;
  }

  // ---------------- Callbacks ----------------
  void gpsCB(const morai_msgs::GPSMessage &msg) {
    if (!have_origin_) return;
    double x,y,z;
    try {
      lc_.Forward(msg.latitude, msg.longitude,
                  std::isfinite(msg.altitude) ? msg.altitude : 0.0, x,y,z);
    } catch(...) {
      ROS_WARN_THROTTLE(1.0, "[pp] LocalCartesian.Forward failed");
      return;
    }
    enu_x_=x; enu_y_=y; have_gps_=true;
  }

  void imuCB(const sensor_msgs::Imu &msg) {
    const geometry_msgs::Quaternion &q=msg.orientation;
    tf::Quaternion tfq(q.x,q.y,q.z,q.w);
    yaw_ = tf::getYaw(tfq);
    have_yaw_=true;
  }

  void egoCB(const morai_msgs::EgoVehicleStatus &msg) {
    ego_speed_ms_ = msg.velocity.x;
  }

  void scanCB(const sensor_msgs::LaserScan &scan) {
    last_scan_stamp_ = scan.header.stamp.toSec();
    // 간단화를 위해 scan의 frame이 base_link에 정렬되어 있다고 가정 (x forward, y left).
    // 그렇지 않다면 TF를 사용해 변환해야 함.
    cached_points_.clear();
    cached_points_.reserve(scan.ranges.size());
    const double a_min=scan.angle_min, a_inc=scan.angle_increment;
    for (size_t i=0;i<scan.ranges.size();++i){
      const float r = scan.ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < scan.range_min || r > scan.range_max) continue;
      const double a = a_min + a_inc * static_cast<double>(i);
      const double x = r*std::cos(a);
      const double y = r*std::sin(a);
      cached_points_.emplace_back(x,y);
    }
  }

  // ---------------- Obstacle/ROI ----------------
  struct ObstacleFlags {
    bool block_front{false};
    bool block_left{false};
    bool block_right{false};
    int  count_front{0};
    int  count_left{0};
    int  count_right{0};
  };

  inline bool inRect(double x, double y, double x0, double x1, double y0, double y1) const {
    return (x>=x0 && x<=x1 && y>=y0 && y<=y1);
  }

  ObstacleFlags evaluateObstacles() {
    ObstacleFlags f;
    if (cached_points_.empty()) return f;

    const double half_front_w = roi_front_w_/2.0;

    for (auto &pt : cached_points_) {
      const double x=pt.first, y=pt.second;
      const double d = std::hypot(x,y);
      if (d > obstacle_dist_thresh_) continue;

      // front ROI
      if (inRect(x,y, 0.0, roi_front_x_, -half_front_w, half_front_w)) {
        f.count_front++;
      }
      // left-front ROI
      if (inRect(x,y, 0.0, roi_side_x_, 0.0, roi_side_w_)) {
        f.count_left++;
      }
      // right-front ROI
      if (inRect(x,y, 0.0, roi_side_x_, -roi_side_w_, 0.0)) {
        f.count_right++;
      }
    }

    f.block_front = (f.count_front >= min_points_to_block_);
    f.block_left  = (f.count_left  >= min_points_to_block_);
    f.block_right = (f.count_right >= min_points_to_block_);
    return f;
  }

  // ---------------- Pure Pursuit ----------------
  int findNearestIdx(double x, double y) const {
    if (path_xy_.empty()) return -1;
    int best=-1; double best_d2=std::numeric_limits<double>::infinity();
    for (size_t i=0;i<path_xy_.size();++i){
      double dx=path_xy_[i].first-x, dy=path_xy_[i].second-y;
      double d2=dx*dx+dy*dy;
      if (d2<best_d2){ best_d2=d2; best=static_cast<int>(i); }
    }
    return best;
  }

  bool pickForwardPoint(double &lx, double &ly) {
    const int nearest = findNearestIdx(enu_x_, enu_y_);
    if (nearest<0) return false;

    const double c=std::cos(yaw_), s=std::sin(yaw_);
    const int N = static_cast<int>(path_xy_.size());

    for (int i=nearest;i<N;++i){
      double dx=path_xy_[i].first-enu_x_, dy=path_xy_[i].second-enu_y_;
      double x_local =  c*dx + s*dy;
      double y_local = -s*dx + c*dy;
      if (x_local>0.0){
        double d=std::hypot(x_local,y_local);
        if (d>=lfd_){ lx=x_local; ly=y_local; return true; }
      }
    }
    // fallback: 끝점
    double dx=path_xy_.back().first-enu_x_, dy=path_xy_.back().second-enu_y_;
    double x_local =  c*dx + s*dy;
    double y_local = -s*dx + c*dy;
    if (x_local>0.0){ lx=x_local; ly=y_local; return true; }
    return false;
  }

  void doPurePursuitStep() {
    double lx=0.0, ly=0.0;
    if (!pickForwardPoint(lx,ly)) {
      ROS_WARN_THROTTLE(1.0,"[pp] forward point not found");
      publishStop(); return;
    }
    const double theta = std::atan2(ly,lx);
    const double delta = std::atan2(2.0*wheelbase_L*std::sin(theta), lfd_);

    cmd_.steering = delta;
    cmd_.velocity = target_vel_kmh;  // km/h (Morai용)
    cmd_.accel = 0.0; cmd_.brake=0.0;
    cmd_pub_.publish(cmd_);

    ROS_INFO_THROTTLE(0.5,"[pp] PP: yaw=%.1fdeg steer=%.3f v=%.1fkm/h",
                      yaw_*180.0/M_PI, cmd_.steering, cmd_.velocity);
  }

  // ---------------- Avoid ----------------
  void doAvoidStep() {
    // 좌측 회피 = 좌로 조향이 아니라, "좌측 장애물을 피하려면 우로 조향"이니
    // Mode::AVOID_LEFT  -> 왼쪽에 장애물이 많다 -> 오른쪽으로 조향(음수 y가 비니 steer을 음으로)
    // 여기서는 x forward, y left 기준: 우측 회피 = 음의 조향(오른쪽 턴)
    double steer = 0.0;
    if (mode_==Mode::AVOID_LEFT)  steer = -avoid_steer_rad_;
    else if (mode_==Mode::AVOID_RIGHT) steer =  +avoid_steer_rad_;

    cmd_.steering = steer;
    cmd_.velocity = avoid_kmh_;
    cmd_.accel = 0.0; cmd_.brake = 0.0;
    cmd_pub_.publish(cmd_);

    ROS_INFO_THROTTLE(0.5,"[pp] AVOID: steer=%.3f v=%.1fkm/h (clear %d/%d)",
                      cmd_.steering, cmd_.velocity, clear_frames_, clear_frames_needed_);
  }

  void publishStop() {
    cmd_.steering=0.0; cmd_.velocity=0.0; cmd_.accel=0.0; cmd_.brake=0.0;
    cmd_pub_.publish(cmd_);
  }

private:
  ros::NodeHandle nh_;
  // subs/pubs
  ros::Subscriber gps_sub_, imu_sub_, ego_sub_, scan_sub_;
  ros::Publisher  cmd_pub_, path_pub_;

  nav_msgs::Path path_msg_;
  morai_msgs::CtrlCmd cmd_;

  // ENU origin
  GeographicLib::LocalCartesian lc_;
  bool have_origin_;

  // state
  bool have_gps_, have_yaw_;
  double enu_x_{0.0}, enu_y_{0.0};
  double yaw_{0.0};
  double ego_speed_ms_{0.0};

  // path
  std::vector<std::pair<double,double>> path_xy_;
  std::string ref_file_, path_file_, scan_topic_;

  // PP params
  double wheelbase_L;
  double lfd_;
  double target_vel_kmh;

  // Obstacle/ROI params
  double obstacle_dist_thresh_;
  int    min_points_to_block_;
  int    clear_frames_needed_;
  double avoid_steer_rad_;
  double avoid_kmh_;
  double roi_front_x_, roi_front_w_;
  double roi_side_x_,  roi_side_w_;

  // cached scan points (base_link frame 가정)
  std::vector<std::pair<double,double>> cached_points_;
  double last_scan_stamp_;

  enum class Mode { TRACK_WAYPOINT, AVOID_LEFT, AVOID_RIGHT } mode_;
  int clear_frames_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pp_fixed_speed_with_avoid");
  PPFixedSpeedNode node;
  node.spin();
  return 0;
}
