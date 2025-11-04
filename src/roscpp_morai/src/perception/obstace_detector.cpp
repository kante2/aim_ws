#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>
#include <cmath>
#include <vector>
#include <utility>
#include <limits>

class ObstacleDetector {
public:
  ObstacleDetector() : nh_("~") {
    // 파라미터
    nh_.param<std::string>("scan_topic",          scan_topic_,           std::string("/scan"));
    nh_.param<double>("obstacle_dist_thresh",     obstacle_dist_thresh_, 2.5);
    nh_.param<int>("min_points_to_block",         min_points_to_block_,  3); // 진입 임계
    nh_.param<int>("release_points_thresh",       release_points_thresh_,1); // 복귀 임계
    nh_.param<int>("clear_frames_needed",         clear_frames_needed_,  3);
    nh_.param<double>("avoid_timeout_sec",        avoid_timeout_sec_,    2.5);
    nh_.param<double>("roi_front_x",              roi_front_x_,          4.0);
    nh_.param<double>("roi_front_w",              roi_front_w_,          2.0);
    nh_.param<double>("roi_side_x",               roi_side_x_,           3.0);
    nh_.param<double>("roi_side_w",               roi_side_w_,           1.2);
    nh_.param<std::string>("flag_topic",          flag_topic_,           std::string("/avoid_flag"));

    flag_pub_  = nh_.advertise<std_msgs::UInt8>(flag_topic_, 10);
    scan_sub_  = nh_.subscribe(scan_topic_,  20, &ObstacleDetector::scanCB, this);

    mode_ = Mode::TRACK_WAYPOINT;
    clear_frames_ = 0;
    avoid_start_time_ = ros::Time(0);

    cached_points_.reserve(2048);
  }

  void spin() {
    ros::Rate r(20);
    while (ros::ok()) {
      ros::spinOnce();

      // 가장 최근 스캔 기반 평가
      const ObstacleCounts c = evaluate();

      const bool front_block = (c.front >= min_points_to_block_);
      const bool left_block  = (c.left  >= min_points_to_block_);
      const bool right_block = (c.right >= min_points_to_block_);
      const bool front_clear = (c.front <= release_points_thresh_);
      const bool left_clear  = (c.left  <= release_points_thresh_);
      const bool right_clear = (c.right <= release_points_thresh_);

      uint8_t flag_to_pub = 0;

      if (mode_ == Mode::TRACK_WAYPOINT) {
        if (front_block || left_block || right_block) {
          if (front_block) {
            mode_ = Mode::AVOID_RIGHT;  // 상태는 우회전
            first_step_hard_right_ = true; // 첫 스텝 하드
            flag_to_pub = 3; // AVOID_RIGHT_HARD
          } else if (left_block) {
            mode_ = Mode::AVOID_RIGHT;
            first_step_hard_right_ = false;
            flag_to_pub = 2;
          } else { // right_block
            mode_ = Mode::AVOID_LEFT;
            first_step_hard_right_ = false;
            flag_to_pub = 1;
          }
          clear_frames_ = 0;
          avoid_start_time_ = ros::Time::now();
        } else {
          flag_to_pub = 0;
        }
      } else {
        // 회피 중 → 복귀 판정
        if (front_clear && left_clear && right_clear) clear_frames_++;
        else clear_frames_ = 0;

        const bool timeout = (!avoid_start_time_.isZero() &&
          (ros::Time::now() - avoid_start_time_).toSec() >= avoid_timeout_sec_);

        if (clear_frames_ >= clear_frames_needed_ || timeout) {
          mode_ = Mode::TRACK_WAYPOINT;
          first_step_hard_right_ = false;
          flag_to_pub = 0;
        } else {
          if (first_step_hard_right_) flag_to_pub = 3;
          else flag_to_pub = (mode_ == Mode::AVOID_RIGHT) ? 2 : 1;
        }
      }

      // 하드 우회전 플래그는 한 프레임만 내보내고 내부 플래그를 내린다
      if (flag_to_pub == 3) first_step_hard_right_ = false;

      std_msgs::UInt8 msg; msg.data = flag_to_pub;
      flag_pub_.publish(msg);

      ROS_DEBUG_THROTTLE(0.5, "[obst] front=%d left=%d right=%d -> flag=%u",
                         c.front, c.left, c.right, msg.data);

      r.sleep();
    }
  }

private:
  // 데이터
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Publisher  flag_pub_;

  std::string scan_topic_, flag_topic_;

  // ROI/회피 파라미터
  double obstacle_dist_thresh_;
  int    min_points_to_block_;
  int    release_points_thresh_;
  int    clear_frames_needed_;
  double avoid_timeout_sec_;
  double roi_front_x_, roi_front_w_;
  double roi_side_x_,  roi_side_w_;

  // 라이다 포인트 (base_link 기준: x 전방, y 좌측)
  std::vector<std::pair<double,double>> cached_points_;

  enum class Mode { TRACK_WAYPOINT, AVOID_LEFT, AVOID_RIGHT };
  Mode mode_;
  int  clear_frames_;
  ros::Time avoid_start_time_;
  bool first_step_hard_right_{false};

  struct ObstacleCounts { int front{0}, left{0}, right{0}; };

  static inline bool inRect(double x, double y, double x0, double x1, double y0, double y1) {
    return (x >= x0 && x <= x1 && y >= y0 && y <= y1);
  }

  void scanCB(const sensor_msgs::LaserScan::ConstPtr& scan) {
    cached_points_.clear();
    cached_points_.reserve(scan->ranges.size());
    const double a_min = scan->angle_min;
    const double a_inc = scan->angle_increment;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const float r = scan->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r < scan->range_min || r > scan->range_max) continue;
      const double a = a_min + a_inc * static_cast<double>(i);
      const double x = r * std::cos(a);
      const double y = r * std::sin(a);
      cached_points_.emplace_back(x, y);
    }
  }

  ObstacleCounts evaluate() const {
    ObstacleCounts f;
    if (cached_points_.empty()) return f;

    const double half_front_w = roi_front_w_/2.0;
    for (const auto& p : cached_points_) {
      const double x = p.first;
      const double y = p.second;
      const double d = std::hypot(x, y);
      if (d > obstacle_dist_thresh_) continue;

      if (inRect(x, y, 0.0, roi_front_x_, -half_front_w, half_front_w)) f.front++;
      if (inRect(x, y, 0.0, roi_side_x_,  0.0,            roi_side_w_)) f.left++;
      if (inRect(x, y, 0.0, roi_side_x_, -roi_side_w_,    0.0))         f.right++;
    }
    return f;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "obstacle_detector");
  ObstacleDetector node;
  node.spin();
  return 0;
}
