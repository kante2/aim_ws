#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

// ---------------- Params & pubs ----------------
ros::Publisher cmd_pub, debug_wp_pub;

double wheelbase_L, lookahead_;
double free_thresh_;
double roi_front_, roi_half_width_;
double bias_to_heading_;

double cruise_kmh_, avoid_kmh_;
double obstacle_dist_thresh_;

double steer_left_limit_rad_, steer_right_limit_rad_; // 좌/우 최대 조향각 (rad)
bool   steer_invert_ = false; // 필요 시 좌우 반전

double min_gap_width_m_; // ★ 추가: 최소 통과 폭(m)

std::string scan_topic_;
bool publish_debug_wp_;

// --------------- Small helpers ----------------
inline double clampd(double v, double lo, double hi){ return std::max(lo, std::min(v, hi)); }
inline bool isFree(float r){ return r > 0.0f && std::isfinite(r) && r > free_thresh_; }

// ★ 라디안 조향각 → [-1..+1] 정규화 (0=직진, +1=좌, -1=우)
double rad_to_norm(double delta_rad){
  const double L = steer_left_limit_rad_;
  const double R = steer_right_limit_rad_;
  const double lo = std::min(L, R);
  const double hi = std::max(L, R);
  const double d  = clampd(delta_rad, lo, hi);

  const double denom = (L - R);
  const double A = std::fabs(denom) < 1e-6 ? 0.0 : (2.0 / denom);
  const double B = 1.0 - A * L;
  double n = A * d + B; // [-1..+1]
  n = clampd(n, -1.0, 1.0);
  if (steer_invert_) n = -n;
  return n;
}

double estimateGapRange(const std::vector<float>& s, int a, int b){
  int mid=(a+b)/2;
  int L=std::max(a, mid-2), R=std::min(b, mid+2);
  double acc=0; int cnt=0;
  for(int i=L;i<=R;++i){ if(s[i]>0){ acc+=s[i]; ++cnt; } }
  return (cnt>0)? acc/cnt : free_thresh_;
}

void publishDebugWP(double x, double y){
  if(!publish_debug_wp_) return;
  geometry_msgs::PoseStamped p;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = "base_link";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  debug_wp_pub.publish(p);
}

// ------------------- Core ---------------------
void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg){
  // 15hz
  static ros::Time last_pub;
  if (!last_pub.isZero() && (ros::Time::now() - last_pub) < ros::Duration(1.0/15.0))
    return;
  last_pub = ros::Time::now();

  const int N = (int)msg->ranges.size();
  if (N==0) return;

  // 1) copy + clean + light smoothing
  std::vector<float> r = msg->ranges;
  for(int i=0;i<N;++i){
    if(!std::isfinite(r[i])) r[i]=msg->range_max;
    r[i] = (float)clampd(r[i], msg->range_min, msg->range_max);
  }
  std::vector<float> s=r;
  for(int i=1;i<N-1;++i) s[i]=(r[i-1]+r[i]+r[i+1])/3.0f;
  s.front()=r.front(); s.back()=r.back();

  // 2) rectangular ROI in base_link: 0<x<=roi_front, |y|<=roi_half_width
  int roi_valid_cnt = 0;
  for(int i=0;i<N;++i){
    double th = msg->angle_min + i*msg->angle_increment;
    double x = s[i]*std::cos(th);
    double y = s[i]*std::sin(th);
    if(x>0.0 && x<=roi_front_ && std::fabs(y)<=roi_half_width_) { /* keep */ ++roi_valid_cnt; }
    else s[i]=0.0f;
  }
  ROS_DEBUG_THROTTLE(0.5, "[gap_nav] ROI valid beams: %d", roi_valid_cnt);

  // quick occupancy check in ROI (for speed mode)
  bool any_obstacle_near=false;
  for(int i=0;i<N;++i){
    if(s[i] > 0.0f && s[i] < std::min<float>(msg->range_max*0.98f, (float)obstacle_dist_thresh_)){
      any_obstacle_near=true; break;
    }
  }

  // 장애물 감지 상태 전이 로그
  static bool prev_obstacle = false;
  if (any_obstacle_near && !prev_obstacle) {
    ROS_WARN("[gap_nav] obj detected ! (%.2fm near) -> AVOID mode", obstacle_dist_thresh_);
  } else if (!any_obstacle_near && prev_obstacle) {
    ROS_INFO("[gap_nav] obj clear. return to cruise.");
  }
  prev_obstacle = any_obstacle_near;

  // ★ 장애물 없으면: 크루즈 모드(직진 0.0)로 즉시 복귀하고 종료
  if(!any_obstacle_near){
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;     // velocity mode
    cmd.steering     = 0.0;   // 0=직진, +1=좌, -1=우
    cmd.velocity     = cruise_kmh_;
    cmd.accel = 0.0; cmd.brake = 0.0;
    cmd_pub.publish(cmd);

    ROS_INFO_THROTTLE(0.5,
      "[gap_nav] CRUISE steer=0.00 v=%.1f km/h (no obstacle in ROI)", cmd.velocity);
    return;
  }

  // 3) bubble around closest obstacle (inflate by width proxy via angle padding)
  int min_idx=-1; float min_r=std::numeric_limits<float>::infinity();
  for(int i=0;i<N;++i){ if(s[i]>0.0f && s[i]<min_r){ min_r=s[i]; min_idx=i; } }
  if(min_idx>=0 && std::isfinite(min_r)){
    // ★ 버블 완화: cap 12deg, 상수 0.5
    double ang_pad = std::min(12.0*M_PI/180.0, 2.0*std::asin(std::min(1.0, 0.5/(2.0*std::max(0.1,(double)min_r)))));
    int pad = (int)std::ceil(ang_pad / msg->angle_increment);
    for(int k=-pad;k<=pad;++k){
      int j=min_idx+k;
      if(0<=j && j<N) s[j]=0.0f;
    }
  }

  // 4) find gaps (continuous free beams)
  struct Gap{ int a,b; double score; };
  std::vector<Gap> gaps;
  int i=0;
  while(i<N){
    while(i<N && !isFree(s[i])) ++i;
    if(i>=N) break;
    int a=i;
    while(i<N && isFree(s[i])) ++i;
    int b=i-1;
    if(b>=a){
      int mid=(a+b)/2;
      double d = s[mid]>0.0f ? s[mid] : estimateGapRange(s,a,b);

      // ★ 개선된 폭 계산: 양 끝 각도의 tan 차이 이용
      double th_a = msg->angle_min + a*msg->angle_increment;
      double th_b = msg->angle_min + b*msg->angle_increment;
      double gap_width = d * std::fabs(std::tan(th_b) - std::tan(th_a));

      double ang_span = (b-a+1)*msg->angle_increment;
      double ang_center = msg->angle_min + ((a+b)/2.0)*msg->angle_increment;
      double heading_bias = (1.0 - bias_to_heading_ * std::min(1.0, std::fabs(ang_center)/(M_PI/3.0)));
      double score = ang_span * heading_bias * (0.5 + 0.5*(d/std::max(0.1,lookahead_)));

      ROS_DEBUG_THROTTLE(0.3, "[gap_nav] gap a=%d b=%d span=%.1fdeg d=%.2f w=%.2f",
                         a,b, ang_span*180.0/M_PI, d, gap_width);

      if(gap_width >= min_gap_width_m_){
        gaps.push_back({a,b,score});
      }
    }
  }

  double th = 0.0;
  double d  = lookahead_;
  if(gaps.empty()){
    // usable gap이 없으면 안전 정지
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 2;
    cmd.steering     = 0.0;   // 정면 유지(중립)
    cmd.velocity     = 1.0;
    cmd.accel = 0.0; cmd.brake = 0.0;
    cmd_pub.publish(cmd);
    ROS_WARN_THROTTLE(0.5, "[gap_nav] no usable gap -> STOP/ SLOW TUNED V=1.0 km/h");
    return;
  } else {
    auto best_gap = *std::max_element(gaps.begin(), gaps.end(),
                        [](const Gap& x, const Gap& y){return x.score<y.score;});
    int mid=(best_gap.a+best_gap.b)/2;
    th = msg->angle_min + mid*msg->angle_increment;
    double dm = (mid>=0 && mid<N && s[mid]>0.0f)? s[mid] : lookahead_;
    d = std::min<double>(dm, lookahead_);
  }

  // 5) convert target (d, th) to steering
  double x = d*std::cos(th);
  double y = d*std::sin(th);
  double delta = std::atan2( 2.0 * wheelbase_L * std::sin(th), lookahead_ ); // rad

  // 라디안 → [-1..+1] 정규화 (0=직진, +1=좌, -1=우)
  double steer_norm = rad_to_norm(delta);

  // (optional) 디버그 웨이포인트
  publishDebugWP(x, y);

  // 6) publish MORAI CtrlCmd
  morai_msgs::CtrlCmd cmd;
  cmd.longlCmdType = 2;            // velocity control
  cmd.steering     = steer_norm;   // -1..+1
  cmd.velocity     = avoid_kmh_;   // 회피 중 감속 속도
  cmd.accel = 0.0; cmd.brake = 0.0;

  cmd_pub.publish(cmd);

  ROS_INFO_THROTTLE(0.5,
    "[gap_nav] AVOID th=%.1fdeg delta=%.3frad steer=%.2f v=%.1f km/h (x=%.2f,y=%.2f)",
    th*180.0/M_PI, delta, steer_norm, cmd.velocity, x, y);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gap_navigator_morai_ctrl");
  ros::NodeHandle nh("~");

  // ---- params ----
  nh.param("scan_topic",        scan_topic_,        std::string("/scan"));
  nh.param("wheelbase",         wheelbase_L,        3.0);     // m
  nh.param("lookahead",         lookahead_,         4.0);     // m
  nh.param("free_thresh",       free_thresh_,       0.80);    // m
  nh.param("roi_front",         roi_front_,         6.0);     // m
  nh.param("roi_half_width",    roi_half_width_,    1.0);     // m
  nh.param("bias_to_heading",   bias_to_heading_,   0.3);     // 0~1
  nh.param("cruise_kmh",        cruise_kmh_,        10.0);     // km/h
  nh.param("avoid_kmh",         avoid_kmh_,         3.0);     // km/h
  nh.param("obstacle_dist_thresh", obstacle_dist_thresh_, 2.5); // m

  // 좌/우 최대 조향각(rad). 기본 ±30deg
  nh.param("steer_left_limit_rad",  steer_left_limit_rad_,  -0.523599); // -30deg
  nh.param("steer_right_limit_rad", steer_right_limit_rad_,  0.523599); // +30deg
  nh.param("steer_invert",          steer_invert_,           false);     // 필요 시 좌우 반전

  // ★ 최소 통과 폭(m)
  nh.param("min_gap_width_m",       min_gap_width_m_,        0.6);

  nh.param("publish_debug_wp",  publish_debug_wp_,  true);

  cmd_pub      = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);
  if(publish_debug_wp_) debug_wp_pub = nh.advertise<geometry_msgs::PoseStamped>("/avoid_waypoint", 1);
  ros::Subscriber sub = nh.subscribe(scan_topic_, 1, &scanCb);

  ros::Rate rate(15);               // ★ 15 Hz
  while (ros::ok()) {
    ros::spinOnce();                // 콜백 처리
    rate.sleep();                   // 주기 맞춤
  }
  return 0;
}

