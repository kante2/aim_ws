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

// ===================== 전역 상태 =====================
ros::Publisher g_cmd_pub, g_path_pub;
morai_msgs::CtrlCmd g_cmd;

GeographicLib::LocalCartesian g_lc(GeographicLib::Geocentric::WGS84());
bool g_have_origin=false, g_have_gps=false, g_have_yaw=false;

double g_enu_x=0.0, g_enu_y=0.0, g_yaw=0.0;   // yaw[rad]
double g_ego_speed_ms=0.0;

std::vector<std::pair<double,double>> g_path_xy;
std::vector<std::pair<double,double>> g_lidar_pts_bl; // base_link 좌표계 (x 전방, y 좌측)

// 파일 경로
std::string g_ref_file, g_path_file;
std::string g_scan_topic;

// 제어 파라미터
double g_wheelbase_L=3.0;   // [m]
double g_lfd=4.5;           // [m]
double g_target_kmh=15.0;   // [km/h]

// 회피 파라미터
double g_obstacle_dist_thresh=2.5;  // [m]
int    g_min_points_to_block=3;     // AVOID 진입 임계
int    g_release_points_thresh=1;   // AVOID 해제 임계
int    g_clear_frames_needed=3;     // 클리어 프레임수
double g_avoid_timeout_sec=2.5;     // 타임아웃
double g_avoid_kmh=8.0;             // 회피 속도[km/h]
double g_roi_front_x=4.0, g_roi_front_w=2.0; // 전방 직사각형 (0~x, |y|<=w/2)
double g_roi_side_x=3.0,  g_roi_side_w=1.2;  // 좌/우 전방 얇은 직사각형

// 모드/상태
enum Mode { TRACK_WAYPOINT, AVOID_LEFT, AVOID_RIGHT };
Mode g_mode = TRACK_WAYPOINT;
int g_clear_frames = 0;
ros::Time g_avoid_start_time(0);
bool g_first_step_hard_right = false;

// ===================== 유틸/로더 =====================
bool loadOrigin(const std::string& file){
  std::ifstream in(file);
  if(!in.is_open()) return false;
  double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
  g_lc.Reset(lat0, lon0, alt0);
  g_have_origin = true;
  ROS_INFO("[pp_fixed] ENU origin: lat=%.15f lon=%.15f alt=%.3f", lat0, lon0, alt0);
  return true;
}

bool loadPath(const std::string& file){
  std::ifstream in(file);
  if(!in.is_open()) return false;
  g_path_xy.clear();
  std::string line; double x,y,z;
  while(std::getline(in,line)){
    if(line.empty()) continue;
    std::istringstream iss(line);
    if(!(iss>>x>>y>>z)) continue;
    g_path_xy.emplace_back(x,y);
  }
  ROS_INFO("[pp_fixed] Path points loaded: %zu", g_path_xy.size());
  return !g_path_xy.empty();
}

void publishPathOnce(){
  nav_msgs::Path p;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = "map";
  for(auto& xy: g_path_xy){
    geometry_msgs::PoseStamped ps; ps.header = p.header;
    ps.pose.position.x = xy.first;
    ps.pose.position.y = xy.second;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  g_path_pub.publish(p);
}

int findNearestIdx(double x, double y){
  if(g_path_xy.empty()) return -1;
  int best=-1; double best_d2=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<g_path_xy.size();++i){
    double dx=g_path_xy[i].first-x, dy=g_path_xy[i].second-y;
    double d2=dx*dx+dy*dy;
    if(d2<best_d2){best_d2=d2; best=(int)i;}
  }
  return best;
}

void publishStop(){
  g_cmd.steering = 0.0; // (정지시 직진이든 무관)
  g_cmd.velocity = 0.0;
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;
  g_cmd_pub.publish(g_cmd);
}

// ===================== 콜백(상태만 갱신) =====================
void gpsCB(const morai_msgs::GPSMessage& msg){
  if(!g_have_origin) return;
  double x,y,z;
  try{
    g_lc.Forward(msg.latitude, msg.longitude,
                 std::isfinite(msg.altitude)? msg.altitude:0.0,
                 x,y,z);
  }catch(...){
    ROS_WARN_THROTTLE(1.0, "[pp_fixed] LocalCartesian.Forward failed");
    return;
  }
  g_enu_x=x; g_enu_y=y; g_have_gps=true;
}

void imuCB(const sensor_msgs::Imu& msg){
  const auto& q = msg.orientation;
  tf::Quaternion tfq(q.x,q.y,q.z,q.w);
  g_yaw = tf::getYaw(tfq);
  g_have_yaw = true;
}

void egoCB(const morai_msgs::EgoVehicleStatus& msg){
  g_ego_speed_ms = msg.velocity.x;
}

void lidarCB(const sensor_msgs::LaserScan::ConstPtr& scan){
  g_lidar_pts_bl.clear();
  g_lidar_pts_bl.reserve(scan->ranges.size());
  const double a0 = scan->angle_min, di = scan->angle_increment;
  for(size_t i=0;i<scan->ranges.size();++i){
    float r = scan->ranges[i];
    if(!std::isfinite(r)) continue;
    if(r < scan->range_min || r > scan->range_max) continue;
    double a = a0 + di*(double)i;
    double x = r*std::cos(a), y=r*std::sin(a); // base_link
    g_lidar_pts_bl.emplace_back(x,y);
  }
}

// ===================== ROI/회피 평가 =====================
struct ObstacleCounts{ int front{0}, left{0}, right{0}; };

inline bool inRect(double x,double y,double x0,double x1,double y0,double y1){
  return (x>=x0 && x<=x1 && y>=y0 && y<=y1);
}

ObstacleCounts evaluateObstacles(){
  ObstacleCounts c;
  if(g_lidar_pts_bl.empty()) return c;

  const double half_front_w = g_roi_front_w/2.0;

  for(const auto& pt: g_lidar_pts_bl){
    double x=pt.first, y=pt.second;
    double d=hypot(x,y);
    if(d>g_obstacle_dist_thresh) continue;

    // front box
    if(inRect(x,y, 0.0, g_roi_front_x, -half_front_w, +half_front_w)) c.front++;
    // left slender
    if(inRect(x,y, 0.0, g_roi_side_x,  0.0,            g_roi_side_w)) c.left++;
    // right slender
    if(inRect(x,y, 0.0, g_roi_side_x, -g_roi_side_w,   0.0))          c.right++;
  }
  return c;
}

// ===================== 메인 =====================
int main(int argc, char** argv){
  ros::init(argc, argv, "pp_fixed_speed_loop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 파라미터
  pnh.param<std::string>("ref_file",  g_ref_file,  std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
  pnh.param<std::string>("path_file", g_path_file, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
  pnh.param<std::string>("scan_topic", g_scan_topic, std::string("/scan"));

  pnh.param("wheelbase", g_wheelbase_L, 3.0);
  pnh.param("lookahead", g_lfd, 4.5);
  pnh.param("target_kmh", g_target_kmh, 15.0);

  pnh.param("obstacle_dist_thresh", g_obstacle_dist_thresh, 2.5);
  pnh.param("min_points_to_block",  g_min_points_to_block,  3);
  pnh.param("release_points_thresh",g_release_points_thresh,1);
  pnh.param("clear_frames_needed",  g_clear_frames_needed,  3);
  pnh.param("avoid_timeout_sec",    g_avoid_timeout_sec,    2.5);
  pnh.param("avoid_kmh",            g_avoid_kmh,            8.0);
  pnh.param("roi_front_x",          g_roi_front_x,          4.0);
  pnh.param("roi_front_w",          g_roi_front_w,          2.0);
  pnh.param("roi_side_x",           g_roi_side_x,           3.0);
  pnh.param("roi_side_w",           g_roi_side_w,           1.2);

  // 파일 로드
  if(!loadOrigin(g_ref_file)){
    ROS_FATAL("[pp_fixed] Failed to load ENU origin from %s", g_ref_file.c_str());
    return 1;
  }
  if(!loadPath(g_path_file)){
    ROS_FATAL("[pp_fixed] Failed to load path from %s", g_path_file.c_str());
    return 1;
  }

  // Pub/Sub
  g_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1, true);
  publishPathOnce();

  ros::Subscriber gps_sub  = nh.subscribe("/gps",       20, gpsCB);
  ros::Subscriber imu_sub  = nh.subscribe("/imu",       50, imuCB);
  ros::Subscriber ego_sub  = nh.subscribe("/Ego_topic", 20, egoCB);
  ros::Subscriber lid_sub  = nh.subscribe(g_scan_topic, 20, lidarCB);

  g_cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

  // 제어 모드: velocity
  g_cmd.longlCmdType = 2;
  g_cmd.accel = 0.0; g_cmd.brake = 0.0;

  ros::Rate rate(15);
  while(ros::ok()){
    ros::spinOnce();

    if(!g_have_gps){
      ROS_INFO_THROTTLE(1.0, "[pp_fixed] waiting /gps ...");
      rate.sleep(); continue;
    }
    if(!g_have_yaw){
      ROS_INFO_THROTTLE(1.0, "[pp_fixed] waiting /imu ...");
      rate.sleep(); continue;
    }

    // ---------- 라이다 기반 회피 상태머신 ----------
    ObstacleCounts c = evaluateObstacles();

    bool front_block = (c.front >= g_min_points_to_block);
    bool left_block  = (c.left  >= g_min_points_to_block);
    bool right_block = (c.right >= g_min_points_to_block);

    bool front_clear = (c.front <= g_release_points_thresh);
    bool left_clear  = (c.left  <= g_release_points_thresh);
    bool right_clear = (c.right <= g_release_points_thresh);

    if(g_mode == TRACK_WAYPOINT){
      if(front_block || left_block || right_block){
        if(front_block){ g_mode = AVOID_RIGHT; g_first_step_hard_right = true; }
        else if(left_block){ g_mode = AVOID_RIGHT; g_first_step_hard_right = false; }
        else { g_mode = AVOID_LEFT; g_first_step_hard_right = false; }
        g_clear_frames = 0;
        g_avoid_start_time = ros::Time::now();
        ROS_WARN("[pp_fixed] OBJ detected -> AVOID mode (F:%d L:%d R:%d)", c.front, c.left, c.right);
      }
    }else{ // AVOID 중
      if(front_clear && left_clear && right_clear) g_clear_frames++;
      else g_clear_frames = 0;

      bool timeout = (!g_avoid_start_time.isZero() &&
                      (ros::Time::now() - g_avoid_start_time).toSec() >= g_avoid_timeout_sec);
      if(g_clear_frames >= g_clear_frames_needed || timeout){
        g_mode = TRACK_WAYPOINT;
        ROS_INFO("[pp_fixed] OBJ clear -> TRACK mode");
      }
    }

    // ---------- AVOID 동작 ----------
    if(g_mode == AVOID_LEFT || g_mode == AVOID_RIGHT){
      // 스티어는 0~1 스케일 가정 (좌=0, 직진=0.5, 우=1)
      if(g_first_step_hard_right){
        g_cmd.steering = 1.0;             // 전방 막힘 첫 스텝 강한 우회전
        g_first_step_hard_right = false;
      }else{
        g_cmd.steering = (g_mode==AVOID_RIGHT)? 0.8 : 0.2;
      }
      g_cmd.velocity = g_avoid_kmh;
      g_cmd.accel=0.0; g_cmd.brake=0.0;
      g_cmd_pub.publish(g_cmd);

      int near_idx = findNearestIdx(g_enu_x, g_enu_y);
      ROS_INFO_THROTTLE(0.5,
        "[pp_fixed][AVOID] ENU(%.2f,%.2f) yaw=%.1fdeg steer=%.2f v=%.1f(km/h) near=%d  (F:%d L:%d R:%d)",
        g_enu_x, g_enu_y, g_yaw*180.0/M_PI, g_cmd.steering, g_cmd.velocity, near_idx, c.front, c.left, c.right);
      rate.sleep();
      continue; // 회피 시 PP 스킵
    }

    // ---------- TRACK(Pure Pursuit) ----------
    int nearest_idx = findNearestIdx(g_enu_x, g_enu_y);
    if(nearest_idx<0){
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] nearest index not found");
      publishStop(); rate.sleep(); continue;
    }

    // 전방(+x)에서 lookahead 이상 첫 점
    double lx=0.0, ly=0.0; bool found=false;
    {
      const double cYaw = std::cos(g_yaw), sYaw = std::sin(g_yaw);
      for(int i=nearest_idx; i<(int)g_path_xy.size(); ++i){
        double dx=g_path_xy[i].first - g_enu_x;
        double dy=g_path_xy[i].second- g_enu_y;
        double x_local =  cYaw*dx + sYaw*dy; // 전방(+)
        double y_local = -sYaw*dx + cYaw*dy; // 좌(+)
        if(x_local>0.0){
          double d=hypot(x_local,y_local);
          if(d>=g_lfd){ lx=x_local; ly=y_local; found=true; break; }
        }
      }
      if(!found){
        double dx=g_path_xy.back().first - g_enu_x;
        double dy=g_path_xy.back().second- g_enu_y;
        lx =  cYaw*dx + sYaw*dy;
        ly = -sYaw*dx + cYaw*dy;
        found = (lx>0.0);
      }
    }

    if(!found){
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] forward point not found (lfd=%.1f)", g_lfd);
      publishStop(); rate.sleep(); continue;
    }

    // PP 조향 (라디안)
    double theta = std::atan2(ly, lx);
    double delta = std::atan2( 2.0 * g_wheelbase_L * std::sin(theta), g_lfd );
    g_cmd.steering = delta;           // ★ PP는 라디안 값 그대로
    g_cmd.velocity = g_target_kmh;    // ★ km/h 그대로 사용(환경에 따라 m/s면 변환 필요)

    g_cmd.accel=0.0; g_cmd.brake=0.0;
    g_cmd_pub.publish(g_cmd);

    ROS_INFO_THROTTLE(0.5,
      "[pp_fixed][TRACK] ENU(%.2f,%.2f) yaw=%.1fdeg steer(rad)=%.3f v=%.1f(km/h) near=%d",
      g_enu_x, g_enu_y, g_yaw*180.0/M_PI, g_cmd.steering, g_cmd.velocity, nearest_idx);

    rate.sleep();
  }
  return 0;
}
