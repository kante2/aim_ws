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
class PPFixedSpeedNode { // -> 구조체  / hpp, cpp 분류 
public:
  PPFixedSpeedNode()
  : nh_(""),lc_(GeographicLib::Geocentric::WGS84()), have_origin_(false), have_gps_(false), have_yaw_(false) {
    // nh은 노드헨들러, ros통신 담당 인터페이스
    // lc_ gps좌표를 로봇이 사용하는 평면좌표 (ENU) 로 변환하는 객체 : lc_
    // lc_가 원점의 위도 경도를 알면, 이후 모든 gps좌표는 , 이원점을 기준으로 enu평면 좌표로 변횐된다. 
    
    // ---- 파라미터 ----
    nh_.param<std::string>("ref_file", ref_file_, std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
    nh_.param<std::string>("path_file", path_file_, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
    nh_.param<double>("wheelbase", wheelbase_L, 3.0);         // 차량 휠베이스 [m]
    nh_.param<double>("lookahead", lfd_, 4.5);      // look-ahead 거리 [m] 4.5로 튜닝함. 
    nh_.param<double>("target_kmh", target_vel_kmh, 15.0); // 15km/h (고정)

    // ENU 원점/경로 로드
    if (!loadOrigin(ref_file_)) {
      ROS_FATAL("[pp_fixed] Failed to load ENU origin from %s", ref_file_.c_str());
      ros::shutdown();
    }
    if (!loadPath(path_file_)) {
      ROS_FATAL("[pp_fixed] Failed to load path from %s", path_file_.c_str());
      ros::shutdown();
    }

    // PUB, SUB
    path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    publishPath();

    gps_sub_ = nh_.subscribe("/gps", 20, &PPFixedSpeedNode::gpsCB, this);
    imu_sub_ = nh_.subscribe("/imu", 50, &PPFixedSpeedNode::imuCB, this);
    ego_sub_ = nh_.subscribe("/Ego_topic", 20, &PPFixedSpeedNode::egoCB, this);

    cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

    // 제어 모드: velocity 제어 -> 2: velocity
    cmd_.longlCmdType = 2;
    cmd_.accel  = 0.0;
    cmd_.brake  = 0.0;
  }

  void spin() {
    ros::Rate rate(15);
    while (ros::ok()) {
      ros::spinOnce(); // 그 순간 콜백 큐에 쌓여 있던 모든 콜백을 처리한다.

      // 0) GPS/IMU 대기
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
      if (nearest_idx < 0) { // -1이 반환되면 경로가 비어있다는 뜻
        ROS_WARN_THROTTLE(1.0, "[pp_fixed] nearest index not found"); // 로그를 일정 주기로만 찍는 경고 매크로 - 1.0Hz
        publishStop();
        rate.sleep();
        continue;
      }

      // 2) 전방(+x)에서 lookahead 이상 첫 점 선택
      double lx = 0.0, ly = 0.0; // 목표점의 차량 로컬 좌표
      bool found = false; // 전방 주시거리를 만족하는 목표점을 찾았는지 플래그 
      {
        const double cos = std::cos(yaw_), sin = std::sin(yaw_); // 차량의 yaw값으로 방향벡터 계산 // imuCB -> yaw_
        // 문법 :: path_xy_.size() : path_xy_ 벡터의 크기 (원소 개수) -> size_t 타입 반환
        // 이를, static_cast<int> : size_t 타입을 int 타입으로 명시적으로 변환
        const int path_size = static_cast<int>(path_xy_.size());
        for (int i = nearest_idx; i < path_size; ++i)
        { // 최근접 인덱스부터 반복한다. 
          const double dx = path_xy_[i].first  - enu_x_; // enu_x_ : 현재 차량의 ENU x좌표 <- gpsCB
          const double dy = path_xy_[i].second - enu_y_; // enu_y_ : 현재 차량의 ENU y좌표 <- gpsCB
          // dx, dy 를 차량 바디 프레임으로 변환
          // 바디 프레임: x축이 전방, y축이 좌측,
          // dx, dy는 ENU좌표계에서의 좌표차이
          const double x_local =  cos*dx + sin*dy;  // 전방(+)
          const double y_local = -sin*dx + cos*dy;  // 좌(+)
          // lookahead 거리 이상이면서, 전방(+)에 있는 첫 점 선택
          if (x_local > 0.0) {
            const double d = std::hypot(x_local, y_local);
            if (d >= lfd_) { lx = x_local; ly = y_local; found = true; break; }
          }
        }
        // 못 찾으면: 경로 끝점(.back)으로 폴백 (단, 앞쪽일 때만)
        if (!found) {
          const double dx = path_xy_.back().first  - enu_x_;
          const double dy = path_xy_.back().second - enu_y_;
          lx =  cos*dx + sin*dy;  ly = -sin*dx + cos*dy;
          found = (lx > 0.0);
        }
      }

      if (!found) {
        // txt의 가장 마지막에 도달시, found가 false가 되어 여기로 옴.
        ROS_WARN_THROTTLE(1.0, "[pp_fixed] forward point not found (lfd=%.1f)", lfd_);
        publishStop();
        rate.sleep();
        continue;
      }

      // 3) Pure Pursuit 조향
      const double theta = std::atan2(ly, lx);                // 바디 프레임에서 목표점 각도
      const double delta = std::atan2(2.0 * wheelbase_L * std::sin(theta), lfd_);
      cmd_.steering = delta;                                  // [rad]

      // 4) 속도: 5 km/h 고정 (파라미터로 바꾸고 싶으면 target_kmh 수정)
      // const double target_ms = target_vel_kmh_ / 3.6;         // 5 km/h => 1.388...
      // cmd_.velocity = target_ms;
      cmd_.velocity = target_vel_kmh; //임시 높은 속도
      cmd_.accel = 0.0;
      cmd_.brake = 0.0;

      cmd_pub_.publish(cmd_);

      ROS_INFO_THROTTLE(0.5,
        "[pp_fixed] ENU(%.2f, %.2f) yaw=%.1fdeg steer=%.3frad v_set=%.2fm/s (near=%d)",
        enu_x_, enu_y_, yaw_*180.0/M_PI, cmd_.steering, cmd_.velocity, nearest_idx);

      rate.sleep();
    }
  }

private:
// private - 1 :: CB, load, publish 등 멤버 함수 선언
  // 파일 로드 -----
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
    std::string line; 
    double x, y, z;
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
    // TF을 통해 여러 좌표계간 변환을 관리 - 이 변환은 시간에 따라 달라지기 때문에, 어느시점의 데이터인지 알려주는 타임스탬프가 필요
    p.header.frame_id = "map";
    // p.header.frame_id = "base_link";
    for (auto &pt : path_xy_) { //auto -자동 타입추론, path_xy요소를 &pt로 참조,
      geometry_msgs::PoseStamped ps; 
      ps.header = p.header;

      ps.pose.position.x = pt.first;
      ps.pose.position.y = pt.second;
      ps.pose.orientation.w = 1.0;
      p.poses.push_back(ps); // p 의 poses리스트에 ps을 추가
    }
    path_pub_.publish(p);
    path_msg_ = p;
  }

  // ---------- Callbacks ----------
  // GPS 콜백: 위도, 경도, 고도를 ENU 좌표로 변환
  void gpsCB(const morai_msgs::GPSMessage &msg) {
    if (!have_origin_) return; // 원점이 없다면 GPS-> ENU 변환 불가-> 종료
    double x, y, z; // 변환된 ENU좌표 저장할 공간 선언
    try {
      // lc_.Reset(lat0, lon0, alt0); -> 원점을 설정하여 ENU 좌표계의 기준을 바꿈. 
      //   설정한 lat0, lon0, alt0이 원점으로 심는 ENU가 된다. 
      // lc_객체는 GeographicLib::LocalCartesian으로, gps좌표를 enu좌표로 변환하는 기능을 제공
      lc_.Forward(msg.latitude, msg.longitude,
                  std::isfinite(msg.altitude) ? msg.altitude : 0.0, //<- altitude가 유한하지 않으면 0.0 사용
                  x, y, z); // 이후 lc_.Forward(lat,lon,h, x,y,z)가 내놓는 (x=East, y=North, z=Up)은 지금 설정한 (lat0,lon0,alt0)을 (0,0,0)으로 삼는 ENU
    } 
    catch (...) { // try문에서 발생한 예외를 처리하는 코드이다. '...'은 모든 종류의 예외를 포착한다는 의미
      ROS_WARN_THROTTLE(1.0, "[pp_fixed] LocalCartesian.Forward failed");
      return;
    }
    enu_x_ = x;
    enu_y_ = y;
    have_gps_ = true;
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
    int best_idx = -1; 
    // 저장할 거리를, 무한대로 초기화 
    double best_d2 = std::numeric_limits<double>::infinity();
    // 모든 점을 for문으로 순회
    for (size_t i = 0; i < path_xy_.size(); ++i) {
      const double dx = path_xy_[i].first  - x;
      const double dy = path_xy_[i].second - y;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; best_idx = static_cast<int>(i); } // 거리를 업데이트
    }
    return best_idx; // 가장 가까운 점의 인덱스 반환
  }

  // 퍼블리시를 종료해야 하는 상황에서 호출
  void publishStop() {
    cmd_.steering = 0.0;
    cmd_.velocity = 0.0;  // velocity 제어 모드에서 정지
    cmd_.accel = 0.0;
    cmd_.brake = 0.0;
    cmd_pub_.publish(cmd_);
  }

private:
// private - 2 :: 내부 통신 자원, 파라미터를 선언 
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
  bool have_gps_, have_yaw_;
  double enu_x_{0.0}, enu_y_{0.0};
  double yaw_{0.0};            // rad (from IMU)

  double ego_speed_ms_{0.0};   // from /Ego_topic (로그)

  // path (ENU)
  // path_xy에, 차량이 따라가야 하는 (double x, double y) 좌표들이 저장된다.
  // vector을 사용하여 객체를 만들면, 리스트와 유사하게 사용할 수 있다. 
  // 메모리에 원소들이 연속적으로 붙어있어, 특정 인덱스에 빠른 접근이 가능함. ** 
  // path_xy_.emplace_back(x, y); --> 가장 뒤에 새로운 (x, y) 쌍을 추가
  // path_xy.push_back(std::make_pair(x, y)); --> 동일한 기능
  std::vector<std::pair<double,double>> path_xy_;
  std::string ref_file_, path_file_;

  // control params
  double wheelbase_L;  // wheelbase [m]
  double lfd_; // lookahead [m]
  double target_vel_kmh; // 기본 15km/h
};

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "pp_fixed_speed");
  PPFixedSpeedNode node;
  node.spin();
  return 0;
}
