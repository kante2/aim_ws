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

// 전역,, 
GeographicLib::LocalCartesian g_lc(GeographicLib::Geocentric::WGS84());
ros::Publisher cmd_pub;

std::vector<std::pair<double,double>> g_path_xy;

double g_enu_x = 0.0, g_enu_y = 0.0;
double imu_yaw  = 0.0;   // [rad]

std::string g_ref_file, g_path_file;

double wheelbase_L = 3.0;   // [m]
double lfd         = 4.5;   // [m]
double target_kmh  = 15.0;  // [km/h]

ros::Publisher g_cmd_pub;
morai_msgs::CtrlCmd g_cmd;

inline void publishStop(){
  g_cmd.steering = 0.0;
  g_cmd.velocity = 0.0;
  g_cmd.accel = 0.0;
  g_cmd.brake = 0.0;
  g_cmd_pub.publish(g_cmd);
}

// -------------------- 유틸/로더 --------------------
bool loadOrigin(const std::string &file) {
  std::ifstream in(file);
  if (!in.is_open()) return false;
  double lat0, lon0, alt0; in >> lat0 >> lon0 >> alt0;
  g_lc.Reset(lat0, lon0, alt0);
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

// CB -------------------------------------
// gpsCB -> // void는, 반환값이 없음을 의미
void gpsCB(const morai_msgs::GPSMessage& msg)
{
    double x,y,z;
    try{
        g_lc.Forward(msg.latitude, msg.longitude,
                     std::isfinite(msg.altitude)? msg.altitude:0.0,
                     x,y,z);
    }catch(...){
        ROS_WARN_THROTTLE(1.0, "[stanley_fixed] LocalCartesian.Forward failed");
        return;
    }
    // g_enu_x, g_enu_y 전역 변수에 변환된 ENU 좌표 저장
    g_enu_x = x;
    g_enu_y = y;
}
// imuCB
void imuCB(const sensor_msgs::Imu& msg)
{
    const auto& q = msg.orientation;
    tf::Quaternion tfq(q.x,q.y,q.z,q.w);
    imu_yaw = tf::getYaw(tfq);
}

// functions------------------------------------

// }
int findNearestIdx(double x, double y)
{
    if (g_path_xy.empty()) return -1;
    int best_idx = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    const int path_size = static_cast<int>(g_path_xy.size());

    for (int i = 0; i < path_size; ++i){
        const double dx = g_path_xy[i].first  - x;
        const double dy = g_path_xy[i].second - y;
        const double distance_tmp = dx*dx + dy*dy;
        if (distance_tmp < best_distance){
            best_distance = distance_tmp;   // ← 오타 수정
            best_idx = i;
        }
    }
    return best_idx;
}

// function2 _ Tangent, Normal을 찾는 함수

struct TN {
  double path_angle; // 접선각
  double nx, ny;  // 좌측 법선 단위벡터
};

TN computeTN(int point_i)
{
    TN out{0.0, 0.0, 0.0};
    const int path_size = static_cast<int>(g_path_xy.size());
    int point_0 = std::max(0, std::min(point_i,         path_size-1));
    int point_1 = std::max(0, std::min(point_i + 1,     path_size-1));  // ← 오타 수정
    if (point_0 == point_1 && point_0 > 0){
        point_0 = point_1 - 1; // 마지막 점 예외 처리
    }

    const double tx  = g_path_xy[point_1].first  - g_path_xy[point_0].first;
    const double ty  = g_path_xy[point_1].second - g_path_xy[point_0].second;
    const double len = std::hypot(tx, ty) + 1e-12; // 0 나눗셈 방지
    const double tnx = tx/len, tny = ty/len;       // 접선 단위벡터
    const double nx  = -tny;                       // 좌측 법선
    const double ny  =  tnx;

    out.path_angle = M_PI/2 - std::atan2(tny, tnx); // 접선 단위벡터,, 로 각도 계산
    out.nx = nx;  out.ny = ny;
    return out;
}

// function3_ wrapAngle
// 헤딩오차는 보통 “–π ~ +π” 범위로 쓰지 않으면 점프가 생김.
// 예: (179°) − (−179°) = 358° → 사실상 +2°여야 함.
// 이 함수는 임의의 라디안 각 a를 –π ~ +π 범위로 접어서 반환.
inline double wrapAngle(double a){
  while(a >  M_PI) a -= 2*M_PI;
  while(a < -M_PI) a += 2*M_PI;
  return a;
}

// main ---------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // 콜백들이 쓰는 전역들을 전역으로 선언해야함,, 
    
    // init ROS node
    ros::init(argc, argv, "stanley_fixed_speed");
    ros::NodeHandle nh;  // global node handle
    ros::NodeHandle pnh("~"); // private node handle  ← Pure Pursuit와 동일하게 사용

    // Pure Pursuit과 동일한 파라미터 키로부터 읽기
    pnh.param<std::string>("ref_file", g_ref_file, std::string("/root/ws/src/roscpp_morai/map/ref.txt"));
    pnh.param<std::string>("path_file", g_path_file, std::string("/root/ws/src/roscpp_morai/map/Path.txt"));
    pnh.param<double>("wheelbase", wheelbase_L, 3.0);
    pnh.param<double>("lookahead", lfd, 4.5);
    pnh.param<double>("target_kmh", target_kmh, 15.0);    

    // ★ Pure Pursuit과 동일: 시작 시 ENU 원점/경로 로드, 실패 시 종료
    if (!loadOrigin(g_ref_file)) {
      ROS_FATAL("[pp_fixed] Failed to load ENU origin from %s", g_ref_file.c_str());
      return 1;
    }
    if (!loadPath(g_path_file)) {
      ROS_FATAL("[pp_fixed] Failed to load path from %s", g_path_file.c_str());
      return 1;
    }

    // 1. subscribers
    ros::Subscriber gps_sub = nh.subscribe("/gps", 20, gpsCB);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 50, imuCB);
    // ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 20, egoCB);
    // ros::Subscriber lidar_sub = nh.subscribe(g_scan_topic, 20, lidarCB);

    // 2. publishers
    // g_cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 2);

    // 제어모드 설정
    g_cmd.longlCmdType = 2; // 2: velocity
    
    // main loop --------------------------------------------------------
    // 15 hz
    ros::Rate rate(15);
    while (ros::ok()){
        ros::spinOnce();

        // 3-0 후륜 기준 -> 전륜의 위치를 정의 
        // g_enu_x, g_enu_y 는 후륜 기준 좌표(txt파일이 후륜 기준으로 작성되었다고 가정하였음)
        const double front_enu_x = g_enu_x + wheelbase_L * std::cos(imu_yaw);
        const double front_enu_y = g_enu_y + wheelbase_L * std::sin(imu_yaw);

        // find nearest index
        int nearest_idx = findNearestIdx(front_enu_x, front_enu_y);
        if (nearest_idx < 0) {
            ROS_WARN_THROTTLE(1.0, "[stanley_fixed] nearest index not found");
            publishStop();
            rate.sleep(); continue;
        }

        // 경로 접선각, 법선 단위벡터
        TN tn = computeTN(nearest_idx);

        // <1>   헤딩오차 = 차 yaw - 경로 접선각
        // double heading_error = wrapAngle(imu_yaw - tn.path_angle);
        double heading_error = wrapAngle(tn.path_angle - imu_yaw);  // MORAI 좌/우 기준과 부호 보정

        // <2> 횡방향 오차 = (차 위치 - 최근접 경로점) · 법선 단위벡터
        double dx = front_enu_x - g_path_xy[nearest_idx].first;
        double dy = front_enu_y - g_path_xy[nearest_idx].second;
        const double c = std::cos(imu_yaw), s = std::sin(imu_yaw);
        // 차량 좌표계: x_local=전방(+), y_local=좌측(+)
        const double x_local =  c*dx + s*dy;
        const double y_local = -s*dx + c*dy;
        double cross_track_error = y_local;  // 좌측(+)·우측(-)이 차량 기준으로 일관

        // Stanley 제어 로직 여기서부터 --------------------------------------------------
        //
        double v = 10;
        double k = 5.0; // 제어 이득 (튜닝 필요)
        double eps = 0.2; // 저속 떨림 방지 < -- 좀더 알아봐야 될듯, 
        
        double stanley_steering = heading_error + std::atan2(k * cross_track_error, v + eps);

        g_cmd.steering = stanley_steering; // Stanley 제어 조향각 설정
        g_cmd.velocity = target_kmh; // 목표 속도 설정 (전역 파라미터 사용)

        cmd_pub.publish(g_cmd);

        ROS_INFO_THROTTLE(0.5,
            "[stanley_fixed] ENU(%.2f, %.2f) yaw=%.1fdeg steer=%.3frad v_set=%.2f(km/h) (near=%d)",
            g_enu_x, g_enu_y, imu_yaw*180.0/M_PI, g_cmd.steering, g_cmd.velocity, nearest_idx);

        rate.sleep();
    }
    return 0;
}
