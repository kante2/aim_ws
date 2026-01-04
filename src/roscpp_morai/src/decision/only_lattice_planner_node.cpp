#include "hybrid_function.hpp"
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <iostream> 
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <utility>
#include <Eigen/Dense>

using namespace std;

// ====================== 전역 변수 정의 ======================
std::vector<Waypoint> waypoints;
VehicleState ego;
ControlData ctrl;
ros::Publisher cmd_pub;
ros::Publisher local_path_pub;
ros::Publisher candidate_paths_pub;

// Costmap 관련 전역 변수
nav_msgs::OccupancyGrid::ConstPtr global_costmap; // 글로벌 코스트맵 포인터
double costmap_resolution = 0.0;
double costmap_origin_x = 0.0;
double costmap_origin_y = 0.0;

// YAML 변수
string path_file_name;
string ref_file_name;
double target_vel;
double curve_vel;
double curve_standard;
double k_gain;
double ld_gain;
int lookahead_idx;
double Kp, Ki, Kd;

// Lattice Planner 파라미터
double LD = 5.0;  // Look Ahead Distance
double LATERAL_OFFSET_STEP = 0.5;  // Lateral offset 간격
double NUM_OFFSETS = 9;  // 샘플링 경로 개수
double SAMPLE_SPACING = 0.2;  // 경로 샘플링 간격
double LETHAL_COST_THRESHOLD = 254;  // Lethal obstacle threshold

// --- 차량/경로 상태 변수 ---
double lat0, lon0, h0;
double x0_ecef, y0_ecef, z0_ecef;

// Lattice planner 관련 구조체
struct CandidatePath {
    vector<pair<double, double>> points;  // (x, y) 포인트들
    double cost = 1e10;             // 총 비용
    double obstacle_cost = 0.0;     // 장애물 비용
    double offset_cost = 0.0;       // offset 비용
    double curvature_cost = 0.0;    // 곡률 비용
    double offset = 0.0;            // 실제 offset 값
    bool valid = true;              // 유효한 경로인지
};

// 이전 선택된 offset (히스테리시스용)
// 시스템의 현재 상태가 과거 이력에 따라 달라지는 현상
double last_selected_offset = 0.0;



// ====================== 좌표 변환 함수 ======================
void wgs84ToECEF(double lat, double lon, double h,
                 double& x, double& y, double& z) {
    double a = 6378137.0;
    double e2 = 6.69437999014e-3;

    double rad_lat = lat * M_PI / 180.0;
    double rad_lon = lon * M_PI / 180.0;
    double N = a / sqrt(1 - e2 * sin(rad_lat) * sin(rad_lat));

    x = (N + h) * cos(rad_lat) * cos(rad_lon);
    y = (N + h) * cos(rad_lat) * sin(rad_lon);
    z = (N * (1 - e2) + h) * sin(rad_lat);
}

void wgs84ToENU(double lat, double lon, double h,
                double lat_ref, double lon_ref, double h_ref,
                double& x, double& y, double& z) {
    double x_ecef, y_ecef, z_ecef;
    wgs84ToECEF(lat, lon, h, x_ecef, y_ecef, z_ecef);

    double dx = x_ecef - x0_ecef;
    double dy = y_ecef - y0_ecef;
    double dz = z_ecef - z0_ecef;

    double rad_lat = lat_ref * M_PI / 180.0;
    double rad_lon = lon_ref * M_PI / 180.0;

    double t[3][3] = {
        {-sin(rad_lon), cos(rad_lon), 0},
        {-sin(rad_lat) * cos(rad_lon), -sin(rad_lat) * sin(rad_lon), cos(rad_lat)},
        {cos(rad_lat) * cos(rad_lon), cos(rad_lat) * sin(rad_lon), sin(rad_lat)}
    };

    x = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz;
    y = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz;
    z = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz;
}

double quaternionToYaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return atan2(siny_cosp, cosy_cosp);
}

// ====================== 스탠리에 쓰이는 함수 ======================
// double lateralPathError(int target_idx, double x, double y){ //정의
//     int i = target_idx;
//     if( i < waypoints.size() -1){
//     double a = waypoints[i].x;
//     double b = waypoints[i].y;
//     double B = waypoints[i+1].x - waypoints[i].x;
//     double A = waypoints[i+1].y - waypoints[i].y;
//     double C = -a*A + b*B; // A*x -B*y + C;
//     if(A==0 && B==0){return 0.0; }
//     return (x*A - y*B+ C ) /sqrt(A*A + B*B);
//     }
//     else { return 0.0; }
// }

// double headingError(double yaw,int target_idx){
//     int i = target_idx;
//     if(i< waypoints.size()-1){
//     double dx = waypoints[i+1].x - waypoints[i].x;
//     double dy = waypoints[i+1].y - waypoints[i].y;
//     double path_heading = atan2(dy,dx);
//     double diff = path_heading - yaw;
//     return atan2(sin(diff),cos(diff));
//     }
//     else {return 0.0;}
// }

// ====================== main안 함수  ======================

bool load_Path_ref(){ //경로,기준점 체크 함수
    // Convert relative paths to absolute using ROS package path
    std::string pkg = ros::package::getPath("roscpp_morai");
    if (ref_file_name[0] != '/') {  
        ref_file_name = pkg + "/" + ref_file_name;
    }
    if (path_file_name[0] != '/') {
        path_file_name = pkg + "/" + path_file_name;
    }

    ROS_INFO("Opening ref file: %s", ref_file_name.c_str());
    ifstream ref_file(ref_file_name);
    if (!ref_file.is_open()) {
        ROS_ERROR("Failed to open ref file: %s", ref_file_name.c_str());
        return false;
    }
    ref_file >> lat0 >> lon0 >> h0;
    ref_file.close();
    wgs84ToECEF(lat0, lon0, h0, x0_ecef, y0_ecef, z0_ecef);

    ROS_INFO("Opening path file: %s", path_file_name.c_str());
    ifstream path_file(path_file_name);
    if (!path_file.is_open()) {
        ROS_ERROR("Failed to open path file: %s", path_file_name.c_str());
        return false;
    }

    // --- Parse CSV (space or comma separated) ---
    string line;
    while (getline(path_file, line)) {
        if (line.empty()) continue;
        
        stringstream ss(line);
        double x, y, z;
        
        // Try space-separated format first
        if (ss >> x >> y >> z) {
            waypoints.push_back({x, y});
        } else {
            // Try comma-separated format
            ss.clear();
            ss.str(line);
            string val;
            vector<double> row;
            
            while (getline(ss, val, ',')) {
                val.erase(0, val.find_first_not_of(" \t\r\n"));
                val.erase(val.find_last_not_of(" \t\r\n") + 1);
                
                try {
                    row.push_back(stod(val));
                } catch (...) {
                    continue;
                }
            }
            
            if (row.size() >= 2) {
                waypoints.push_back({row[0], row[1]});
            }
        }
    }
    // ------------------------------------

    path_file.close();
    ROS_INFO("Waypoints loaded: %zu", waypoints.size());

    // 데이터가 너무 적으면 곡률 계산 시 세그먼테이션 폴트가 발생하므로 체크
    if (waypoints.size() < 3) {
        ROS_ERROR("Not enough waypoints (size: %zu). Curvature calculation requires at least 3 points.", waypoints.size());
        return false;
    }

    return true;
}


bool load_Params(const std::string& yaml_hybrid) { //야뮬 체크 함수
  try {
    YAML::Node config3 = YAML::LoadFile(yaml_hybrid);

    // ==========================================================
    // 여기에 변환 상수를 추가합니다
    // ==========================================================
    const double KPH_TO_MPS = 1.0 / 3.6; // (km/h -> m/s 변환 상수)
    // ==========================================================

    // 1. 임시 변수 (km/h)에 원본 값 로드
    // (로그 출력을 위해 원본 km/h 값을 보관)
    double target_vel_kph     = config3["target_vel"].as<double>();
    double curve_vel_kph    = config3["curve_vel"].as<double>();



    // 2. 나머지 파라미터 로드 (단위 변환 불필요)
    curve_standard     = config3["curve_standard"].as<double>();
    k_gain           = config3["k_gain"].as<double>();
    ld_gain          = config3["ld_gain"].as<double>();
    lookahead_idx= config3["lookahead_idx"].as<int>();
    Kp               = config3["Kp"].as<double>();
    Ki               = config3["Ki"].as<double>();
    Kd               = config3["Kd"].as<double>();
    path_file_name   = config3["path_file_name"].as<std::string>();
    ref_file_name    = config3["ref_file_name"].as<std::string>();

    
    // ==========================================================
    // 3. (중요) 전역 변수에 m/s로 변환하여 저장
    // ==========================================================
    target_vel = target_vel_kph * KPH_TO_MPS;
    curve_vel = curve_vel_kph * KPH_TO_MPS;



    // 4. 로그 출력 (사용자 가독성을 위해 km/h 원본 값과 m/s 변환 값 함께 표시)
    ctrl.lookahead_idx = lookahead_idx;
    return true;

  } catch (const std::exception& e) {
    std::cerr << "YAML Load Error: " << e.what() << std::endl;
    return false;
  }
}

void preprocessCurvature() {
    for (int i=0; i < waypoints.size()-2; ++i){
        double dx1 = waypoints[i+1].x - waypoints[i].x;
        double dx2 = waypoints[i+2].x - waypoints[i+1].x;
        double dy1 = waypoints[i+1].y - waypoints[i].y;
        double dy2 = waypoints[i+2].y - waypoints[i+1].y;
        double alpha1 = std::atan2(dy1, dx1);
        double alpha2 = std::atan2(dy2, dx2);
        double k_val = fabs(alpha1 - alpha2);
        waypoints[i+1].curvature = k_val;
        // ← i+1번 점에 저장!
    }
    cout << "=== End Preprocessing ===" << endl;
    waypoints[0].curvature = waypoints[1].curvature;  // ← 보간
    waypoints.back().curvature = waypoints[waypoints.size()-2].curvature;
}


// ====================== 출발/도착 판정 함수 ======================



void closeWaypointsIdx(const VehicleState& ego, int& out_idx){ //const는 구조체 전체를 읽기 전용으로 받음 ,출력: int& out_idx -> 결과를 여기에 담으면 ctrl.close_idx가 바뀜
    static int last_close_idx = 0;
    double best_close_dist = 10000000000;
    int close_idx = last_close_idx;
    int start = std::max(0,last_close_idx - 10);
    int end = std::min((int)waypoints.size() - 1,last_close_idx + 30);
    for(int i = start; i <= end ; ++i){
        double dx = waypoints[i].x - ego.x;
        double dy = waypoints[i].y - ego.y;
        double dist = sqrt(dx*dx + dy*dy);
        if (dist < best_close_dist){
                best_close_dist = dist;
                close_idx = i;
            }
           
        }
    last_close_idx = close_idx;
    out_idx = close_idx;
    ROS_INFO("close_idx: %d",close_idx);
}

void getGoalwaypoint(const VehicleState& ego, int close_idx, int& out_target_idx, double& ld){
    ld = 5.0 + ld_gain * ego.vel;
    int target_idx = close_idx;
    int i = close_idx;
    for(; i <= waypoints.size()-1; ++i ){
        double dx = waypoints[i].x-ego.x;
        double dy = waypoints[i].y-ego.y;
        double dist = sqrt(dx*dx + dy*dy);
        if(dist > ld){
            target_idx = i;
            break;
          }
        }
    out_target_idx = target_idx;
}


// void getTargetSpeed(double max_curvature, double& out_target_vel){
//     if(max_curvature > curve_standard){
//         out_target_vel = curve_vel;
//     }
//     else {out_target_vel = target_vel;}
// }

// void getsteering(const VehicleState& ego, ControlData& ctrl){
//     double path_e = lateralPathError(ctrl.target_idx, ego.x, ego.y);
//     double heading_e = headingError(ego.yaw, ctrl.target_idx);
//     double v = std::max(1.0, ego.vel);
    
//     // Stanley steering control with limits
//     double steering_raw = heading_e + atan(k_gain * path_e / v);
    
//     // Limit steering angle to ±30 degrees (±0.524 radians)
//     const double MAX_STEERING = 30.0 * M_PI / 180.0;  // 30 degrees
//     ctrl.steering = std::max(-MAX_STEERING, std::min(MAX_STEERING, steering_raw));
    
//     ROS_INFO("[Steering] path_e: %.3f, heading_e: %.3f, raw: %.3f, limited: %.3f deg",
//              path_e, heading_e, steering_raw, ctrl.steering * 180.0 / M_PI);
// }

// void computePID(double vel,double target_vel,double& out_accel, double& out_brake){

//     static double prev_error = 0.0;     // 이전 오차 기억용
//     static double integral_error = 0.0; // 적분 누적용
//     double error = target_vel - vel;
//     integral_error += error * 0.02;

//     if(integral_error > 10.0) integral_error = 10.0;
//     if(integral_error < -10.0) integral_error = -10.0;

//     double p_error = Kp*error;
//     double i_error = Ki*integral_error; //controlLoop가 0.02초(50Hz)마다 도니까 dt = 0.02
//     double d_error = Kd*((error - prev_error)/0.02);
//     prev_error = error;

//     double total_output = p_error + i_error + d_error;

//     if (total_output > 0) {
//         out_accel = min(total_output, 1.0);
//         out_brake = 0.0;
//     } 
//     else {
//         out_accel = 0.0;
//         out_brake = min(-total_output, 1.0); 
//     }

// }


void pubCmd(const ControlData& data) {
    morai_msgs::CtrlCmd cmd;
    cmd.longlCmdType = 1;
    cmd.accel = data.accel;
    cmd.brake = data.brake;
    cmd.steering = data.steering;
    cmd_pub.publish(cmd);
}




// ====================== CALLBACK ======================

void gpsCallback(const morai_msgs::GPSMessage::ConstPtr& msg) {
    double x, y, z;
    wgs84ToENU(msg->latitude, msg->longitude, msg->altitude,
               lat0, lon0, h0, x, y, z);
    ego.x = x;
    ego.y = y;
}

void egoCallback(const morai_msgs::EgoVehicleStatus::ConstPtr& msg){
    ego.vel = msg->velocity.x;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ego.yaw = quaternionToYaw(msg->orientation.x,
                              msg->orientation.y,
                              msg->orientation.z,
                              msg->orientation.w);

}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    global_costmap = msg;
    costmap_resolution = msg->info.resolution;
    costmap_origin_x = msg->info.origin.position.x;
    costmap_origin_y = msg->info.origin.position.y;
}

// ====================== Lattice Planner 함수들 ======================

// ** 
// 5차 다항식으로 시작점과 끝점을 연결 (시작 각도와 끝 각도도 고려)
// Returns: (x, y) 좌표 쌍의 벡터
vector<pair<double, double>> generatePolynomialPath(
    double start_x, double start_y, double start_yaw,
    double end_x, double end_y, double end_yaw,
    double sample_spacing) {
    
    vector<pair<double, double>> path;
    
    // 로컬 좌표계로 변환 (시작점이 원점, 시작 방향이 x축)
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < 1e-6) {
        path.push_back({start_x, start_y});
        return path;
    }
    
    // 간단한 선형 보간 (5차 다항식 대신 더 간단한 방법)
    int num_points = (int)(distance / sample_spacing) + 1;
    for (int i = 0; i <= num_points; i++) {
        // 진행 퍼센트
        // 조건 ? (조건이 true일 때 값) : (조건이 false일 때 값)
        double t = (num_points > 0) ? (double)i / num_points : 0.0; 
        double x = start_x + dx * t;
        double y = start_y + dy * t;
        path.push_back({x, y});
    }
    
    return path;
}

// **
// Costmap 좌표계로 변환
// 그 좌표가 맵 안에 들어오는지까지 검사
// (world_x, world_y): 월드 좌표 --> (grid_x, grid_y): 그리드 좌표
bool worldToCostmapCoord(double world_x, double world_y, int& grid_x, int& grid_y) {
    if (!global_costmap) return false;
    
    // 월드 좌표 -> costmap 그리드 좌표
    // costmap_resolution : 그리드 한 칸의 크기 (m)
    grid_x = (int)((world_x - costmap_origin_x) / costmap_resolution);
    grid_y = (int)((world_y - costmap_origin_y) / costmap_resolution);
    
    // 범위 체크
    if (grid_x < 0 || grid_x >= (int)global_costmap->info.width ||
        grid_y < 0 || grid_y >= (int)global_costmap->info.height) {
        return false;
    }
    
    return true; 
    // true  : 범위 안
    // false : 범위 밖
}

// ** 
// Costmap에서 특정 좌표의 비용 읽기
uint8_t getCostmapCost(double world_x, double world_y) {
    if (!global_costmap) return 0;
    
    int grid_x, grid_y;
    if (!worldToCostmapCoord(world_x, world_y, grid_x, grid_y)) {
        return LETHAL_COST_THRESHOLD + 1;  // Out of bounds = 치명적
    }
    // costmap의 1차원 데이터 배열에서 인덱스 계산
    // (grid_x, grid_y) → 1차원 인덱스(idx)로 변환
    int idx = grid_y * global_costmap->info.width + grid_x;
    if (idx < 0 || idx >= (int)global_costmap->data.size()) {
        return LETHAL_COST_THRESHOLD + 1;   // Out of bounds = 치명적
    }
    
    return global_costmap->data[idx]; // read cost
}

// **
// 경로의 장애물 비용 평가
void evaluatePathCollision(CandidatePath& path) {
    path.obstacle_cost = 0.0;
    path.valid = true;
    
    int lethal_count = 0;
    
    for (const auto& pt : path.points) {
        // pt.first  : x 좌표
        // pt.second : y 좌표
        uint8_t cost = getCostmapCost(pt.first, pt.second);
        
        if (cost >= LETHAL_COST_THRESHOLD) {
            lethal_count++;
        }
        
        // 장애물 비용 누적 (정규화)
        // cost / 255.0 : 0~1 사이 값으로 정규화
        // ** 누적합은 점 개수에 따라 자동으로 커지거나 작아져서 공정 비교가 어려움 <1>
        path.obstacle_cost += cost / 255.0;
    }
    
    // Lethal cost를 만나면 경로 폐기 --> VALID = false
    if (lethal_count > 0) {
        path.valid = false;
        path.cost = 1e10;
        return;
    }

    // <2>마지막에 점 개수로 나눠서 평균을 만들기
    // 평균 = (각 점의 정규화 비용 합) / (점 개수)
    if (path.points.size() > 0) {
        path.obstacle_cost /= path.points.size();
    }
}

//**
// 경로의 곡률 계산
// 가장 많이 꺾이는 구간”이 얼마나 심한지를 하나의 값으로 뽑아내는 함수
// “진짜 곡률(κ)”과는 조금 다름
double calculatePathCurvature(const vector<pair<double, double>>& path) {
    double max_curvature = 0.0;
    //점이 3개 미만이면 0 반환
    if (path.size() < 3) return 0.0;
    
    // i번째 점에서 꺾임 계산
    for (size_t i = 1; i < path.size() - 1; i++) {
        // 3점 (i-1, i, i+1)
        double x0 = path[i-1].first, y0 = path[i-1].second;
        double x1 = path[i].first, y1 = path[i].second;
        double x2 = path[i+1].first, y2 = path[i+1].second;
        
        // 두 구간 벡터 만들기
        double dx1 = x1 - x0, dy1 = y1 - y0; // v1 = P0 -> P1
        double dx2 = x2 - x1, dy2 = y2 - y1; // v2 = P1 -> P2
        // θ = 0° (직선) → sin(0)=0 → curvature=0
        // θ = 90° (직각) → sin(90)=1 → curvature≈1
        // θ가 클수록(급격히 꺾일수록) 값이 커짐
        // curvature = |sin(θ)| / (|v1| * |v2|)
        double curvature = fabs((dx1*dy2 - dy1*dx2) / 
                                (sqrt(dx1*dx1 + dy1*dy1) * sqrt(dx2*dx2 + dy2*dy2) + 1e-6));
        max_curvature = max(max_curvature, curvature);
    }
    
    return max_curvature;
}

// 모든 후보 경로 생성 및 평가
vector<CandidatePath> generateAndEvaluateCandidates(
    const VehicleState& ego,
    // global path(waypoints)에서 lookahead로 잡은 목표 인덱스
    int goal_idx, 
    // 목표 참조점(보통 goal_idx waypoint 좌표)
    double goal_ref_x, double goal_ref_y) {
    
    vector<CandidatePath> candidates;
    
    // goal_ref 지점에서 “경로 진행 방향 벡터(dir)” 구하기
    // 경로의 방향 벡터 (goal_ref를 중심으로)
    double dir_x = 0.0, dir_y = 0.0;
    if (goal_idx < waypoints.size() - 1) {
        // direction from goal_idx to goal_idx + 1
        double dx = waypoints[goal_idx + 1].x - waypoints[goal_idx].x;
        double dy = waypoints[goal_idx + 1].y - waypoints[goal_idx].y;
        double len = sqrt(dx*dx + dy*dy);
        if (len > 1e-6) {
            dir_x = dx / len;
            dir_y = dy / len;
        }
    }
    
    // 법선 벡터 (경로 방향에 수직)
    // norm 방향이 횡방향 offset을 주는 방향
    double norm_x = -dir_y;
    double norm_y = dir_x;
    
    // 여러 offset 값으로 후보 경로 생성
    // NUM_OFFSETS개의 후보
    // ex ) NUM_OFFSETS=9, STEP=0.5면 [-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2]
    // 이런 식으로 중앙(0)을 기준으로 좌우 대칭으로 생성
    for (int i = 0; i < (int)NUM_OFFSETS; i++) {
        double offset = -LATERAL_OFFSET_STEP * (NUM_OFFSETS - 1) / 2.0 + 
                        LATERAL_OFFSET_STEP * i;
        
        // Goal point with offset
        // standard : goal_ref --> goal (offset 적용) 
        double goal_x = goal_ref_x + offset * norm_x;
        double goal_y = goal_ref_y + offset * norm_y;
        
        // 시작점의 yaw 추정
        double start_yaw = atan2(norm_y, norm_x);
        
        // 끝점의 yaw (경로 방향)
        double end_yaw = atan2(dir_y, dir_x);
        
        // 경로 생성
        // ego -> offset goal point 까지 경로 생성
        vector<pair<double, double>> path = generatePolynomialPath(
            ego.x, ego.y, ego.yaw,
            goal_x, goal_y, end_yaw,
            SAMPLE_SPACING
        );
        
        // 후보 경로 생성
        CandidatePath candidate;
        candidate.points = path;
        candidate.offset = offset;
        
        // 충돌 평가
        evaluatePathCollision(candidate);
        
        if (candidate.valid) {
            // 비용 계산
            double curvature = calculatePathCurvature(path);
            candidate.curvature_cost = curvature;
            
            // Offset 페널티 (중앙 유지)
            candidate.offset_cost = fabs(offset) * 0.5;
            
            // 이전 offset과의 변화 페널티 (히스테리시스)
            double offset_change_cost = fabs(offset - last_selected_offset) * 0.3;
            
            // 총 비용
            candidate.cost = candidate.obstacle_cost * 100.0 + 
                           candidate.offset_cost + 
                           candidate.curvature_cost * 10.0 +
                           offset_change_cost;
            
            candidates.push_back(candidate);
        }
    }
    
    return candidates; // CandidatePath 구조체(클래스) 객체들이 여러 개 들어있는 벡터
}

// 최적 경로 선택
CandidatePath selectBestPath(const vector<CandidatePath>& candidates) {
    // exception 처리: 후보가 하나도 없으면 중앙 경로 반환
    if (candidates.empty()) {
        // 유효한 후보가 없으면 중앙 경로 반환
        CandidatePath fallback;
        fallback.valid = true;
        fallback.offset = 0.0;
        return fallback;
    }
    
    double best_cost = 1e10; // 매우 큰 값 -> 최소값 찾기 용
    int best_idx = 0;
    
    // 가장 비용이 낮은 후보 선택
    for (int i = 0; i < (int)candidates.size(); i++) {
        if (candidates[i].cost < best_cost) {
            best_cost = candidates[i].cost;
            best_idx = i;
        }
    }
    // 전역/외부 변수 last_selected_offset에 “이번에 선택된 경로의 offset”을 저장해둠.
    // -> double offset_change_cost = fabs(offset - last_selected_offset) * 0.3;
    //   에서 사용됨 (히스테리시스 효과),, 좌우로 경로가 자주 바뀌는 걸 완화(부드럽게)
    last_selected_offset = candidates[best_idx].offset;
    return candidates[best_idx];
}

// Path를 nav_msgs/Path로 변환
// candidatePath (x,y) --> nav_msgs/Path 형식 to make easier visualization and use
nav_msgs::Path convertToNavPath(const CandidatePath& path) {
    nav_msgs::Path nav_path; // : 이 Path 메시지를 만든 시간
    nav_path.header.stamp = ros::Time::now();
    nav_path.header.frame_id = "map"; // 이 경로가 표현되는 좌표계 프레임
    
    for (const auto& pt : path.points) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = pt.first;
        pose.pose.position.y = pt.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        
        nav_path.poses.push_back(pose);
    }
    
    return nav_path;
}

// 모든 후보 경로를 MarkerArray로 시각화
visualization_msgs::MarkerArray visualizeCandidatePaths(
    const vector<CandidatePath>& candidates) {
    
    visualization_msgs::MarkerArray markers;
    
    for (int i = 0; i < (int)candidates.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.1;  // Line width
        
        // 유효한 경로는 녹색, 무효한 경로는 빨강
        if (candidates[i].valid) {
            marker.color.g = 1.0;
            marker.color.a = 0.5;
        } else {
            marker.color.r = 1.0;
            marker.color.a = 0.2;
        }
        
        for (const auto& pt : candidates[i].points) {
            geometry_msgs::Point p;
            p.x = pt.first;
            p.y = pt.second;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        
        markers.markers.push_back(marker);
    }
    
    return markers;
}


void controlLoop(const ros::TimerEvent&) {
    if (!global_costmap) {
        ROS_WARN("Waiting for costmap...");
        return;
    }

    // 1. 현재 위치에서 가장 가까운 경로 포인트 찾기
    closeWaypointsIdx(ego, ctrl.close_idx);
    
    // 2. LD 기반으로 goal 포인트 결정
    getGoalwaypoint(ego, ctrl.close_idx, ctrl.target_idx, ctrl.ld);
    
    // Goal reference point 추출
    double goal_ref_x = waypoints[ctrl.target_idx].x;
    double goal_ref_y = waypoints[ctrl.target_idx].y;
    
    ROS_INFO("Goal Ref: (%.2f, %.2f), LD: %.2f", goal_ref_x, goal_ref_y, ctrl.ld);
    
    // 3. 후보 경로 생성 및 평가
    vector<CandidatePath> candidates = generateAndEvaluateCandidates(
        ego, ctrl.target_idx, goal_ref_x, goal_ref_y
    );
    
    ROS_INFO("Generated %zu candidate paths", candidates.size());
    
    // 4. 최적 경로 선택
    CandidatePath best_path = selectBestPath(candidates);
    
    // 5. 선택된 경로 퍼블리시 (nav_msgs/Path)
    if (best_path.valid && !best_path.points.empty()) {
        nav_msgs::Path local_path = convertToNavPath(best_path);
        local_path_pub.publish(local_path);
        
        ROS_INFO("Selected path with cost: %.4f, offset: %.2f", 
                 best_path.cost, best_path.offset);
    } else {
        ROS_WARN("No valid candidate path found!");
    }
    
    // 6. 모든 후보 경로 시각화
    visualization_msgs::MarkerArray marker_array = visualizeCandidatePaths(candidates);
    candidate_paths_pub.publish(marker_array);
    
    // 제어 부분은 제거됨 (costmap 기반 경로 생성만 수행)
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "lattice_planner");
    ros::NodeHandle nh;

    // Get ROS package path for file locations
    std::string pkg = ros::package::getPath("roscpp_morai");
    std::string yaml_file = pkg + "/config/yaml_hybrid.yaml";

    // Load YAML parameters first
    if (!load_Params(yaml_file)) {
        ROS_FATAL("Failed to load YAML parameters!");
        return -1;
    }
    
    // Load path and reference point
    if (!load_Path_ref()) {
        ROS_FATAL("Failed to load path and reference files!");
        return -1;
    }
    
    // Preprocess curvature
    preprocessCurvature();
    
    // Publisher 설정
    cmd_pub = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd_0", 1);
    local_path_pub = nh.advertise<nav_msgs::Path>("/local_path", 1);
    candidate_paths_pub = nh.advertise<visualization_msgs::MarkerArray>("/candidate_paths", 1);
    
    // Subscriber 설정
    ros::Subscriber gps_sub = nh.subscribe("/gps", 1, gpsCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback);
    ros::Subscriber ego_sub = nh.subscribe("/Ego_topic", 1, egoCallback);
    ros::Subscriber costmap_sub = nh.subscribe("/costmap_from_lidar", 1, costmapCallback);
    
    // Control loop timer (50 Hz)
    ros::Timer timer = nh.createTimer(ros::Duration(0.02), controlLoop);

    ROS_INFO("Lattice Planner Node Started!");
    ROS_INFO("Waiting for costmap...");
    
    ros::spin();

    return 0;
}

