#ifndef COSTMAP_HPP
#define COSTMAP_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>

#include <vector>
#include <string>

// ========================================
// Detection 구조체 (3D AABB)
// ========================================
struct Detection {
  int id{0};
  Eigen::Vector3f min_pt{0,0,0};
  Eigen::Vector3f max_pt{0,0,0};
  Eigen::Vector3f centroid{0,0,0};
};

// ========================================
// Costmap 파라미터
// ========================================
struct CostmapParams {
  float resolution = 0.05f;
  float width = 50.0f;
  float height = 50.0f;
  int8_t unknown_cost = -1;
  int8_t free_cost = 0;
  int8_t obstacle_cost = 100;
  float inflation_radius = 0.0f;
};

// ========================================
// 런타임 상태 (프레임마다 갱신)
// ========================================
struct CostmapState {
  bool tf_ok{false};
  sensor_msgs::PointCloud2 baselink_cloud;
  std::vector<Detection> detections;
  nav_msgs::OccupancyGrid costmap;
};

// ========================================
// 전역 변수 (이 구조 유지할 거면 extern)
// ========================================
extern tf2_ros::Buffer *tfBuffer;
extern ros::Publisher pub_costmap;
extern CostmapParams params;
extern std::string costmap_frame;
extern CostmapState state;

// ========================================
// 초기화
// ========================================
bool loadParams(const std::string &yaml_path);
void initCostmapModule(tf2_ros::Buffer *tf_buf, const std::string &frame);

// ========================================
// 콜백
// ========================================
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

// ========================================
// 주요 단계 함수들
// ========================================
void transformLidarToBaselink(const sensor_msgs::PointCloud2 &input);
void initializeCostmap(const ros::Time &stamp);
void clearOriginArea();

// (선택) 더미/실제 detection 연결용
void getDetections();

// ========================================
// 내부 유틸/헬퍼
// ========================================
std::string normalizeFrameId(std::string frame_id);
int clampToRange(int value, int min_val, int max_val);

void projectAABB3DTo2D(const Eigen::Vector3f &min_3d,
                       const Eigen::Vector3f &max_3d,
                       float &min_2d_x, float &min_2d_y,
                       float &max_2d_x, float &max_2d_y);

bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y);

int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y);

void projectAABBToGrid(const nav_msgs::OccupancyGrid &costmap,
                       float min_x, float min_y,
                       float max_x, float max_y,
                       int &grid_x0, int &grid_y0,
                       int &grid_x1, int &grid_y1,
                       float inflation_radius);

// 인덱스로 1칸 채우기 (콜백에서 흐름 노출용)
void fillCostAtIndex(nav_msgs::OccupancyGrid &costmap,
                     int idx,
                     int8_t cost_value);

#endif // COSTMAP_HPP
