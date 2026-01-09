#ifndef COSTMAP_HPP
#define COSTMAP_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <string>

// ========================================
// Costmap 파라미터 구조체
// ========================================
struct CostmapParams {
  // 코스트맵 크기 및 해상도
  float resolution = 0.05f;      // 그리드 한 칸 크기 (m)
  float width = 50.0f;           // 코스트맵 너비 (m)
  float height = 50.0f;          // 코스트맵 높이 (m)
  
  // 코스트 값
  int8_t unknown_cost = -1;      // 알 수 없는 영역
  int8_t free_cost = 0;          // 자유 공간
  int8_t obstacle_cost = 100;    // 장애물
  
  // Inflation
  float inflation_radius = 0.0f; // inflation 반경 (m)
};

// ========================================
// 1. TF 좌표계 변환
// ========================================
bool transformLidarToBaselink(const sensor_msgs::PointCloud2 &input_cloud,
                              sensor_msgs::PointCloud2 &output_cloud,
                              const geometry_msgs::TransformStamped &transform);

std::string normalizeFrameId(std::string frame_id);

// ========================================
// 2. 코스트맵 초기화
// ========================================
void initializeCostmap(nav_msgs::OccupancyGrid &costmap,
                       const std_msgs::Header &header,
                       const CostmapParams &params);

// ========================================
// 3. 3D → 2D 투영
// ========================================

void projectAABB3DTo2D(const Eigen::Vector3f &min_3d,
                       const Eigen::Vector3f &max_3d,
                       float &min_2d_x, float &min_2d_y,
                       float &max_2d_x, float &max_2d_y);

// ========================================
// 4. 베이스링크 좌표계 → 코스트맵 그리드 좌표계
// ========================================
bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y);

int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y);

// ========================================
// 5. 코스트맵에 장애물 투영
// ========================================

void projectAABBToGrid(const nav_msgs::OccupancyGrid &costmap,
                       float min_x, float min_y,
                       float max_x, float max_y,
                       int &grid_x0, int &grid_y0,
                       int &grid_x1, int &grid_y1,
                       float inflation_radius = 0.0f);

// ========================================
// 6. 그리드 칸에 코스트값 채우기
// ========================================

bool fillCostAtCell(nav_msgs::OccupancyGrid &costmap,
                    int grid_x, int grid_y,
                    int8_t cost_value);

void fillCostInRegion(nav_msgs::OccupancyGrid &costmap,
                      int grid_x0, int grid_y0,
                      int grid_x1, int grid_y1,
                      int8_t cost_value);

void fillObstacleAABB(nav_msgs::OccupancyGrid &costmap,
                      float min_x, float min_y,
                      float max_x, float max_y,
                      int8_t cost_value,
                      float inflation_radius = 0.0f);

// ========================================
// 헬퍼 함수
// ========================================

int clampToRange(int value, int min_val, int max_val);

void clearOriginArea(nav_msgs::OccupancyGrid &costmap,
                     int radius,
                     const CostmapParams &params);

#endif 