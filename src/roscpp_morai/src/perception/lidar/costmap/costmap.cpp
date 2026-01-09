#include "costmap.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <algorithm>
#include <cmath>

// ========================================
// 헬퍼 함수
// ========================================

int clampToRange(int value, int min_val, int max_val) {
  return std::max(min_val, std::min(value, max_val));
}

void clearOriginArea(nav_msgs::OccupancyGrid &costmap,
                     int radius,
                     const CostmapParams &params)
{
  // 원점(0,0)의 그리드 좌표 찾기
  int origin_x, origin_y;
  if (!convertBaselinkToGrid(costmap, 0.0f, 0.0f, origin_x, origin_y)) {
    // 원점이 맵 밖이면 스킵
    return;
  }
  
  // 원점 주변 radius 범위를 free로 설정
  for (int y = origin_y - radius; y <= origin_y + radius; ++y) {
    for (int x = origin_x - radius; x <= origin_x + radius; ++x) {
      int idx = convertGridToIndex(costmap, x, y);
      if (idx >= 0 && idx < (int)costmap.data.size()) {
        costmap.data[idx] = params.free_cost;
      }
    }
  }
}


// ========================================
// 1. TF 좌표계 변환
// ========================================

bool transformLidarToBaselink(const sensor_msgs::PointCloud2 &input_cloud,
                              sensor_msgs::PointCloud2 &output_cloud,
                              const geometry_msgs::TransformStamped &transform)
{
  try {
    // TF2를 사용하여 PointCloud2 변환
    // LiDAR 좌표계 → base_link 좌표계
    // 여전히 3D (x, y, z 모두 유지)
    tf2::doTransform(input_cloud, output_cloud, transform);
    return true;
  }
  catch (const std::exception &e) {
    ROS_ERROR("TF transform failed: %s", e.what());
    return false;
  }
}

std::string normalizeFrameId(std::string frame_id) {
  // ROS TF에서 leading '/' 때문에 lookup 실패하는 경우 방지
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

// ========================================
// 2. 코스트맵 초기화
// ========================================

void initializeCostmap(nav_msgs::OccupancyGrid &costmap,
                       const std_msgs::Header &header,
                       const CostmapParams &params)
{
  // 헤더 설정
  costmap.header = header;
  
  // 해상도 설정
  costmap.info.resolution = params.resolution;
  
  // 그리드 크기 계산 (m → cell)
  const int width_cells = (int)std::round(params.width / params.resolution);
  const int height_cells = (int)std::round(params.height / params.resolution);
  
  costmap.info.width = width_cells;
  costmap.info.height = height_cells;
  
  // 원점 위치 설정
  // base_link 기준으로 맵의 중심이 차량 위치 (0,0)
  // 따라서 원점은 (-width/2, -height/2)
  costmap.info.origin.position.x = -params.width * 0.5;
  costmap.info.origin.position.y = -params.height * 0.5;
  costmap.info.origin.position.z = 0.0;
  
  // 회전 없음 (identity quaternion)
  costmap.info.origin.orientation.x = 0.0;
  costmap.info.origin.orientation.y = 0.0;
  costmap.info.origin.orientation.z = 0.0;
  costmap.info.origin.orientation.w = 1.0;
  
  // 모든 셀을 unknown으로 초기화
  const int total_cells = width_cells * height_cells;
  costmap.data.assign(total_cells, params.unknown_cost);
}

// ========================================
// 3. 3D → 2D 투영
// ========================================

void projectAABB3DTo2D(const Eigen::Vector3f &min_3d,
                       const Eigen::Vector3f &max_3d,
                       float &min_2d_x, float &min_2d_y,
                       float &max_2d_x, float &max_2d_y) //// AABB 박스 전체를 투영할 때
{
  // ★ 핵심: 3D 박스를 위에서 내려다본 2D 사각형
  // Z축을 무시하고 XY 평면만 사용
  min_2d_x = min_3d.x();
  min_2d_y = min_3d.y();
  max_2d_x = max_3d.x();
  max_2d_y = max_3d.y();
  // min_3d.z(), max_3d.z()는 무시!
}

// ========================================
// 4. 베이스링크 좌표계 → 코스트맵 그리드 좌표계
// ========================================

bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y)
{
  // base_link 월드 좌표(m) → 그리드 좌표(cell)
  // 
  // 변환 공식:
  // grid_x = floor((baselink_x - origin_x) / resolution)
  // grid_y = floor((baselink_y - origin_y) / resolution)
  //
  // origin은 맵의 왼쪽 아래 모서리
  // base_link (0,0)은 맵 중앙
  
  grid_x = (int)std::floor((baselink_x - costmap.info.origin.position.x) / costmap.info.resolution);
  grid_y = (int)std::floor((baselink_y - costmap.info.origin.position.y) / costmap.info.resolution);
  
  // 범위 체크
  if (grid_x < 0 || grid_x >= (int)costmap.info.width) return false;
  if (grid_y < 0 || grid_y >= (int)costmap.info.height) return false;
  
  return true;
}

int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y)
{
  // 2D 그리드 좌표 → 1D 배열 인덱스
  // row-major order: index = y * width + x
  
  // 범위 체크
  if (grid_x < 0 || grid_x >= (int)costmap.info.width) return -1;
  if (grid_y < 0 || grid_y >= (int)costmap.info.height) return -1;
  
  return grid_y * (int)costmap.info.width + grid_x;
}

// ========================================
// 5. 코스트맵에 장애물 투영
// ========================================

void projectAABBToGrid(const nav_msgs::OccupancyGrid &costmap,
                       float min_x, float min_y,
                       float max_x, float max_y,
                       int &grid_x0, int &grid_y0,
                       int &grid_x1, int &grid_y1,
                       float inflation_radius)
{
  // 좌표 정렬 (min < max 보장)
  if (min_x > max_x) std::swap(min_x, max_x);
  if (min_y > max_y) std::swap(min_y, max_y);
  
  // Inflation 적용 (장애물 주변 팽창)
  min_x -= inflation_radius;
  min_y -= inflation_radius;
  max_x += inflation_radius;
  max_y += inflation_radius;
  
  // base_link 좌표 → 그리드 좌표 변환
  
  // 시작 좌표
  if (!convertBaselinkToGrid(costmap, min_x, min_y, grid_x0, grid_y0)) {
    // 맵 밖이면 직접 계산
    grid_x0 = (int)std::floor((min_x - costmap.info.origin.position.x) / costmap.info.resolution);
    grid_y0 = (int)std::floor((min_y - costmap.info.origin.position.y) / costmap.info.resolution);
  }
  
  // 끝 좌표
  if (!convertBaselinkToGrid(costmap, max_x, max_y, grid_x1, grid_y1)) {
    // 맵 밖이면 직접 계산
    grid_x1 = (int)std::floor((max_x - costmap.info.origin.position.x) / costmap.info.resolution);
    grid_y1 = (int)std::floor((max_y - costmap.info.origin.position.y) / costmap.info.resolution);
  }
  
  // 좌표 정렬
  if (grid_x0 > grid_x1) std::swap(grid_x0, grid_x1);
  if (grid_y0 > grid_y1) std::swap(grid_y0, grid_y1);
  
  // 맵 범위로 제한
  grid_x0 = clampToRange(grid_x0, 0, (int)costmap.info.width - 1);
  grid_x1 = clampToRange(grid_x1, 0, (int)costmap.info.width - 1);
  grid_y0 = clampToRange(grid_y0, 0, (int)costmap.info.height - 1);
  grid_y1 = clampToRange(grid_y1, 0, (int)costmap.info.height - 1);
}

// ========================================
// 6. 그리드 칸에 코스트값 채우기
// ========================================

bool fillCostAtCell(nav_msgs::OccupancyGrid &costmap,
                    int grid_x, int grid_y,
                    int8_t cost_value)
{
  // 그리드 인덱스 계산
  int idx = convertGridToIndex(costmap, grid_x, grid_y);
  if (idx < 0 || idx >= (int)costmap.data.size()) {
    return false;
  }
  
  // 코스트 값 설정
  // 기존 값보다 큰 경우만 업데이트 (높은 코스트 유지)
  costmap.data[idx] = std::max(costmap.data[idx], cost_value);
  return true;
}

void fillCostInRegion(nav_msgs::OccupancyGrid &costmap,
                      int grid_x0, int grid_y0,
                      int grid_x1, int grid_y1,
                      int8_t cost_value)
{
  // 사각형 영역의 모든 셀을 채움
  const int width = (int)costmap.info.width;
  
  for (int y = grid_y0; y <= grid_y1; ++y) {
    for (int x = grid_x0; x <= grid_x1; ++x) {
      int idx = y * width + x;
      if (idx >= 0 && idx < (int)costmap.data.size()) {
        // 기존 값보다 큰 경우만 업데이트
        costmap.data[idx] = std::max(costmap.data[idx], cost_value);
      }
    }
  }
}

void fillObstacleAABB(nav_msgs::OccupancyGrid &costmap,
                      float min_x, float min_y,
                      float max_x, float max_y,
                      int8_t cost_value,
                      float inflation_radius)
{
  // 통합 함수: projectAABBToGrid + fillCostInRegion
  
  // 1) AABB를 그리드 좌표로 변환
  int grid_x0, grid_y0, grid_x1, grid_y1;
  projectAABBToGrid(costmap, 
                    min_x, min_y, max_x, max_y,
                    grid_x0, grid_y0, grid_x1, grid_y1,
                    inflation_radius);
  
  // 2) 그리드 영역에 코스트 값 채우기
  fillCostInRegion(costmap, 
                   grid_x0, grid_y0, grid_x1, grid_y1,
                   cost_value);
}

