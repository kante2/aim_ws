#include "costmap.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

using namespace std;

// ========================================
// 전역 변수 정의 (extern의 실제 실체)
// ========================================
tf2_ros::Buffer *tfBuffer = nullptr;
ros::Publisher pub_costmap;
CostmapParams params;
std::string costmap_frame = "base_link";
CostmapState state;

// ========================================
// 초기화
// ========================================

bool loadParams(const std::string &yaml_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    params.resolution       = config["resolution"].as<float>();
    params.width            = config["width"].as<float>();
    params.height           = config["height"].as<float>();
    params.unknown_cost     = config["unknown_cost"].as<int>();
    params.free_cost        = config["free_cost"].as<int>();
    params.obstacle_cost    = config["obstacle_cost"].as<int>();
    params.inflation_radius = config["inflation_radius"].as<float>();

    costmap_frame = config["costmap_frame"].as<std::string>();

    ROS_INFO("Loaded costmap params from YAML");
    return true;
  }
  catch (const std::exception &e) {
    ROS_ERROR("Failed to load YAML: %s", e.what());
    return false;
  }
}

void initCostmapModule(tf2_ros::Buffer *tf_buf, const std::string &frame) {
  tfBuffer = tf_buf;
  costmap_frame = normalizeFrameId(frame);
  ROS_INFO("Costmap module initialized (frame=%s)", costmap_frame.c_str());
}

// ========================================
// 라이다콜백 (라이다한테 state.detections 받고나서 흐름임, 콜백폴더에 옮겨야함 임시로 여기에 뒀어용)
// ========================================

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

  transformLidarToBaselink(*msg);   // 라이다좌표계-> 베이스링크좌표계 TF 변환
  if (!state.tf_ok) return;
  initializeCostmap(msg->header.stamp); // Costmap 초기화
  clearOriginArea();  //원점 주변 클리어

  for (const auto &det : state.detections) // 장애물마다 단계별로 직접 호출하며 투영
  {
    float min_x, min_y, max_x, max_y;
    projectAABB3DTo2D(det.min_pt, det.max_pt, min_x, min_y, max_x, max_y); // 3D AABB -> 2D AABB
    int gx0, gy0, gx1, gy1;
    projectAABBToGrid(state.costmap,min_x, min_y, max_x, max_y,gx0, gy0, gx1, gy1,params.inflation_radius); // 2D AABB -> grid 좌표
    for (int y = gy0; y <= gy1; ++y) // grid 영역 채우기
    {
      for (int x = gx0; x <= gx1; ++x)
      {
        int idx = convertGridToIndex(state.costmap, x, y); // grid -> index
        fillCostAtIndex(state.costmap, idx, params.obstacle_cost); // 비용 채우기
      }
    }
  }
  pub_costmap.publish(state.costmap);

}

// ========================================
// 0.헬퍼 함수
// ========================================

std::string normalizeFrameId(std::string frame_id) {
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

int clampToRange(int value, int min_val, int max_val) {
  return std::max(min_val, std::min(value, max_val));
}

// ========================================
// 1. TF 변환
// ========================================

void transformLidarToBaselink(const sensor_msgs::PointCloud2 &input) {
  state.tf_ok = false;

  const std::string src = normalizeFrameId(input.header.frame_id);
  const std::string tgt = costmap_frame;

  if (src == tgt) {
    state.baselink_cloud = input;
    state.tf_ok = true;
    return;
  }

  try {
    if (!tfBuffer) {
      ROS_WARN_THROTTLE(1.0, "tfBuffer is null. Did you call initCostmapModule()?");
      state.tf_ok = false;
      return;
    }

    geometry_msgs::TransformStamped tf =
        tfBuffer->lookupTransform(tgt, src, ros::Time(0), ros::Duration(0.05));

    tf2::doTransform(input, state.baselink_cloud, tf);
    state.tf_ok = true;
  }
  catch (const std::exception &e) {
    ROS_WARN_THROTTLE(1.0, "TF error: %s", e.what());
    state.tf_ok = false;
  }
}

// ========================================
// 2. Costmap 초기화
// ========================================

void initializeCostmap(const ros::Time &stamp) {
  state.costmap.header.frame_id = costmap_frame;
  state.costmap.header.stamp = stamp;

  state.costmap.info.resolution = params.resolution;

  const int w = (int)std::round(params.width / params.resolution);
  const int h = (int)std::round(params.height / params.resolution);

  state.costmap.info.width = w;
  state.costmap.info.height = h;

  state.costmap.info.origin.position.x = -params.width * 0.5;
  state.costmap.info.origin.position.y = -params.height * 0.5;
  state.costmap.info.origin.position.z = 0.0;

  state.costmap.info.origin.orientation.w = 1.0;
  state.costmap.info.origin.orientation.x = 0.0;
  state.costmap.info.origin.orientation.y = 0.0;
  state.costmap.info.origin.orientation.z = 0.0;

  const int total = w * h;
  state.costmap.data.assign(total, params.unknown_cost);
}

// ========================================
// 3. 원점 주변 클리어
// ========================================

void clearOriginArea() {
  int ox, oy;
  if (!convertBaselinkToGrid(state.costmap, 0.0f, 0.0f, ox, oy)) return;

  const int radius = 3;
  for (int y = oy - radius; y <= oy + radius; ++y) {
    for (int x = ox - radius; x <= ox + radius; ++x) {
      int idx = convertGridToIndex(state.costmap, x, y);
      if (idx >= 0 && idx < (int)state.costmap.data.size()) {
        state.costmap.data[idx] = params.free_cost;
      }
    }
  }
}


// ========================================
// 4.. 3D AABB → 2D AABB (위에서 내려다본 투영)
// ========================================

void projectAABB3DTo2D(const Eigen::Vector3f &min_3d,
                       const Eigen::Vector3f &max_3d,
                       float &min_2d_x, float &min_2d_y,
                       float &max_2d_x, float &max_2d_y) {
  min_2d_x = min_3d.x();
  min_2d_y = min_3d.y();
  max_2d_x = max_3d.x();
  max_2d_y = max_3d.y();
}
// ========================================
// 5. base_link 좌표(미터) → costmap 그리드 좌표(셀) // 한점을 변환 그래서 7번안 함수에서 호출됨
// ========================================

bool convertBaselinkToGrid(const nav_msgs::OccupancyGrid &costmap,
                           float baselink_x, float baselink_y,
                           int &grid_x, int &grid_y) {
  grid_x = (int)std::floor((baselink_x - costmap.info.origin.position.x) / costmap.info.resolution);
  grid_y = (int)std::floor((baselink_y - costmap.info.origin.position.y) / costmap.info.resolution);

  if (grid_x < 0 || grid_x >= (int)costmap.info.width) return false;
  if (grid_y < 0 || grid_y >= (int)costmap.info.height) return false;
  return true;
}
// ========================================
// 6. 2D grid 좌표 → 1D 배열 인덱스
// ========================================
int convertGridToIndex(const nav_msgs::OccupancyGrid &costmap,
                       int grid_x, int grid_y) {
  if (grid_x < 0 || grid_x >= (int)costmap.info.width) return -1;
  if (grid_y < 0 || grid_y >= (int)costmap.info.height) return -1;
  return grid_y * (int)costmap.info.width + grid_x;
}
// ========================================
// 7.2D AABB(미터) → costmap grid 영역
// ========================================
void projectAABBToGrid(const nav_msgs::OccupancyGrid &costmap,
                       float min_x, float min_y,
                       float max_x, float max_y,
                       int &grid_x0, int &grid_y0,
                       int &grid_x1, int &grid_y1,
                       float inflation_radius) {
  if (min_x > max_x) std::swap(min_x, max_x);
  if (min_y > max_y) std::swap(min_y, max_y);

  min_x -= inflation_radius;
  min_y -= inflation_radius;
  max_x += inflation_radius;
  max_y += inflation_radius;

  if (!convertBaselinkToGrid(costmap, min_x, min_y, grid_x0, grid_y0)) {
    grid_x0 = (int)std::floor((min_x - costmap.info.origin.position.x) / costmap.info.resolution);
    grid_y0 = (int)std::floor((min_y - costmap.info.origin.position.y) / costmap.info.resolution);
  }

  if (!convertBaselinkToGrid(costmap, max_x, max_y, grid_x1, grid_y1)) {
    grid_x1 = (int)std::floor((max_x - costmap.info.origin.position.x) / costmap.info.resolution);
    grid_y1 = (int)std::floor((max_y - costmap.info.origin.position.y) / costmap.info.resolution);
  }

  if (grid_x0 > grid_x1) std::swap(grid_x0, grid_x1);
  if (grid_y0 > grid_y1) std::swap(grid_y0, grid_y1);

  grid_x0 = clampToRange(grid_x0, 0, (int)costmap.info.width - 1);
  grid_x1 = clampToRange(grid_x1, 0, (int)costmap.info.width - 1);
  grid_y0 = clampToRange(grid_y0, 0, (int)costmap.info.height - 1);
  grid_y1 = clampToRange(grid_y1, 0, (int)costmap.info.height - 1);
}
// ========================================
// 8. costmap 셀 하나에 장애물 cost 기록
// ========================================

void fillCostAtIndex(nav_msgs::OccupancyGrid &costmap,
                     int idx,
                     int8_t cost_value) {
  if (idx < 0 || idx >= (int)costmap.data.size()) return;
  costmap.data[idx] = std::max(costmap.data[idx], cost_value);
}
