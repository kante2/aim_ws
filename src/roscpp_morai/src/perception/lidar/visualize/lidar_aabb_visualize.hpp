#ifndef LIDAR_AABB_VISUALIZE_HPP
#define LIDAR_AABB_VISUALIZE_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <cmath>

// Helper: clamp int
static inline int clampi(int v, int lo, int hi) {
  return std::max(lo, std::min(v, hi));
}

// Paint AABB on costmap
static void paintAABB(nav_msgs::OccupancyGrid &cm,
                      float min_x, float min_y, float max_x, float max_y,
                      int8_t value, float inflation = 0.0f)
{
  if (min_x > max_x) std::swap(min_x, max_x);
  if (min_y > max_y) std::swap(min_y, max_y);

  // inflation 적용
  min_x -= inflation; min_y -= inflation;
  max_x += inflation; max_y += inflation;

  int gx0, gy0, gx1, gy1;
  if (!worldToGrid(cm, min_x, min_y, gx0, gy0)) {
    gx0 = (int)std::floor((min_x - cm.info.origin.position.x) / cm.info.resolution);
    gy0 = (int)std::floor((min_y - cm.info.origin.position.y) / cm.info.resolution);
  }
  if (!worldToGrid(cm, max_x, max_y, gx1, gy1)) {
    gx1 = (int)std::floor((max_x - cm.info.origin.position.x) / cm.info.resolution);
    gy1 = (int)std::floor((max_y - cm.info.origin.position.y) / cm.info.resolution);
  }

  if (gx0 > gx1) std::swap(gx0, gx1);
  if (gy0 > gy1) std::swap(gy0, gy1);

  gx0 = clampi(gx0, 0, (int)cm.info.width  - 1);
  gx1 = clampi(gx1, 0, (int)cm.info.width  - 1);
  gy0 = clampi(gy0, 0, (int)cm.info.height - 1);
  gy1 = clampi(gy1, 0, (int)cm.info.height - 1);

  const int w = (int)cm.info.width;
  for (int y = gy0; y <= gy1; ++y) {
    for (int x = gx0; x <= gx1; ++x) {
      int idx = y * w + x;
      if (idx >= 0 && idx < (int)cm.data.size()) {
        cm.data[idx] = std::max(cm.data[idx], value);
      }
    }
  }
}

#endif // LIDAR_AABB_VISUALIZE_HPP