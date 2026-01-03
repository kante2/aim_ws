#ifndef LIDAR_COSTMAP_UTILS_H
#define LIDAR_COSTMAP_UTILS_H

#include <vector>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>

namespace costmap_utils {

// ============================================================================
// Coordinate Conversion Functions
// ============================================================================

/**
 * @brief Convert world coordinates to grid coordinates
 * @param x World X coordinate
 * @param y World Y coordinate
 * @param grid_x Output grid X coordinate
 * @param grid_y Output grid Y coordinate
 * @param resolution Cell resolution in meters
 * @param map_width Total map width in meters
 * @param map_height Total map height in meters
 */
inline void worldToGrid(float x, float y, int& grid_x, int& grid_y,
                       float resolution, float map_width, float map_height) {
    grid_x = (int)((x + map_width / 2.0) / resolution);
    grid_y = (int)((y + map_height / 2.0) / resolution);
}

/**
 * @brief Convert grid coordinates to world coordinates
 * @param grid_x Grid X coordinate
 * @param grid_y Grid Y coordinate
 * @param x Output world X coordinate
 * @param y Output world Y coordinate
 * @param resolution Cell resolution in meters
 * @param map_width Total map width in meters
 * @param map_height Total map height in meters
 */
inline void gridToWorld(int grid_x, int grid_y, float& x, float& y,
                       float resolution, float map_width, float map_height) {
    x = grid_x * resolution - map_width / 2.0;
    y = grid_y * resolution - map_height / 2.0;
}

// ============================================================================
// Grid Index Functions
// ============================================================================

/**
 * @brief Get linear index from grid coordinates
 * @param grid_x Grid X coordinate
 * @param grid_y Grid Y coordinate
 * @param width Grid width
 * @return Linear index in the costmap data array, or -1 if out of bounds
 */
inline int getGridIndex(int grid_x, int grid_y, int width) {
    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= width) {
        return -1;
    }
    return grid_y * width + grid_x;
}

// ============================================================================
// Inflation/Dilation Functions
// ============================================================================

/**
 * @brief Inflate obstacles in the costmap (expand obstacle regions)
 * @param costmap Input/output costmap
 * @param inflation_radius Inflation radius in cells
 * @param obstacle_threshold Cost value above which a cell is considered an obstacle
 */
void inflateObstacles(std::vector<int8_t>& costmap,
                      int width, int height,
                      int inflation_radius,
                      int8_t obstacle_threshold);

/**
 * @brief Apply cost decay around obstacles (gradient)
 * @param costmap Input/output costmap
 * @param decay_radius Radius over which to apply cost decay
 * @param max_cost Maximum cost value
 */
void decayObstacleCost(std::vector<int8_t>& costmap,
                       int width, int height,
                       int decay_radius,
                       int8_t max_cost);

// ============================================================================
// Smoothing Functions
// ============================================================================

/**
 * @brief Apply Gaussian blur to costmap
 * @param costmap Input/output costmap
 * @param kernel_size Size of the Gaussian kernel (should be odd)
 */
void smoothCostmap(std::vector<int8_t>& costmap,
                   int width, int height,
                   int kernel_size);

// ============================================================================
// Analysis Functions
// ============================================================================

/**
 * @brief Get costmap statistics (min, max, mean cost)
 * @param costmap Input costmap
 * @param min_cost Output minimum cost
 * @param max_cost Output maximum cost
 * @param mean_cost Output mean cost
 * @param obstacle_count Output number of obstacle cells
 */
void getCostmapStatistics(const std::vector<int8_t>& costmap,
                         int8_t& min_cost, int8_t& max_cost,
                         float& mean_cost, int& obstacle_count,
                         int8_t obstacle_threshold);

/**
 * @brief Check if a path is clear (no obstacles)
 * @param costmap Input costmap
 * @param start_x Start grid X
 * @param start_y Start grid Y
 * @param end_x End grid X
 * @param end_y End grid Y
 * @param threshold Cost threshold for obstacle detection
 * @return True if path is clear, false if obstacle exists
 */
bool isPathClear(const std::vector<int8_t>& costmap,
                 int width,
                 int start_x, int start_y,
                 int end_x, int end_y,
                 int8_t threshold);

// ============================================================================
// Visualization/Debug Functions
// ============================================================================

/**
 * @brief Create a simple text visualization of the costmap
 * @param costmap Input costmap
 * @param width Grid width
 * @param height Grid height
 * @return String representation of the costmap
 */
std::string visualizeCostmap(const std::vector<int8_t>& costmap,
                            int width, int height);

} // namespace costmap_utils

#endif // LIDAR_COSTMAP_UTILS_H
