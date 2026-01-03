#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>
#include <queue>

// 전역 변수
ros::Publisher costmap_pub;
nav_msgs::OccupancyGrid current_costmap;
sensor_msgs::PointCloud2 latest_pointcloud;
bool has_pointcloud = false;
visualization_msgs::MarkerArray latest_markers;

// 파라미터
float map_resolution = 0.05;  // 5cm per cell
float map_width = 50.0;       // 50m width
float map_height = 50.0;      // 50m height
float max_height = 0.5;       // Max height to consider (m)
float min_height = -0.5;      // Min height to consider (m)
float lidar_range = 25.0;     // Max lidar range (m)
int obstacle_threshold = 10;  // Obstacle cost threshold
int unknown_cost = 0;         // Unknown cell cost
int obstacle_cost = 100;      // Obstacle cell cost

// 좌표 변환: world to grid
void worldToGrid(float x, float y, int& grid_x, int& grid_y) {
    grid_x = (int)((x + map_width / 2.0) / map_resolution);
    grid_y = (int)((y + map_height / 2.0) / map_resolution);
}

// 좌표 변환: grid to world
void gridToWorld(int grid_x, int grid_y, float& x, float& y) {
    x = grid_x * map_resolution - map_width / 2.0;
    y = grid_y * map_resolution - map_height / 2.0;
}

// Grid index 계산
int getGridIndex(int grid_x, int grid_y, int width) {
    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= width) {
        return -1;
    }
    return grid_y * width + grid_x;
}

// Bresenham line drawing algorithm - ray tracing
void rayTracing(std::vector<int8_t>& costmap_data, int start_x, int start_y, 
                int end_x, int end_y, int width) {
    int dx = abs(end_x - start_x);
    int dy = abs(end_y - start_y);
    int sx = (start_x < end_x) ? 1 : -1;
    int sy = (start_y < end_y) ? 1 : -1;
    int err = dx - dy;
    
    int current_x = start_x;
    int current_y = start_y;
    
    // Ray path를 free space로 표시 (0으로 설정)
    while (true) {
        int idx = getGridIndex(current_x, current_y, width);
        if (idx >= 0) {
            // Free space는 50으로 설정하거나 유지
            if (costmap_data[idx] < obstacle_threshold) {
                costmap_data[idx] = 0;
            }
        }
        
        if (current_x == end_x && current_y == end_y) {
            break;
        }
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            current_x += sx;
        }
        if (e2 < dx) {
            err += dx;
            current_y += sy;
        }
    }
    
    // 마지막 포인트는 obstacle로 표시
    int final_idx = getGridIndex(end_x, end_y, width);
    if (final_idx >= 0) {
        costmap_data[final_idx] = obstacle_cost;
    }
}

// PointCloud2 데이터로부터 costmap 생성
nav_msgs::OccupancyGrid generateCostmapFromPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    
    // OccupancyGrid 초기화
    nav_msgs::OccupancyGrid costmap;
    costmap.header.frame_id = cloud_msg->header.frame_id;
    costmap.header.stamp = cloud_msg->header.stamp;
    
    int grid_width = (int)(map_width / map_resolution);
    int grid_height = (int)(map_height / map_resolution);
    
    costmap.info.resolution = map_resolution;
    costmap.info.width = grid_width;
    costmap.info.height = grid_height;
    costmap.info.origin.position.x = -map_width / 2.0;
    costmap.info.origin.position.y = -map_height / 2.0;
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.w = 1.0;
    
    // Costmap data 초기화 (모두 unknown으로 설정)
    costmap.data.assign(grid_width * grid_height, unknown_cost);
    
    // PointCloud2를 PCL 형식으로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Lidar origin (sensor location) - 보통 (0, 0, 0)
    int origin_x, origin_y;
    worldToGrid(0.0, 0.0, origin_x, origin_y);
    
    ROS_DEBUG("Origin grid: (%d, %d)", origin_x, origin_y);
    
    // 각 point에 대해 처리
    for (const auto& point : cloud->points) {
        // 높이 필터링
        if (point.z > max_height || point.z < min_height) {
            continue;
        }
        
        // 거리 필터링
        float distance = sqrt(point.x * point.x + point.y * point.y);
        if (distance > lidar_range) {
            continue;
        }
        
        // World to grid 변환
        int grid_x, grid_y;
        worldToGrid(point.x, point.y, grid_x, grid_y);
        
        // Grid 범위 확인
        int idx = getGridIndex(grid_x, grid_y, grid_width);
        if (idx < 0) {
            continue;
        }
        
        // Ray casting을 이용한 occupancy grid 업데이트
        rayTracing(costmap.data, origin_x, origin_y, grid_x, grid_y, grid_width);
    }
    
    // Origin 주변은 free로 설정
    int free_radius = 3;
    for (int x = origin_x - free_radius; x <= origin_x + free_radius; x++) {
        for (int y = origin_y - free_radius; y <= origin_y + free_radius; y++) {
            int idx = getGridIndex(x, y, grid_width);
            if (idx >= 0) {
                costmap.data[idx] = 0;
            }
        }
    }
    
    return costmap;
}

// Voxel grid를 이용한 포인트 클라우드 다운샘플링
void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud);
}

// Height 기반 포인트 클라우드 필터링
void filterPointCloudByHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               float min_z, float max_z) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud);
}

// AABB 박스를 Costmap에 마킹
void markAABBBoxOnCostmap(nav_msgs::OccupancyGrid& costmap,
                          float min_x, float min_y, float max_x, float max_y,
                          int8_t cost_value) {
    int grid_width = costmap.info.width;
    int grid_height = costmap.info.height;
    
    // World to Grid 변환
    int grid_min_x = (int)((min_x + map_width / 2.0) / map_resolution);
    int grid_min_y = (int)((min_y + map_height / 2.0) / map_resolution);
    int grid_max_x = (int)((max_x + map_width / 2.0) / map_resolution);
    int grid_max_y = (int)((max_y + map_height / 2.0) / map_resolution);
    
    // Grid 범위 클리핑
    grid_min_x = std::max(0, std::min(grid_min_x, grid_width - 1));
    grid_min_y = std::max(0, std::min(grid_min_y, grid_height - 1));
    grid_max_x = std::max(0, std::min(grid_max_x, grid_width - 1));
    grid_max_y = std::max(0, std::min(grid_max_y, grid_height - 1));
    
    // AABB 박스 영역에 cost 값 적용
    for (int y = grid_min_y; y <= grid_max_y; y++) {
        for (int x = grid_min_x; x <= grid_max_x; x++) {
            int idx = y * grid_width + x;
            if (idx >= 0 && idx < (int)costmap.data.size()) {
                costmap.data[idx] = std::max(costmap.data[idx], cost_value);
            }
        }
    }
}

// AABB Marker 배열로부터 Costmap 업데이트
void updateCostmapWithAABB(nav_msgs::OccupancyGrid& costmap,
                           const visualization_msgs::MarkerArray& markers) {
    for (const auto& marker : markers.markers) {
        if (marker.type != visualization_msgs::Marker::CUBE) {
            continue;
        }
        
        // 마커의 위치와 스케일로부터 AABB 박스 계산
        float center_x = marker.pose.position.x;
        float center_y = marker.pose.position.y;
        float scale_x = marker.scale.x;
        float scale_y = marker.scale.y;
        
        float min_x = center_x - scale_x / 2.0;
        float min_y = center_y - scale_y / 2.0;
        float max_x = center_x + scale_x / 2.0;
        float max_y = center_y + scale_y / 2.0;
        
        // 마커의 색상으로 cost 값 결정
        int8_t cost_value = 100;  // 기본값
        if (marker.color.r > 0.5 && marker.color.g < 0.5) {
            cost_value = 100;  // 빨강 = 매우 높은 비용 (미감지 차량)
        } else if (marker.color.g > 0.5 && marker.color.r < 0.5) {
            cost_value = 75;   // 초록 = 높은 비용 (감지된 차량) ✨ 수정됨
        }
        
        markAABBBoxOnCostmap(costmap, min_x, min_y, max_x, max_y, cost_value);
    }
}

// Lidar 포인트클라우드 콜백
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!ros::ok()) return;
    
    ROS_DEBUG("Received point cloud with %d points", msg->width * msg->height);
    
    // 포인트클라우드 다운샘플링
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // 높이 필터링
    filterPointCloudByHeight(cloud, min_height, max_height);
    
    // 다운샘플링 (계산 효율성)
    downsamplePointCloud(cloud, map_resolution * 2);
    
    // Costmap 생성
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud, filtered_msg);
    filtered_msg.header = msg->header;
    
    // filtered_msg를 ConstPtr로 변환
    sensor_msgs::PointCloud2::ConstPtr filtered_msg_ptr = 
        boost::make_shared<sensor_msgs::PointCloud2>(filtered_msg);
    
    nav_msgs::OccupancyGrid costmap = generateCostmapFromPointCloud(filtered_msg_ptr);
    
    // AABB 박스 적용 (만약 마커가 있다면)
    if (!latest_markers.markers.empty()) {
        updateCostmapWithAABB(costmap, latest_markers);
    }
    
    // Costmap 퍼블리시
    costmap_pub.publish(costmap);
    
    ROS_DEBUG("Published costmap with resolution %.2f m/cell", map_resolution);
}

// AABB Marker 콜백
void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
    latest_markers = *msg;
    ROS_DEBUG("Received %zu markers for AABB boxes", msg->markers.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_costmap_generator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    // 파라미터 로드
    pnh.param("map_resolution", map_resolution, 0.05f);
    pnh.param("map_width", map_width, 50.0f);
    pnh.param("map_height", map_height, 50.0f);
    pnh.param("max_height", max_height, 0.5f);
    pnh.param("min_height", min_height, -0.5f);
    pnh.param("lidar_range", lidar_range, 25.0f);
    pnh.param("obstacle_cost", obstacle_cost, 100);
    pnh.param("obstacle_threshold", obstacle_threshold, 10);
    
    ROS_INFO("Lidar Costmap Generator started");
    ROS_INFO("Resolution: %.2f m/cell", map_resolution);
    ROS_INFO("Map size: %.1f x %.1f m", map_width, map_height);
    ROS_INFO("Height range: [%.2f, %.2f] m", min_height, max_height);
    ROS_INFO("Lidar range: %.1f m", lidar_range);
    
    // Subscriber 및 Publisher
    // MORAI 시뮬레이터 또는 실제 LiDAR 토픽 구독
    std::string lidar_topic = "/lidar3D";  // MORAI default topic
    pnh.param("lidar_topic", lidar_topic, lidar_topic);
    
    std::string marker_topic = "/pub_cav_aabb_boxes";  // AABB marker 토픽
    pnh.param("marker_topic", marker_topic, marker_topic);
    
    ros::Subscriber lidar_sub = nh.subscribe(lidar_topic, 10, lidarCallback);
    ros::Subscriber marker_sub = nh.subscribe(marker_topic, 10, markerCallback);
    costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap_from_lidar", 10);
    
    ROS_INFO("Subscribing to lidar topic: %s", lidar_topic.c_str());
    ROS_INFO("Subscribing to marker topic: %s", marker_topic.c_str());
    
    // 초기 costmap 생성
    int grid_width = (int)(map_width / map_resolution);
    int grid_height = (int)(map_height / map_resolution);
    
    current_costmap.header.frame_id = "base_link";
    current_costmap.info.resolution = map_resolution;
    current_costmap.info.width = grid_width;
    current_costmap.info.height = grid_height;
    current_costmap.info.origin.position.x = -map_width / 2.0;
    current_costmap.info.origin.position.y = -map_height / 2.0;
    current_costmap.data.assign(grid_width * grid_height, unknown_cost);
    
    ros::spin();
    
    return 0;
}
