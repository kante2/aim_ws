// lidar_aabb_costmap_node.cpp (modified: TF 안정화 + frame/stamp 강제 통일)
// - PointCloud를 costmap_frame(기본 base_link)로 변환(필수)
// - costmap / marker / debug cloud 모두 frame_id를 costmap_frame으로 강제
// - TF lookup은 시뮬에서 extrapolation 줄이려고 ros::Time(0) (latest) 사용

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Dense>
#include <limits>
#include <cmath>
#include <algorithm>
#include <memory>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

// =================refactoring==================

// Config
#include "config/lidar_aabb_params.hpp"

using std::string;

// Global params (loaded from config)
static LidarAabbParams g_params;

// ------------------------------
// Publishers
// ------------------------------
ros::Publisher pub_costmap;
ros::Publisher pub_ransac;
ros::Publisher pub_cluster_all;
ros::Publisher pub_cluster_cav;
ros::Publisher pub_cav_aabb_centers;
ros::Publisher pub_cav_aabb_boxes;
ros::Publisher pub_cav_aabb_center_spheres;

// ------------------------------
// TF
// ------------------------------
std::unique_ptr<tf2_ros::Buffer> tfBufferPtr;
std::unique_ptr<tf2_ros::TransformListener> tfListenerPtr;

// Helper function for frame normalization
static inline std::string normalizeFrame(std::string f) {
  while (!f.empty() && f.front() == '/') f.erase(f.begin());
  return f;
}

// Detection struct
struct Detection {
  int id = -1;
  ros::Time stamp;

  Eigen::Vector3f centroid;
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;
  Eigen::Vector3f size;

  int num_points = 0;
  float range = 0.0f;
};

// init costmap (must be before visualize which uses worldToGrid)
#include "costmap/costmap_init.hpp"

// visualize
#include "visualize/lidar_aabb_visualize.hpp"

// pointcloud processing
#include "pointcloud_processing/pointcloud_processing.hpp"

// callback
#include "lidar_callback/lidar_callback.hpp"
// ===============================================


// ------------------------------
// Main callback
// ------------------------------


// ------------------------------
// main
// ------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_aabb_costmap_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Load all parameters from config
  loadParams(pnh, g_params);
  g_params.costmap_frame = normalizeFrame(g_params.costmap_frame);

  // TF init (cache time 늘려서 시뮬에서 lookup 여유)
  tfBufferPtr.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
  tfListenerPtr.reset(new tf2_ros::TransformListener(*tfBufferPtr));

  // publishers
  pub_costmap = nh.advertise<nav_msgs::OccupancyGrid>(g_params.costmap_topic, 1);
  pub_ransac = nh.advertise<sensor_msgs::PointCloud2>("/ransac", 1);
  pub_cluster_all = nh.advertise<sensor_msgs::PointCloud2>("/cluster", 1);
  pub_cluster_cav = nh.advertise<sensor_msgs::PointCloud2>("/cluster_cav", 1);

  pub_cav_aabb_centers = nh.advertise<geometry_msgs::PoseArray>("/cav_aabb_centers", 1);
  pub_cav_aabb_boxes = nh.advertise<visualization_msgs::MarkerArray>("/cav_aabb_boxes", 1);
  pub_cav_aabb_center_spheres = nh.advertise<visualization_msgs::MarkerArray>("/cav_aabb_center_spheres", 1);

  // subscriber
  ros::Subscriber sub = nh.subscribe(g_params.lidar_topic, 1, lidarCallback);

  ROS_INFO("lidar_aabb_costmap_node started");
  ROS_INFO(" - lidar_topic: %s", g_params.lidar_topic.c_str());
  ROS_INFO(" - costmap_topic: %s", g_params.costmap_topic.c_str());
  ROS_INFO(" - use_tf: %s, require_tf: %s, costmap_frame: %s",
           g_params.use_tf ? "true":"false",
           g_params.require_tf ? "true":"false",
           g_params.costmap_frame.c_str());
  ROS_INFO(" - map: %.1fm x %.1fm, res=%.2fm", g_params.map_width, g_params.map_height, g_params.map_resolution);
  ROS_INFO(" - height: [%.2f, %.2f], range=%.1f, voxel=%.2f", g_params.min_height, g_params.max_height, g_params.lidar_range, g_params.voxel_leaf);
  ROS_INFO(" - cluster tol=%.2f, min=%d, max=%d", g_params.cluster_tolerance, g_params.cluster_min_size, g_params.cluster_max_size);
  ROS_INFO(" - cav dims dx[%.2f,%.2f], dy[%.2f,%.2f], dz[%.2f,%.2f], min_pts=%d",
           g_params.cav_dx_min,g_params.cav_dx_max,g_params.cav_dy_min,g_params.cav_dy_max,g_params.cav_dz_min,g_params.cav_dz_max,g_params.cav_min_points);

  ros::spin();
  return 0;
}
