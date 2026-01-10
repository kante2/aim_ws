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
#include <vector>
#include <string> 

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

// =================refactoring==================


using namespace std;

// ========================= Type ========================= //
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef uint64_t uint64;
typedef short int16;
typedef int int32;
typedef float float32;
typedef double float64;
typedef char char8;

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

// =========================================
// convex hull
// =========================================
struct Point2D 
{
    double x;
    double y;
};

struct Rectangle 
{
    Eigen::Vector2d corners[4];
    double heading;
    double score;
};

struct LidarParam
{
    // 전역 변수만 선언해야 (파라미터 값 할당x -> 함수 완성하면 삭제하기)

    // lidar prefilter
    float min_height = -3.0f;
    float max_height = 5.0f;
    float lidar_range = 20.0f; //25.0f

    // ego 제거 ROI (passthrough) -> 차체 x, y 값에 따라 세밀한 파라미터 조정 필요
    float ego_xmin = 2.0f; // |x|<=2 and |y|<=2 영역 제거 ==> 자차 차체/센서 브라켓/지붕 포인트 때문에 생기는 “가짜 클러스터” 방지
    float ego_xmax = 2.0f; 
    float ego_ymin = 2.0f;
    float ego_ymax = 2.0f; 

    // voxel downsample
    float voxel_leaf = 0.10f;

    // RANSAC
    float ransac_dist_thresh = 0.50f; // 평면에서 0.30m 이내면 지면(inlier)로 간주
    float ransac_eps_angle_deg = 30.0f; // 지면 평면의 법선이 z축에서 10도 이내면 지면으로 인정 ==> 약간 기울어진 도로도 지면으로 인식 / 경사로/언덕이 있으면 조금 더 키워야 할 수도 있음(예: 15~20)
    int ransac_max_iter = 200; // RANSAC 반복 횟수 (크면 안정적이지만 느려짐)

    // clustering
    float euclidean_tolerance = 0.2f; // 점과 점이 0.4m 이내면 같은 클러스터로 연결
    int euclidean_min_size = 5;
    int euclidean_max_size = 2000; // 너무 작은 덩어리(노이즈) 제거 / 너무 큰 덩어리(벽/지형) 제거

    //

};

// Lshapefitting 사용 struct
struct LidarCluster
{
    pcl::PointCloud<pcl::PointXYZI> pcl_cluster_point; // 포인트 클라우드를 담은 클러스터 1개

    double theta = 0.0; // 최적 angle
    
    double rotate_rect_min_x = 0.0; // 직사각형을 축정렬했을 때 나오는 꼭짓점
    double rotate_rect_max_x = 0.0;
    double rotate_rect_min_y = 0.0;
    double rotate_rect_max_y = 0.0;

    // Point2D corner[4]; // 이건 왜 필요?

};

struct Lidar
{
    LidarParam st_LidarParam;
    LidarCluster st_LidarCluster;
    
    // PointCloud2
    sensor_msgs::PointCloud2::Ptr input_cloud_msg;
    sensor_msgs::PointCloud2::Ptr output_cloud_msg;

    // PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output_cloud; // 최종 출력 클라우드

    // ========================
    // vector 객체 정보
    // ========================

    vector<LidarCluster> vec_cluster; // 포인트 클라우드 클러스터 묶음

    // ========================
    // 중간 처리 단계별 PointCloud
    // ========================

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filterheight_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_filterrange_cloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_passthrough_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_voxel_cloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ransac_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_euclidean_cloud;

    
    // Constructor
    Lidar()
    {
        input_cloud_msg.reset(new sensor_msgs::PointCloud2);
        output_cloud_msg.reset(new sensor_msgs::PointCloud2);

        pcl_input_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl_output_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        pcl_filterheight_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl_filterrange_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        pcl_passthrough_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl_voxel_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        pcl_ransac_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl_euclidean_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

    }
};

