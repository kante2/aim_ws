#include <Lidar/Euclidean/Euclidean.hpp>

void Euclidean (Lidar &st_Lidar) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterrange_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    kdtree->setInputCloud(st_Lidar.pcl_ransac_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setInputCloud(st_Lidar.pcl_ransac_cloud);
    ec.setSearchMethod(kdtree);
    ec.setClusterTolerance(st_LidarParam.euclidean_tolerance);
    ec.setMinClusterSize(st_LidarParam.euclidean_min_size);
    ec.setMaxClusterSize(st_LidarParam.euclidean_max_size);

    vector<pcl::PointIndices> indices;
    ec.extract(indices);
}

