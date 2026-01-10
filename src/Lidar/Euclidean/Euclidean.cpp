#include <Lidar/Euclidean/Euclidean.hpp>

void Euclidean (Lidar &st_Lidar) 
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterrange_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclidean;
    vector<pcl::PointIndices> vec_cluster_indices;
    
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_point(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cluster_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    
    vector<LidarCluster> vec_cluster;
    LidarCluster st_LidarCluster;

    pcl::PointXYZI pcl_point;
    
    int32 i, j;
    int32 pointidx;
    int32 intensity = 0;

    if (!st_Lidar.pcl_ransac_cloud -> empty())
    {
        kdtree->setInputCloud(st_Lidar.pcl_ransac_cloud);

        euclidean.setInputCloud(st_Lidar.pcl_ransac_cloud);
        euclidean.setSearchMethod(kdtree);
        euclidean.setClusterTolerance(st_LidarParam.euclidean_tolerance);
        euclidean.setMinClusterSize(st_LidarParam.euclidean_min_size);
        euclidean.setMaxClusterSize(st_LidarParam.euclidean_max_size);
        euclidean.extract(vec_cluster_indices); 
    }
    // 여기까지가 기존 클러스터링 과정 (클러스터 안에 포인트클라우드의 인덱스가 저장)
    
    for (i = 0; i < vec_cluster_indices.size(); i++)
    {
        pcl_cluster_point.reset(new pcl::PointCloud<pcl::PointXYZI>());
        for (j = 0; j < vec_cluster_indices[i].indices.size(); j++)
        {
            pointidx = vec_cluster_indices[i].indices[j];
            pcl_point = st_Lidar.pcl_ransac_cloud->points[pointidx];
            pcl_point.intensity = intensity % 10;
            pcl_cluster_point->push_back(pcl_point);   
            
            
        }
        // *pcl_cluster_cloud += *pcl_cluster_point; 
        // 굳이 ? 모든 클러스터 점군을 한꺼번에 모은 게 시각화할 때나 필요..?

        // 클러스터별 벡터 생성. 클러스터 정보 계산은 아직 X
        st_LidarCluster.pcl_cluster_point = *pcl_cluster_point;
        vec_cluster.push_back(st_LidarCluster);
    }
    // *st_Lidar.pcl_euclidean_cloud = *pcl_cluster_cloud;
    st_Lidar.vec_cluster = vec_cluster;

}

