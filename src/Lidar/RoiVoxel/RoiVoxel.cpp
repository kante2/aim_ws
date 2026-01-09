#include <Lidar/RoiVoxel/RoiVoxel.hpp>

void FilterHeight (Lidar& st_Lidar)
{
    pcl::PassThrough<pcl::PointXYZI> filterheight;

    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    filterheight.setInputCloud(st_Lidar.pcl_input_cloud);
    filterheight.setFilterFieldName("z");
    filterheight.setFilterLimits(st_LidarParam.min_height, st_LidarParam.max_height);
    filterheight.filter(*st_Lidar.pcl_input_cloud);
}

void FilterRange (Lidar& st_Lidar)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterrange_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    *filterrange_cloud = *st_Lidar.pcl_filterheight_cloud;

    filterrange_cloud -> reserve(st_Lidar.pcl_filterheight_cloud->size());
    for (const auto &point : *(st_Lidar.pcl_filterheight_cloud))
    {
        float distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance <= st_LidarParam.lidar_range)
        {
            filterrange_cloud->push_back(point);
        }
    }
    st_Lidar.pcl_filterrange_cloud = filterrange_cloud;
}

void Passthrough (Lidar& st_Lidar)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr x_inside(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr x_outside(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_passthrough_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr passthrough_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PassThrough<pcl::PointXYZI> x_filter;
    pcl::PassThrough<pcl::PointXYZI> y_filter;

    LidarParam st_LidarParam = st_Lidar.st_LidarParam;
    *input_passthrough_cloud = *st_Lidar.pcl_filterrange_cloud;

    x_filter.setInputCloud(input_passthrough_cloud);
    x_filter.setFilterFieldName("x");
    x_filter.setFilterLimits(st_LidarParam.ego_xmin, st_LidarParam.ego_xmax);
    x_filter.setFilterLimitsNegative(false);
    x_filter.filter(*x_inside);

    x_filter.setFilterLimitsNegative(true);
    x_filter.filter(*x_outside);

    y_filter.setInputCloud(x_inside);
    y_filter.setFilterFieldName("y");
    y_filter.setFilterLimits(st_LidarParam.ego_ymin, st_LidarParam.ego_ymax);
    y_filter.setFilterLimitsNegative(true);
    y_filter.filter(*passthrough_cloud);

    *passthrough_cloud += *x_outside;
    *st_Lidar.pcl_passthrough_cloud = *passthrough_cloud;
}

void Voxel (Lidar& st_Lidar)
{
    pcl::VoxelGrid<pcl::PointXYZI> pcl_voxel_filter;
    LidarParam st_LidarParam = st_Lidar.st_LidarParam;

    pcl_voxel_filter.setInputCloud(st_Lidar.pcl_passthrough_cloud);
    pcl_voxel_filter.setLeafSize(st_LidarParam.voxel_leaf, st_LidarParam.voxel_leaf, st_LidarParam.voxel_leaf);
    pcl_voxel_filter.filter(*st_Lidar.pcl_voxel_cloud);
}