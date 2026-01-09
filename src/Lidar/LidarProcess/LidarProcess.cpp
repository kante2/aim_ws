#include <Lidar/LidarProcess/LidarProcess.hpp>

void LidarProcess() //callback
{
    FilterHeight(st_Lidar);
    FilterRange(st_Lidar);
    Passthrough(st_Lidar);
    Voxel(st_Lidar);
    Ransac(st_Lidar);
    Euclidean(st_Lidar);
    LShapeFitting(st_Lidar);
}