#ifndef ROI_VOXEL_HPP
#define ROI_VOXEL_HPP
#include <Global/Global.hpp>


void FilterHeight(Lidar& st_Lidar);
void FilterRange(Lidar& st_Lidar);

void Passthrough(Lidar& st_Lidar);
void Voxel(Lidar& st_Lidar);


#endif // ROI_VOXEL_H
