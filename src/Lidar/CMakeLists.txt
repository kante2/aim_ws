cmake_minimum_required(VERSION 3.0.2)
project(lidar)


find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  morai_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  nav_msgs
  geometry_msgs
  visualization_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar
  CATKIN_DEPENDS 
  roscpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
  pcl_ros pcl_conversions 
  laser_geometry 
  tf2 tf2_ros tf2_sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)


add_executable(
euclidean
src/euclidean.cpp
)

# add_executable(
# ransac
# src/ransac.cpp
# )

add_executable(
func_3D
src/func_3D.cpp
src/convex.cpp 
)

# add_executable(
# patch
# src/patch.cpp
# )

target_link_libraries(euclidean
  ${catkin_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

# target_link_libraries(ransac
#   ${catkin_LIBRARIES}
#   ${PCL_INCLUDE_DIRS}
# )

target_link_libraries(func_3D
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# target_link_libraries(patch
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )


include_directories(include 
  ${catkin_INCLUDE_DIRS}
)
