cmake_minimum_required(VERSION 2.8.3)
project(lattice_planner_morai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  pcl_conversions
  pcl_ros
)

## 헤더 파일 디렉토리
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## catkin 패키지 정의
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs pcl_conversions pcl_ros
)

## Lidar Costmap Generator 실행파일 빌드
add_executable(lidar_costmap_generator src/lidar_costmap_generator.cpp)
target_link_libraries(lidar_costmap_generator ${catkin_LIBRARIES})

## 설치
install(TARGETS lidar_costmap_generator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)