cmake_minimum_required(VERSION 3.0.2)
project(parkie_morai)

## 선택: C++ 표준 지정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  morai_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    morai_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    tf2_ros
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${PROJECT_SOURCE_DIR}/include   # include 폴더가 있다면 주석 해제
)

add_executable(convert_lidar src/convert_lidar.cpp)
add_dependencies(convert_lidar
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(convert_lidar ${catkin_LIBRARIES})
