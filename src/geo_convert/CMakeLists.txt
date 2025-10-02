cmake_minimum_required(VERSION 3.0.2)
project(geo_convert)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  morai_msgs
  rosbag
  roscpp_serialization
)

# GeographicLib는 system package이므로 따로 find_package 안 써도 됨
# (단, 링크는 반드시 필요)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 실행 파일 지정
add_executable(geo_converter src/geo_converter.cpp)
target_compile_features(geo_converter PRIVATE cxx_std_11)

add_executable(geo_converter_2 src/geo_converter_2.cpp)
target_compile_features(geo_converter_2 PRIVATE cxx_std_11)

add_executable(coord_convert_ego_node src/geo_converter_ego.cpp)
target_link_libraries(coord_convert_ego_node ${catkin_LIBRARIES} Geographic)

add_executable(coord_convert_gps_node src/geo_converter_gps.cpp)
target_link_libraries(coord_convert_gps_node ${catkin_LIBRARIES} Geographic)


## 링크 반드시 지정!
target_link_libraries(geo_converter
  ${catkin_LIBRARIES}
  Geographic
)

target_link_libraries(geo_converter_2
  ${catkin_LIBRARIES}
  Geographic
)
