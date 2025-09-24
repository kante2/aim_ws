cmake_minimum_required(VERSION 3.0.2)
project(roscpp_morai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  morai_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ego_path_recorder src/ego_path_recorder.cpp)
target_compile_features(ego_path_recorder PRIVATE cxx_std_11)
target_link_libraries(ego_path_recorder ${catkin_LIBRARIES})

add_executable(global_path_publisher src/global_path_publisher.cpp)
target_compile_features(global_path_publisher PRIVATE cxx_std_11)
target_link_libraries(global_path_publisher ${catkin_LIBRARIES})

add_executable(local_path_maker src/local_path_maker.cpp)
target_compile_features(local_path_maker PRIVATE cxx_std_11)
target_link_libraries(local_path_maker ${catkin_LIBRARIES})

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
target_compile_features(pure_pursuit_controller PRIVATE cxx_std_11)
target_link_libraries(pure_pursuit_controller ${catkin_LIBRARIES})
