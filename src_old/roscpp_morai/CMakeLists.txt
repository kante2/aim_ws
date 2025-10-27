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


add_executable(ego_path_recorder_kante src/ego_path_recorder_kante.cpp)
target_compile_features(ego_path_recorder_kante PRIVATE cxx_std_11)
target_link_libraries(ego_path_recorder_kante ${catkin_LIBRARIES})


add_executable(global_path_pub_kante src/global_path_pub_kante.cpp)
target_compile_features(global_path_pub_kante PRIVATE cxx_std_11)
target_link_libraries(global_path_pub_kante ${catkin_LIBRARIES})

add_executable(local_path_pub_kante src/local_path_pub_kante.cpp)
target_compile_features(local_path_pub_kante PRIVATE cxx_std_11)
target_link_libraries(local_path_pub_kante ${catkin_LIBRARIES})

add_executable(pure_pursuit_controller_kante src/pure_pursuit_controller_kante.cpp)
target_compile_features(pure_pursuit_controller_kante PRIVATE cxx_std_11)
target_link_libraries(pure_pursuit_controller_kante ${catkin_LIBRARIES})