cmake_minimum_required(VERSION 3.0.2)
project(roscpp_morai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  morai_msgs
  rosbag
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# add_executable(ego_path_recorder_bag_kante src/ego_path_recorder_bag_kante.cpp)
# target_compile_features(ego_path_recorder_bag_kante PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder_bag_kante ${catkin_LIBRARIES})

# add_executable(ego_path_recorder_csv_kante src/ego_path_recorder_csv_kante.cpp)
# target_compile_features(ego_path_recorder_csv_kante PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder_csv_kante ${catkin_LIBRARIES})

add_executable(pp_from_csv_node src/pp_from_csv_node.cpp)
target_compile_features(pp_from_csv_node PRIVATE cxx_std_11)
target_link_libraries(pp_from_csv_node ${catkin_LIBRARIES})


add_executable(ego_path_recorder src/ego_path_recorder.cpp)
target_compile_features(ego_path_recorder PRIVATE cxx_std_11)
target_link_libraries(ego_path_recorder ${catkin_LIBRARIES})


add_executable(global_path_pub_kante src/global_path_pub_kante.cpp)
target_compile_features(global_path_pub_kante PRIVATE cxx_std_11)
target_link_libraries(global_path_pub_kante ${catkin_LIBRARIES})

add_executable(local_path_pub_kante src/local_path_pub_kante.cpp)
target_compile_features(local_path_pub_kante PRIVATE cxx_std_11)
target_link_libraries(local_path_pub_kante ${catkin_LIBRARIES})

add_executable(pure_pursuit_controller_kante src/pure_pursuit_controller_kante.cpp)
target_compile_features(pure_pursuit_controller_kante PRIVATE cxx_std_11)
target_link_libraries(pure_pursuit_controller_kante ${catkin_LIBRARIES})