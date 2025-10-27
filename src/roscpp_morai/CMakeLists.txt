cmake_minimum_required(VERSION 3.0.2)
project(roscpp_morai)

## -----------------------------------------------------------------------------
## Catkin deps
## -----------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  morai_msgs
  rosbag
  tf
)

## -----------------------------------------------------------------------------
## Geographic(lib) – config 파일이 없는 환경 대비: 수동 탐색
## (패키지명: libgeographic-dev / libGeographic.so)
## -----------------------------------------------------------------------------
find_path(GEOGRAPHIC_INCLUDE_DIR GeographicLib/LocalCartesian.hpp)
find_library(GEOGRAPHIC_LIBRARY NAMES Geographic GeographicLib)

if(NOT GEOGRAPHIC_INCLUDE_DIR OR NOT GEOGRAPHIC_LIBRARY)
  message(FATAL_ERROR
    "Geographic(lib) not found. Install it first:\n"
    "  sudo apt-get update && sudo apt-get install -y libgeographic-dev")
endif()

## -----------------------------------------------------------------------------
## Catkin package
## -----------------------------------------------------------------------------
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GEOGRAPHIC_INCLUDE_DIR}   # ← 헤더 경로(예: /usr/include)
)

## -----------------------------------------------------------------------------
## Executables
## -----------------------------------------------------------------------------
# add_executable(ego_path_recorder_bag_kante src/ego_path_recorder_bag_kante.cpp)
# target_compile_features(ego_path_recorder_bag_kante PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder_bag_kante ${catkin_LIBRARIES})

# add_executable(ego_path_recorder_csv_kante src/ego_path_recorder_csv_kante.cpp)
# target_compile_features(ego_path_recorder_csv_kante PRIVATE cxx_std_11)
# target_link_libraries(ego_path_recorder_csv_kante ${catkin_LIBRARIES})

add_executable(ego_path_recorder src/ego_path_recorder.cpp)
target_compile_features(ego_path_recorder PRIVATE cxx_std_11)
target_link_libraries(ego_path_recorder ${catkin_LIBRARIES})

add_executable(global_path_pub_kante src/global_path_pub_kante.cpp)
target_compile_features(global_path_pub_kante PRIVATE cxx_std_11)
target_link_libraries(global_path_pub_kante ${catkin_LIBRARIES})

add_executable(local_path_pub_kante src/local_path_pub_kante.cpp)
target_compile_features(local_path_pub_kante PRIVATE cxx_std_11)
target_link_libraries(local_path_pub_kante ${catkin_LIBRARIES})


add_executable(pp_1026 src/pp_1026.cpp)
target_compile_features(pp_1026 PRIVATE cxx_std_11)
# Geographic(lib) 는 이 타깃(pp_1026)에서만 필요
target_link_libraries(pp_1026 ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})

add_executable(path_visualizer src/path_visualizer.cpp)
target_link_libraries(path_visualizer ${catkin_LIBRARIES} Geographic)
add_dependencies(path_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
