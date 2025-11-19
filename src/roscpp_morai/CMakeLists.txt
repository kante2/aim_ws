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

# -------------------------------------------------------------------------------
# ONLY PRUE PURSUIT - KANTE UPLODADED / 1026 ~ 1031
add_executable(pp_1026 src/control/pp_1026.cpp)
target_compile_features(pp_1026 PRIVATE cxx_std_11)
# Geographic(lib) 는 이 타깃(pp_1026)에서만 필요
target_link_libraries(pp_1026 ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})

add_executable(pp_1026_noclass_test src/control/pp_1026_noclass_test.cpp)
target_compile_features(pp_1026_noclass_test PRIVATE cxx_std_11)
# Geographic(lib) 는 이 타깃(pp_1026_noclass_test)에서만 필요
target_link_libraries(pp_1026_noclass_test ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})


add_executable(path_visualizer src/path_visualizer.cpp)
target_link_libraries(path_visualizer ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})
add_dependencies(path_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# --------------------------------------------------------------------------------
# ver1 -> PURE PURSUIT + OBSTACLE_AVOIDANCE ALL _ KANTE UPLODADED

# (경로 수정) pp_1102 는 decision 폴더에 존재
add_executable(pp_1102 src/decision/pp_1102.cpp)
target_compile_features(pp_1102 PRIVATE cxx_std_11)
target_link_libraries(pp_1102 ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})
# OR.
add_executable(pp_local_path_1102 src/pp_local_path_1102.cpp)
target_link_libraries(pp_local_path_1102 ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})
add_dependencies(pp_local_path_1102 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# --------------------------------------------------------------------------------
# ver2 -> PURE PURSUIT + OBSTACLE_AVOIDANCE _ KANTE UPLODADED / 1102 ~
# pp_with_obstacle.launch로 실행이 가능하다.! 
# (경로 수정) convert_lidar_kante 는 perception 폴더에 존재
add_executable(convert_lidar_kante src/perception/convert_lidar_kante.cpp)
target_link_libraries(convert_lidar_kante ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})
add_dependencies(convert_lidar_kante ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# (파일명 오타 반영) 현재 파일명이 obstace_detector.cpp 임
#   → 나중에 파일명을 obstacle_detector.cpp 로 바꾸면 여기 경로도 함께 바꿔주세요.
add_executable(obstacle_detector src/perception/obstace_detector.cpp)
target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})

# ----------------------------------------------------
# gap navigatior morai.cpp
add_executable(gap_navigator_morai src/decision/gap_navigator_morai.cpp)
target_link_libraries(gap_navigator_morai ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})

# ----------------------------------------------------
# stanely morai.cpp / 1105~1106
# 실제 파일 경로에 맞춰서 수정
add_executable(stanely_1105 src/control/stanely_1105.cpp)
target_link_libraries(stanely_1105 ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})
# (옵션) 메시지/생성물 의존성을 명시하고 싶다면:
# add_dependencies(stanely_1105 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# -----------------------------------------------------
# pp + stanely
#add_executable(curvature_control src/control/curvature_control.cpp)
#target_link_libraries(curvature_control ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})

add_executable(curvature_control_ver2 src/control/curvature_control_ver2.cpp)
target_link_libraries(curvature_control_ver2 ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})


add_executable(curvature_control_verPID src/control/curvature_control_verPID.cpp)
target_link_libraries(curvature_control_verPID ${catkin_LIBRARIES} ${GEOGRAPHIC_LIBRARY})