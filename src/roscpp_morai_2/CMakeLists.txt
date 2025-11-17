cmake_minimum_required(VERSION 3.0.2)
project(roscpp_morai_2)

add_compile_options(-std=c++14 -Wall -Wextra -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf
  morai_msgs
)

# ---- GeographicLib: 수동 탐색(권장, 가장 호환됨) ----
# 먼저 조용히 시도해보고 실패 시 수동탐색
# find_package(GeographicLib QUIET)

# 수동 탐색(대부분의 Ubuntu/Noetic 환경에서 안정적)
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/LocalCartesian.hpp
  /usr/include
  /usr/local/include
)
find_library(GEOGRAPHICLIB_LIBRARY NAMES Geographic
  /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

if(NOT GEOGRAPHICLIB_INCLUDE_DIR OR NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR
    "GeographicLib not found. Install it first:\n"
    "  sudo apt-get update && sudo apt-get install -y libgeographic-dev"
  )
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscpp_morai_2_core
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs sensor_msgs tf morai_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GEOGRAPHICLIB_INCLUDE_DIR}
)

add_library(roscpp_morai_2_core
  src/control_compute_function.cpp
  src/control_file_function.cpp
)

target_link_libraries(roscpp_morai_2_core
  ${catkin_LIBRARIES}
  ${GEOGRAPHICLIB_LIBRARY}
)

add_dependencies(roscpp_morai_2_core ${catkin_EXPORTED_TARGETS})

add_executable(curvature_control_clear
  src/curvature_control_clear.cpp
)

target_link_libraries(curvature_control_clear
  roscpp_morai_2_core
  ${catkin_LIBRARIES}
  ${GEOGRAPHICLIB_LIBRARY}
)

add_dependencies(curvature_control_clear ${catkin_EXPORTED_TARGETS})

# (옵션) 설치 규칙
install(TARGETS roscpp_morai_2_core curvature_control_clear
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml" PATTERN "*.txt"
)
