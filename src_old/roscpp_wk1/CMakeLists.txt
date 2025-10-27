cmake_minimum_required(VERSION 3.0.2)
project(roscpp_wk1)

add_compile_options(-std=gnu++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## 메시지/서비스 파일 등록
add_message_files(FILES
  Greeting.msg
)

add_service_files(FILES
  AddTwoInts.srv
)

## 코드 생성
generate_messages(DEPENDENCIES std_msgs)

## 패키지 정의
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## node1.
add_executable(publisher  src/publisher.cpp)
target_link_libraries(publisher  ${catkin_LIBRARIES})


## node2.
add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})

## 새 예제 노드 4개
add_executable(greet_pub src/greet_pub.cpp)
target_link_libraries(greet_pub ${catkin_LIBRARIES})
add_dependencies(greet_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(greet_sub src/greet_sub.cpp)
target_link_libraries(greet_sub ${catkin_LIBRARIES})
add_dependencies(greet_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(adder_server src/adder_server.cpp)
target_link_libraries(adder_server ${catkin_LIBRARIES})
add_dependencies(adder_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(adder_client src/adder_client.cpp)
target_link_libraries(adder_client ${catkin_LIBRARIES})
add_dependencies(adder_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})