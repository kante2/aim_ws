#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <Global/Global.hpp>

// publisher 전역 선언
// visualization 함수 선언
// 퍼블리셔 실행되면 -> visual 함수 실행되고 + 판제한테 넘겨줘야 하는 값있으면 메시지도 같이 publish
//