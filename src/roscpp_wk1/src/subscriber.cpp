#include "ros/ros.h"
#include "std_msgs/String.h"   // ← 대소문자 정확히!

//  /chatter 토픽에서 std_msgs/String 메시지를 받을 때마다 호출
// nodehandler선언 이전에 호출되어야 한다.

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  // nodehande은 토픽, 서비스, 파라미터, 타이머 등을 만드는 출입구이다.
  // 첫 노드헨들러가 생성되면 완전 초기화됨
  // 마지막 노드헨들러 파괴 -> 노드가 정리된다, == 구독,발행이 해제된다.
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // 현재 메인 스레드에서 무한 루프를 돌며 콜백 큐에 들어온 이벤트를 처리한다.
  ros::spin();
  // ros::spinOnce() + ros::Rate로 직접 루프를 돌릴 수도 있다.
  return 0;
}
