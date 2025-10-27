// src/ego_bag_recorder.cpp
#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <rosbag/bag.h>

rosbag::Bag g_bag;

void egoCb(const morai_msgs::EgoVehicleStatus::ConstPtr& ego_msg) {
  // 메시지에 header가 있다면 msg->header.stamp 쓰는 게 재생에 더 좋음
  ros::Time stamp = ros::Time::now();
  g_bag.write("/Ego_topic", stamp, *ego_msg); 
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ego_bag_recorder");
  ros::NodeHandle nodehandler("~");

  // 파일명 파라미터 (기본 ego.bag)
  // 지역변수 설정
  std::string bag_file;
  // nodehandler.param <type> (key, 출력변수, 기본값)
  // 파라미터 서버란 - ros에서 쓰는 전역 설정 저장소이다,
  // 키-값 형태로 설정들을 보관한다., rosmaster프로세스가 보관하고 제공한다.
  //  노드들이 필요할때 읽고쓰는 공용 딕셔너리

  // launch에서 경로를 파라미터로 넣어주면 된다.
  //bagfile변수에 launch에서 보내준 파라미터값이 그대로 박힘.
  nodehandler.param<std::string>("bag_file", bag_file, "/tmp/ego.bag");

  // bag 열기
  // rosbag file을 쓰기 모드로 연다. 닫을때 .bag으로 바뀐다.
  g_bag.open(bag_file, rosbag::bagmode::Write);

  // 로그용
  ROS_INFO_STREAM("Recording /Ego_topic to " << bag_file);

  // 구독 시작
  ros::Subscriber sub = nodehandler.subscribe("/Ego_topic", 100, egoCb);

  ros::spin();

  // 종료 시 닫기
  g_bag.close();
  return 0;
}

// -p 옵션: CSV 형태로 평탄화 출력
// rostopic echo -b /root/ws/src/roscpp_morai/rosbag/ego.bag -p /Ego_topic > ego_topic.csv
// 하여 저장한 rosbag을 csv로 변경한다.
