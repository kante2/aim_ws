// src/ego_csv_recorder.cpp
#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <fstream>
#include <iomanip>   // setprecision

static std::ofstream g_csv;
static bool g_open_csv_flag = false;
static bool g_wrote_header_flag = false;

void CB_ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego_msg)
{
  if(!g_open_csv_flag) return;

  if(!g_wrote_header_flag){
    g_csv << "stamp,x,y,yaw\n";
    g_wrote_header_flag = true;
  }

  // 타임스탬프: 메시지에 header가 없다고 가정하고 now() 사용
  const double stamp = ros::Time::now().toSec();

  g_csv << std::fixed << std::setprecision(6)
        << ego_msg->position.x << ","
        << ego_msg->position.y << ","
        << ego_msg->heading    << "\n";
  // g_csv.flush();  // 필요시 주석 해제(실시간 디스크 기록)
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ego_csv_recorder");
  ros::NodeHandle nodehandler("~");

  // 저장 경로 파라미터 (기본값은 /tmp)
  std::string csv_file;
  nodehandler.param<std::string>("csv_file", csv_file, "/tmp/ego_topic.csv");

  // 파일 열기 (덮어쓰기 모드)
  g_csv.open(csv_file, std::ios::out | std::ios::trunc);
  if(!g_csv){
    ROS_FATAL("Cannot open csv_file: %s", csv_file.c_str());
    return 1;
  }
  g_open_csv_flag = true;
  ROS_INFO_STREAM("[ego_csv_recorder] Recording /Ego_topic to " << csv_file);

  // 구독
  ros::Subscriber sub = nnodehandlerh.subscribe("/Ego_topic", 1, CB_ego);

  ros::spin();

  // 종료 시 자동 close (ofstrem 소멸자) — 명시적으로 닫아도 OK
  g_csv.close();
  return 0;
}
