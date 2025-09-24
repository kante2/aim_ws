#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <fstream> 
// 파일 입출력 표준 헤더이다. 
// 입력 전용, 출력전용, 입출력 겸용 스트림 클래스가 들어가 있다.
// 현재 상황은 ego_topic값을 csv로 기록하는 것이기에 std::ofstream을 사용한다.
#include <cmath>

struct Pose2D { double x,y,yaw; };

class EgoPathRecorder {
  ros::Subscriber subscriber;
  std::ofstream ofstream; //출력 파일 스트림 객체이다.
  double save_distance;
  bool have_prev_ = false;
  Pose2D prev_pos{};

// public - 밖에서 써야 하는 함수는 public으로 열어야 한다.
//  그래야 main등에서 생성이 가능하다. 또한 콜백 포인터로 넘길 수 있다.
// private - class의 기본접근 제한자이다.
public:
  EgoPathRecorder(ros::NodeHandle& nh){
    // /root/ws/src/roscpp_morai/data
    // file에 변수가 안들어오면 다음 "root/...부분이 경로로 들어가게 된다"
    std::string file; nh.param<std::string>("file", file, "/root/ws/src/roscpp_morai/data/ego_path.cvs");
    // save_distance는 이전 기록점과의 최소 이동거리이다 -> 이 이상 움직이면 한 줄을 저장한다.
    nh.param<double>("save_dist", save_distance, 0.7); // 0.5~1.0 m 권장
    // 1.출력 파일 스트림 객체를 연다.
    ofstream.open(file);
    // 2. 출력 객체를 열면, << 연산자로 쓸 수 있다.
    ofstream << "x,y,yaw\n"; // z/speed 필요하면 컬럼 추가
    subscriber = nh.subscribe(
        "/Ego_topic", // 토픽 이름 -> (X, Y, YAW)를 받는다. 
        50, //큐 크기
        &EgoPathRecorder::CB_Ego_topic, //콜백함수
        this // 콜백을 실행할 객체 
    );
    ROS_INFO_STREAM("[recorder] recording to " << file);
  }

  static double norm(double a){ while(a>M_PI) a-=2*M_PI; while(a<-M_PI) a+=2*M_PI; return a; }

  void CB_Ego_topic(const morai_msgs::EgoVehicleStatus::ConstPtr& m){
    Pose2D p{m->position.x, m->position.y, m->heading}; // P에 현재 포즈를 담는다. 

    // 첫 샘플  무조한 한줄쓰고 prev를 리턴한다.
    if(!have_prev_){
      ofstream <<p.x<<","<<p.y<<","<<p.yaw<<"\n"; prev_=p; have_prev_=true; return;
    }

    double dx=p.x-prev_pos.x, dy=p.y-prev_pos.y;
    // 첫 샘플, 이후 나머지는 여기루프를 돈다.
    // save_distance 이상일 때만 CSV에 한 줄 저장 + prev_ 업데이트한다.
    if(std::hypot(dx,dy) >= save_distance){
      ofstream <<p.x<<","<<p.y<<","<<p.yaw<<"\n";
      prev_pos=p;
    }
  }
};

int main(int argc,char**argv){
    // 1. ROS를 초기화 하고, NODE에 이름을 붙여준다. 
    ros::init(argc,argv,"ego_path_recorder");
    // 2. 프라이빗 네임스페이스를용 헨들을 지정한다. 노드 전용 파라미터를 쉽게 받게 해준다. 
    ros::NodeHandle nh("~");
    // 3. 객체를 만들면 생성자가 실행되면서, 
    //  파라미터 읽고, 파일 열고 /Ego_topic 구독을 등록한다. 
    EgoPathRecorder node(nh);
    // 4. 무한 이벤트 루프, 메세지가 들어오몀ㄴ 콜백함수를 호출한다. 
    ros::spin();
    return 0;
}
