#include "ros/ros.h"
#include "std_msgs/String.h" // 퍼블리시 할 메세지 타입
#include <sstream> // 문자열 만들기 위한 c++ 스트림


int main(int argc, char** argv)
{
  // 노드이름을 talker로 지정한다.
  // 같은 네임스페이스에 같은 이름의 노드가 있으면 기존노드가 내려감
  //  노드 헨들러 선언 이전에 해야한다.
  ros::init(argc, argv, "talker"); 

  ros::NodeHandle n;

  // 이노드가 /chatter을 std_msgs/String타입으로 발행하겠다고 roscore에게 등록한다.
  // 이후 구독자가 생기면 서로를 연결해주고 노드끼리 P2P 송수신
  // 큐 사이즈는, 실시간성이 중요하면 큐의 사이즈를 작게 잡는다.
  // 큐사이즈가 큰경우는, 손실을 최소화 해야할때이다.
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
  // 10HZ은 1초에 10번을 반복 - 목표 주기를 설정한다.
  // loop_rate.sleep();통해 남는 시간을 재워서 주기를 맞춘다.
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    // std_msgs 네임스페이스안 String타입을 쓰겠다.
    // msg라는 스택 객체를 생성한다.
    std_msgs::String msg;
    // ss은 문자열을 대상으로 입출력을 하는,
    // ss라는 이름의 스트림 객체이다.
    std::stringstream ss;
    // ss 스트림 객체에 넣는 표현이다.
    // count는 정수이고, stringstream용 operator << (int)오버로드가 호출되어 문자열 표현으로 변환되어 버퍼에 추가된다.
    // 왼쪽에서 오른쪽으로 평가되어 최종버퍼는 hello world 42같은 형태가 된다. 
    ss << "hello world " << count;
    // ss.str() : 스트램 객체 내부의 버퍼를 복사해 반환하는 함수이다
    // msg.data에 복사본을 대입시킨다.
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    // SPINONCE는 콜백 큐를 한번 처리하고 리턴한다.
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}


// #include "ros/ros.h" /*#include "ros/ros.h": roscpp의 핵심 API. 노드 초기화, 로그(ROS_INFO), NodeHandle, Rate 등.*/
// #include "std_msgs/string.h"/*#include "std_msgs/String.h": 퍼블리시할 메시지 타입(문자열).*/

// #include <sstream>

// int main(int argc, char **argv)
// {
//     /*
//     argc, argv를 통해서 command line에서 제공되는것을 리매핑할수 있음,
//     ros::init부분은 어떠한 ros system을 쓰기에 앞서 call해야 함.
//     */
//     ros::init(argc, argv, "talker");

//     /*
//     nodehandle은 ros system과 소통하기 위한 메인 접근포인트,
//     첫 nodehanlde이 이 노드를 초기화하고 마지막 nodehandle이 노드를 닫는다.
//     */
//     ros::NodeHandle n;

//     /*
//     ros master에게 이 노드가 어떤토픽명/어떤타입/으로 발행하는지__등록한다.
//     등록후, 구독자가 생길때마다 rosmaster가 서로 연결해주고 노드간 p2p(직접)통신이 이뤄진다.
//     advertise가 들려주는 publisher핸들로 나중에 publish(msg)를 호출하여 실제 메세지를 내보낸다.
//     */
//     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//     /*10 Hz로 루프를 돌리려는 의도.*/
//     ros::Rate loop_rate(10);

//     int count = 0;

//     while (ros::ok()) {
//         std_msgs::String msg;
//         std::stringstream ss;
//         ss << "hello world " << count;
//         msg.data = ss.str();

//         ROS_INFO("%s", msg.data.c_str());  // /rosout 로 로그 전송

//         chatter_pub.publish(msg);          // 구독자에게 전송(연결 없으면 빠르게 리턴)
//         ros::spinOnce();                   // 콜백 처리(여기선 콜백 없지만 관용적으로 둠)
//         loop_rate.sleep();                 // 10 Hz 유지
//         ++count;
//     }
// }