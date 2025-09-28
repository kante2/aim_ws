#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// cpp에서 csv열때 사용하는  
#include <fstream>
#include <sstream>

// std::는 cpp 표준라이브러리의 이름공간이다.
// std::무언가 형태로 씀,

int main(int argc, char** argv){
    ros::init(argc, argv, "global_path_pub");
    ros::NodeHandle nodehandler("~");

    // 1) 퍼블리셔: 
    // latch=true → 한 번 보내면 새 구독자도 즉시 최신값 수신
    // nh.advertise<nav_msgs::Path>("/global_path", queue, latch)
    ros::Publisher pub = nodehandler.advertise<nav_msgs::Path>("/global_path", 1, true);

    // 2) Path 메시지 준비
    // Path - 메세지의 타입(클래스이름)
    // path는 그 타입으로 만든 변수명  
    // csv -> nav_msg/Path로 만들고 --> /global_path토픽으로 퍼블리시 하는 역할이다.
    nav_msgs::Path path;
    std::string csv_file; nodehandler.param<std::string>("csv_file", csv_file, "/tmp/ego_path.csv");
    std::string frame_id; nodehandler.param<std::string>("frame_id", frame_id, "map");

    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id;

    // 4) CSV 열기
    std::ifstream ifs(csv_file);
    if(!ifs){
    ROS_FATAL("Cannot open CSV: %s", csv_file.c_str());
    return 1;
    }

    // 5) 한 줄씩 읽어 x,y(,yaw) 파싱 → PoseStamped로 push_back
    // geometry::msgs ->
    auto makePose = [&](double x, double y){
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = frame_id;
    ps.header.stamp    = path.header.stamp;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0; // yaw 안 쓰는 최소 설정
    return ps;
    };

    std::string line;
    // 헤더 스킵 (첫 줄이 "x,y,yaw")
    std::getline(ifs, line);

    // getline(ifs, line) -> ifs에사 한줄을 읽어 line 에 넣는다
    while (std::getline(ifs, line)){
    if(line.empty()) continue;

    // line을 문자열 스트림으로 감싼다. - 이렇게 하면 문자열을 다시한번 읽기형태로 다룰수 있다.
    std::stringstream ss(line);
    std::string sx, sy, syaw;

    // ss에서 콤마를 구분자로 삼아 첫번째 토큰을 sx, 없으면 continue
    if(!std::getline(ss, sx, ',')) continue; // x
    if(!std::getline(ss, sy, ',')) continue; // y
    // yaw는 있어도/없어도 됨 (쓰지 않음)
    std::getline(ss, syaw, ',');

    // 문자열을 실수로 변환한다. 
    double px = std::stod(sx);
    double py = std::stod(sy);

    // makepose로  geometry_msgs::PoseStamped하나를 만들고 그걸
    // nav_msgs::Path안에는 std::vector<geometry_msgs::PoseStamped> poses; 라는 벡터=리스트가있다.
    // pushback으로 리스트 맨뒤에 원소 하나를 추가한다.
    path.poses.push_back(makePose(px, py));
    }

    // 6) 퍼블리시 (라치드라 1회면 충분)
    ROS_INFO_STREAM("[gb_pose] =" << path);
    pub.publish(path);
    ROS_INFO("[global_path_pub] published %zu poses from %s",
            path.poses.size(), csv_file.c_str());

    ros::spin(); // latched 유지
    return 0;
}