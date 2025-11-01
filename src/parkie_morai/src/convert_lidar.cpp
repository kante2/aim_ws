

#include <vector>
#include <algorithm>
#include <memory>
#include <limits>   // 추가: infinity 사용
#include <cmath>    // 추가: sin/cos/M_PI

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;

class LidarConvert : public ros::NodeHandle
{
public:
    LidarConvert()
    {
        this->lidar_sub_ = this->subscribe("lidar2D", 10, &LidarConvert::lidarCallback, this);
        this->lidar_pub_ = this->advertise<sensor_msgs::LaserScan>("scan", 10);
    }

private:
    ros::Publisher lidar_pub_;
    ros::Subscriber lidar_sub_;
    bool init_flag_=false;
    sensor_msgs::LaserScan scan_msg_;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        if(!this->init_flag_){
            this->scan_msg_.header.frame_id = msg->header.frame_id;
            this->scan_msg_.angle_min = msg->angle_min;
            this->scan_msg_.angle_max = msg->angle_max;
            this->scan_msg_.angle_increment = msg->angle_increment;
            this->scan_msg_.time_increment = msg->time_increment;
            this->scan_msg_.scan_time = msg->scan_time;
            this->scan_msg_.range_min = msg->range_min;
            this->scan_msg_.range_max = msg->range_max;
            this->scan_msg_.ranges.resize(msg->ranges.size());
            this->init_flag_=true;
        }

        // 타임스탬프: 동기화를 위해 원본 사용 권장
        this->scan_msg_.header.stamp = msg->header.stamp;  // (필요하면 ros::Time::now()로 변경 가능)

        // --- 원본 코드 유지: 앞/뒤 180도 교환 ---
        std::transform(msg->ranges.begin(), msg->ranges.begin()+180,
                       scan_msg_.ranges.begin()+180, [](auto value){ return value; });

        std::transform(msg->ranges.begin()+180, msg->ranges.end(),
                       scan_msg_.ranges.begin(), [](auto value){ return value; });
        // ---------------------------------------

        /* === [추가] ROI 마스킹: 전방(±90°)만 남기고, 자기차량(근거리/차체 영역) 제거 ===
           - 배열 길이/angle_min/max 등 메타데이터는 그대로 유지
           - center: 전방 중심 각도 (0rad가 전방이면 0.0, 180°가 전방이면 M_PI)
           - r_ex: 라이다 하우징/범퍼 등 근거리 제거 반경
           - 직사각형 차체 박스(x: 앞뒤, y: 좌우)도 추가로 제거(선택적)
        */
        {
            const double amin = this->scan_msg_.angle_min;
            const double ainc = this->scan_msg_.angle_increment;
            const size_t N    = this->scan_msg_.ranges.size();

            if (N > 0 && std::abs(ainc) > 1e-12) {
                // 전방 기준 설정: 0.0(정면) 또는 M_PI(정면이 180도일 때)
                const double center = 0.0;                  // 필요시 M_PI 로 변경
                const double half   = M_PI * 0.5;           // ±90°
                const double a0     = center - half;
                const double a1     = center + half;

                // 자기차량 제거 파라미터 (상황 맞게 조정)
                const double r_ex     = 0.60;  // 반경 제거(0.4~0.8m 등 조정)
                const double half_len = 0.90;  // 차체 앞/뒤 절반 길이
                const double half_wid  = 0.50; // 차체 좌/우 절반 폭

                for (size_t i = 0; i < N; ++i) {
                    const double a = amin + static_cast<double>(i) * ainc;

                    // 전방(±90°) 창 밖은 무한대로 마스킹
                    if (a < a0 || a > a1) {
                        this->scan_msg_.ranges[i] = std::numeric_limits<float>::infinity();
                        continue;
                    }

                    float &r = this->scan_msg_.ranges[i];

                    // 유효하지 않거나 너무 가까운 값(자기차량/하우징)은 제거
                    if (!std::isfinite(r) || r < r_ex) {
                        r = std::numeric_limits<float>::infinity();
                        continue;
                    }

                    // (선택) 차체 직사각형 박스 제거: 라이다 기준 좌표 (x전방, y좌우)
                    const double x = r * std::cos(a);
                    const double y = r * std::sin(a);
                    if (std::fabs(x) < half_len && std::fabs(y) < half_wid) {
                        r = std::numeric_limits<float>::infinity();
                    }
                }
            }
        }
        /* === [추가 끝] === */

        this->lidar_pub_.publish(scan_msg_);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "lidar_convert");
    auto lidar_convert=make_shared<LidarConvert>();
    ros::spin();
    return 0;
}



// #include <vector>
// #include <algorithm>
// #include <memory>

// #include <ros/ros.h>
// #include "sensor_msgs/LaserScan.h"


// using namespace std;

// class LidarConvert : public ros::NodeHandle
// {
//     public:
//         LidarConvert()
//         {
//             this->lidar_sub_ = this->subscribe("lidar2D", 10, &LidarConvert::lidarCallback, this);
//             this->lidar_pub_ = this->advertise<sensor_msgs::LaserScan>("scan", 10);            
//         }

//     private:
//         ros::Publisher lidar_pub_;
//         ros::Subscriber lidar_sub_;
//         bool init_flag_=false;
//         sensor_msgs::LaserScan scan_msg_;

//         void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
//         {
//             if(!this->init_flag_){
//                 this->scan_msg_.header.frame_id = msg->header.frame_id;
//                 this->scan_msg_.angle_min = msg->angle_min;
//                 this->scan_msg_.angle_max = msg->angle_max;
//                 this->scan_msg_.angle_increment = msg->angle_increment;
//                 this->scan_msg_.time_increment = msg->time_increment;
//                 this->scan_msg_.scan_time = msg->scan_time;
//                 this->scan_msg_.range_min = msg->range_min;
//                 this->scan_msg_.range_max = msg->range_max;
//                 this->scan_msg_.ranges.resize(msg->ranges.size());
//                 this->init_flag_=true;
//             }
//             this->scan_msg_.header.stamp = ros::Time::now();

//             std::transform(msg->ranges.begin(), msg->ranges.begin()+180,
//                          scan_msg_.ranges.begin()+180,[](auto value){ return value; });
                         

//             std::transform(msg->ranges.begin()+180, msg->ranges.end(),
//                          scan_msg_.ranges.begin(),[](auto value){ return value; });

//             this->lidar_pub_.publish(scan_msg_);
//         }
// };

// int main(int argc, char ** argv)
// {
//     ros::init(argc, argv, "lidar_convert");
//     auto lidar_convert=make_shared<LidarConvert>();
//     ros::spin();

//     return 0;
// }
