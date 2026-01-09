#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

enum CriterionType {
    AREA,
    CLOSENESS,
    VARIANCE
};

struct RectResult {
    double theta;
    double score;
    double c1, c2, c3, c4; // bounds in (e1,e2)
};

/* =========================
   Projection utilities
   ========================= */
inline void computeAxes(double theta,
                         Eigen::Vector2d& e1,
                         Eigen::Vector2d& e2)
{
    e1 << std::cos(theta), std::sin(theta);
    e2 << -std::sin(theta), std::cos(theta);
}

inline void projectPoints(const CloudT::Ptr& cloud,
                          const Eigen::Vector2d& e1,
                          const Eigen::Vector2d& e2,
                          std::vector<double>& C1,
                          std::vector<double>& C2)
{
    C1.clear(); C2.clear();
    for (const auto& p : cloud->points) {
        Eigen::Vector2d pt(p.x, p.y);
        C1.push_back(pt.dot(e1));
        C2.push_back(pt.dot(e2));
    }
}

/* =========================
   Algorithm 3: Area
   ========================= */
double areaCriterion(const std::vector<double>& C1,
                     const std::vector<double>& C2)
{
    double w = *std::max_element(C1.begin(), C1.end())
             - *std::min_element(C1.begin(), C1.end());
    double h = *std::max_element(C2.begin(), C2.end())
             - *std::min_element(C2.begin(), C2.end());

    return -(w * h); // smaller area is better
    // 알고리즘2에서 점수가 큰 세타를 선택하도록 되어 있는데,
    // area criterion 은 면적이 작을수록 좋다고 되어 있기 때문에
    // 부호를 뒤집에서 클수록 좋은 점수가 나오게 한 것

    // 최소 외접 직사각형
    // 최소 면적 직사각형
    // 을 찾아야 하는데 면적 계산 말고는 전부 점수q 를 계산. 점수가 높을 수록 좋은 거지. 클수록 좋다는 개념을 맞춰줭
}

/* =========================
   Algorithm 4: Closeness
   ========================= */

// 직사각형 내부가 아니라 변 근처에 점들이 강하게 분포해야 좋은 이유는, 
// 라이다에서 L-shape 는 표면에서만 레이저가 반사되고, 내부는 관측 X 
// 그래서 점이 내부에 있으면 노이즈

// area 알고리즘은 직사각형의 면적을 계산하기 때문에 점들이 변에 붙어있는지 구분을 못함. 
// -> closeness 필요한 이유.

double closenessCriterion(const std::vector<double>& C1,
                          const std::vector<double>& C2,
                          double d0 = 0.05)
{   
    // 직사각형 변 정의 
    double c1 = *std::min_element(C1.begin(), C1.end()); // 좌
    double c3 = *std::max_element(C1.begin(), C1.end()); // 우
    double c2 = *std::min_element(C2.begin(), C2.end()); // 아래
    double c4 = *std::max_element(C2.begin(), C2.end()); // 위

    // 
    double score = 0.0;
    for (size_t i = 0; i < C1.size(); ++i) {

        // 각 점에서 가장 가까운 변 찾기
        double D1 = std::min(std::abs(C1[i] - c1), std::abs(C1[i] - c3));
        double D2 = std::min(std::abs(C2[i] - c2), std::abs(C2[i] - c4));
        double d = std::max(std::min(D1, D2), d0);
        score += 1.0 / d;
    }
    return score;
}

/* =========================
   Algorithm 5: Variance
   ========================= */
double varianceCriterion(const std::vector<double>& C1,
                         const std::vector<double>& C2)
{
    double c1 = *std::min_element(C1.begin(), C1.end());
    double c3 = *std::max_element(C1.begin(), C1.end());
    double c2 = *std::min_element(C2.begin(), C2.end());
    double c4 = *std::max_element(C2.begin(), C2.end());

    std::vector<double> E1, E2;

    for (size_t i = 0; i < C1.size(); ++i) {
        double D1 = std::min(std::abs(C1[i] - c1), std::abs(C1[i] - c3));
        double D2 = std::min(std::abs(C2[i] - c2), std::abs(C2[i] - c4));

        if (D1 < D2)
            E1.push_back(D1);
        else
            E2.push_back(D2);
    }

    if (E1.size() < 2 || E2.size() < 2)
        return -1e9;

    auto variance = [](const std::vector<double>& v) {
        double mean = 0.0;
        for (double x : v) mean += x;
        mean /= v.size();

        double var = 0.0;
        for (double x : v)
            var += (x - mean) * (x - mean);
        return var / v.size();
    };

    return variance(E1) - variance(E2);
}

/* =========================
   Algorithm 2: Search
   ========================= */
RectResult fitRectangle(const CloudT::Ptr& cloud,
                        CriterionType type,
                        double dtheta = M_PI / 180.0)
{
    RectResult best;
    best.score = -std::numeric_limits<double>::infinity();

    for (double theta = 0; theta <= M_PI / 2.0; theta += dtheta) {

        Eigen::Vector2d e1, e2;
        computeAxes(theta, e1, e2);

        std::vector<double> C1, C2;
        projectPoints(cloud, e1, e2, C1, C2);

        double score;
        if (type == AREA)
            score = areaCriterion(C1, C2);
        else if (type == CLOSENESS)
            score = closenessCriterion(C1, C2);
        else
            score = varianceCriterion(C1, C2);

        if (score > best.score) {
            best.score = score;
            best.theta = theta;
            best.c1 = *std::min_element(C1.begin(), C1.end());
            best.c3 = *std::max_element(C1.begin(), C1.end());
            best.c2 = *std::min_element(C2.begin(), C2.end());
            best.c4 = *std::max_element(C2.begin(), C2.end());
        }
    }
    return best;
}

/* =========================
   ROS Node (example)
   ========================= */
void cloudCallback(const CloudT::Ptr& msg)
{
    RectResult res = fitRectangle(msg, CLOSENESS);

    ROS_INFO("Best theta = %.2f deg", res.theta * 180.0 / M_PI);
    ROS_INFO("Bounds: c1=%.2f c2=%.2f c3=%.2f c4=%.2f",
             res.c1, res.c2, res.c3, res.c4);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lshape_fitting_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<CloudT>(
        "/cluster_cloud", 1, cloudCallback);

    ros::spin();
    return 0;
}
