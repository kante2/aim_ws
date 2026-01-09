
#include "lidar/convex.h"

#include <cmath>
#include <limits>
#include <algorithm>

// =====================
// Constructor
// =====================
LShapeClosenessFitter::LShapeClosenessFitter(double delta_deg, double d0)
    : delta_theta_(delta_deg * M_PI / 180.0), d0_(d0) {}
// 점들을 정렬하기 위한 보조 함수 (x좌표 우선, 같으면 y좌표 기준)
bool comparePoints(const Point2D& a, const Point2D& b) {
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}

// 세 점의 방향성을 확인하는 함수 (외적 이용) [cite: 56, 67]
// 결과가 0보다 크면 반시계 방향, 0이면 일직선, 0보다 작으면 시계 방향 
double crossProduct(const Point2D& a, const Point2D& b, const Point2D& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}
//-------------------점들을 감싸는 외곽선 정점 뽑ㄱ-------------------------------------------------------
std::vector<Point2D> LShapeClosenessFitter::findConvexHull(const std::vector<Point2D>& pts) {
    int n = pts.size();
    if (n <= 3) return pts; // 점이 3개 이하면 그 자체가 외곽선임

    std::vector<Point2D> sorted_pts = pts;
    std::sort(sorted_pts.begin(), sorted_pts.end(), comparePoints); // 1. 점 정렬 

    std::vector<Point2D> hull;

    // 2. 아래쪽 껍질(Lower Hull) 만들기 [cite: 52]
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2 && crossProduct(hull[hull.size() - 2], hull.back(), sorted_pts[i]) <= 0) {
            hull.pop_back(); // 안쪽으로 꺾이면 마지막 점 제거 
        }
        hull.push_back(sorted_pts[i]);
    }

    // 3. 위쪽 껍질(Upper Hull) 만들기 [cite: 52]
    int lower_hull_size = hull.size();
    for (int i = n - 2; i >= 0; --i) {
        while (hull.size() > lower_hull_size && crossProduct(hull[hull.size() - 2], hull.back(), sorted_pts[i]) <= 0) {
            hull.pop_back(); // 안쪽으로 꺾이면 제거 
        }
        hull.push_back(sorted_pts[i]);
    }

    hull.pop_back(); // 시작점이 중복되므로 마지막 점 하나 제거
    return hull;
}
//-----------------------면적 뽑기--------------------------------------------------
double LShapeClosenessFitter::calculateArea(const std::vector<Point2D>& hull) {  
    double area = 0.0;
    int n = hull.size();
    if (n < 3) return 0.0; // 점이 3개는 있어야 면적이 생김 [cite: 41, 42]

    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += hull[i].x * hull[j].y;
        area -= hull[j].x * hull[i].y;
    }
    return std::abs(area) / 2.0; // 절댓값을 취해 최종 면적 산출 [cite: 58, 72]
}
//------면적 가중치 ------------------------------------------------------------------------------------------------------
double LShapeClosenessFitter::calculateCHAWCScore(const std::vector<Point2D>& cluster, double theta, double a_hull) {
    // (a) 현재 각도로 점들을 정렬(투영)하고 박스 크기(A_OBB) 구하기 [cite: 92, 95]
    Eigen::Vector2d e1(cos(theta), sin(theta));
    Eigen::Vector2d e2(-sin(theta), cos(theta));
    
    double c1_min = std::numeric_limits<double>::max();
    double c1_max = -std::numeric_limits<double>::max();
    double c2_min = std::numeric_limits<double>::max();
    double c2_max = -std::numeric_limits<double>::max();

    std::vector<double> C1, C2;
    for (const auto& p : cluster) {
        double p1 = p.x * e1.x() + p.y * e1.y();
        double p2 = p.x * e2.x() + p.y * e2.y();
        C1.push_back(p1); C2.push_back(p2);
        c1_min = std::min(c1_min, p1); c1_max = std::max(c1_max, p1);
        c2_min = std::min(c2_min, p2); c2_max = std::max(c2_max, p2);
    }

    // (b) 테두리 근접도(Closeness Score, beta) 계산 [cite: 82, 95]
    double beta = 0.0;
    for (size_t i = 0; i < C1.size(); ++i) {
        double d1 = std::min(c1_max - C1[i], C1[i] - c1_min);
        double d2 = std::min(c2_max - C2[i], C2[i] - c2_min);
        double d = std::max(std::min(d1, d2), d0_);
        beta += 1.0 / d;
    }

    // (c) 면적 효율성 가중치(rho) 계산 [cite: 89, 92]
    double a_obb = (c1_max - c1_min) * (c2_max - c2_min);
    double rho = a_hull / (a_obb + 1e-6); // 1e-6은 0으로 나누기 방지 [cite: 92, 95]

    // (d) 최종 점수 = 근접도 * 면적 가중치 [cite: 90, 95]
    return beta * rho;
}
//-------------------------------------fit-----------------------------------------------
bool LShapeClosenessFitter::fit(const std::vector<Point2D>& cluster, Rectangle& rect) {
    if (cluster.size() < 10) return false;

    // 1. 외곽선(Convex Hull) 및 실제 면적 계산 [cite: 41, 71]
    std::vector<Point2D> hull = findConvexHull(cluster); 
    double a_hull = calculateArea(hull); 

    double best_score = -1.0;
    double best_theta = 0.0;

    // 2. 외곽선 모서리(Edge) 각도들만 후보로 탐색 [cite: 62]
    for (size_t i = 0; i < hull.size(); ++i) {
        int next = (i + 1) % hull.size();
        double dx = hull[next].x - hull[i].x;
        double dy = hull[next].y - hull[i].y;
        double theta = atan2(dy, dx); 

        // 각도를 0 ~ 90도 범위로 정규화 (선택 사항이나 권장) 
        while (theta < 0) theta += M_PI / 2.0;
        while (theta >= M_PI / 2.0) theta -= M_PI / 2.0;

        // 3. CH-AWC 점수 계산 [cite: 90, 92]
        double score = calculateCHAWCScore(cluster, theta, a_hull);
        
        if (score > best_score) {
            best_score = score;
            best_theta = theta;
        }
    }

    computeRectangle(cluster, best_theta, best_score, rect);
    return true;
}


// =====================
// Projection
// =====================
void LShapeClosenessFitter::project(const std::vector<Point2D>& pts,
                                    const Eigen::Vector2d& e1,
                                    const Eigen::Vector2d& e2,
                                    std::vector<double>& C1,
                                    std::vector<double>& C2)
{
    C1.clear();
    C2.clear();

    for (const auto& p : pts) {
        Eigen::Vector2d x(p.x, p.y);
        C1.push_back(x.dot(e1));
        C2.push_back(x.dot(e2));
    }
}


// =====================
// Closeness score
// =====================
double LShapeClosenessFitter::calculateCloseness(
    const std::vector<double>& C1,
    const std::vector<double>& C2)
{
    double c1_min = *std::min_element(C1.begin(), C1.end());
    double c1_max = *std::max_element(C1.begin(), C1.end());
    double c2_min = *std::min_element(C2.begin(), C2.end());
    double c2_max = *std::max_element(C2.begin(), C2.end());

    double beta = 0.0;

    for (size_t i = 0; i < C1.size(); ++i) {
        double d1 = std::min(c1_max - C1[i], C1[i] - c1_min);
        double d2 = std::min(c2_max - C2[i], C2[i] - c2_min);

        double d = std::max(std::min(d1, d2), d0_);
        beta += 1.0 / (d*d);
    }

    return beta;
}


// =====================
// Rectangle computation
// =====================
void LShapeClosenessFitter::computeRectangle(
    const std::vector<Point2D>& pts,
    double theta,
    double score,
    Rectangle& rect)
{
    Eigen::Vector2d e1(std::cos(theta), std::sin(theta));
    Eigen::Vector2d e2(-std::sin(theta), std::cos(theta));

    std::vector<double> C1, C2;
    project(pts, e1, e2, C1, C2);

    double c1_min = *std::min_element(C1.begin(), C1.end());
    double c1_max = *std::max_element(C1.begin(), C1.end());
    double c2_min = *std::min_element(C2.begin(), C2.end());
    double c2_max = *std::max_element(C2.begin(), C2.end());

    rect.corners[0] = c1_min * e1 + c2_min * e2;
    rect.corners[1] = c1_min * e1 + c2_max * e2;
    rect.corners[2] = c1_max * e1 + c2_max * e2;
    rect.corners[3] = c1_max * e1 + c2_min * e2;

    rect.heading = theta;
    rect.score = score;
}
