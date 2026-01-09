#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

struct Point2D {
    double x;
    double y;
};

struct CHAWCResult {
    double score;
    double x_min, x_max, y_min, y_max; // OBB(=회전 후 AABB) in rotated frame
    double beta;  // closeness sum
    double rho;   // area ratio
    double A_OBB; // area of OBB
};

// 1. 회전되어 있는 직사각형을 축 정렬 직사각형으로 바꾼다. 
// 세타를 고정하고 축 정렬 되게끔 좌표계를 돌려서 min, max 로 직사각형 생성 

// Q) 그럼 처음 박스를 생성할 때 OBB로 박스를 생성해둔 상태여야 하나? 

// Rotate a point by -theta (same as applying xr = x*c - y*s, yr = x*s + y*c)
static inline Point2D rotate_minus_theta(const Point2D& p, double c, double s) {
    // xr = x*c - y*s
    // yr = x*s + y*c
    return { p.x * c - p.y * s, p.x * s + p.y * c };
}

// CH-AWC score for a given theta
CHAWCResult CalculateCHAWCScore(
    const std::vector<Point2D>& V,   // points (cluster points) 원본
    double theta,                    // angle candidate
    double A_hull,                   // convex hull area (precomputed)
    double d0,                       // distance floor
    double eps                       // small epsilon
) {
    CHAWCResult out{};
    if (V.empty()) {
        out.score = 0.0;
        out.x_min = out.x_max = out.y_min = out.y_max = 0.0;
        out.beta = 0.0;
        out.rho = 0.0;
        out.A_OBB = 0.0;
        return out;
    }

    const double c = std::cos(theta);
    const double s = std::sin(theta);

    // (a) Rotation and Axis-Aligned Bounding Box Calculation (in rotated frame)
    double x_min =  std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min =  std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();

    std::vector<Point2D> Vr;
    Vr.reserve(V.size());

    // 축 정렬 직사각형 만든 후에 직사각형 꼭짓점 정의
    for (const auto& p : V) {
        // p : V 원본 클러스터 포인트클라우드 중 한 포인트 
        Point2D pr = rotate_minus_theta(p, c, s);
        // theta : 박스가 놓일 방향을 가정한 각
        // pr : p를 -theta 만큼 회전한 점(축 정렬버전 포인트)
        Vr.push_back(pr);
        // Vr : 회전된 점들을 전부 저장

        // 회전된 좌표계(축 정렬)에서의 직사각형 경계(변)
        x_min = std::min(x_min, pr.x);
        x_max = std::max(x_max, pr.x);
        y_min = std::min(y_min, pr.y);
        y_max = std::max(y_max, pr.y);
    }

    // (b) Closeness Score beta = sum(1/d)

    // d = max( min(dX, dY), d0 )
    // dX = min(xr - x_min, x_max - xr), dY = min(yr - y_min, y_max - yr)

    // 각 점이 박스의 어느 변에 얼마나 가까운지를 재서 경계에 점들이 잘 붙어있는 theta를 높게 점수 주려는 것

    double beta = 0.0;
    for (const auto& pr : Vr) {
        const double dX = std::min(pr.x - x_min, x_max - pr.x);
        // pr.x - x_min : 점에서 왼쪽 변(x_min) 까지 거리
        // x_max - pr.x : 점에서 오른쪽 변(x_max) 까지 거리
        // → 둘 중 더 작은 값이 x방향으로 가장 가까운 수직 경계까지 거리

        const double dY = std::min(pr.y - y_min, y_max - pr.y);

        // => dX : 점이 좌/우 변 중 더 가까운 변까지의 거리
        // => dY : 상 / 하

        // If numerical noise puts a point slightly outside, clamp negatives to 0
        const double dXc = std::max(dX, 0.0);
        const double dYc = std::max(dY, 0.0);
        // 모든 점 pr은 상자 안에 있어야 하는데 아주 조금이라도 상자 밖으로 나가면 거리를 0으로 취급해버림

        const double d = std::max(std::min(dXc, dYc), d0);
        // 전체 4개 변 중 최소거리

        beta += 1.0 / d;
        // 박스 변 근처에 점이 많을수록 d는 0에 수렴하니까 beta 값은 커짐
    }

    // (c) Convex Hull Area Ratio rho = A_hull / (A_OBB + eps)

    // 박스를 크게 만들어도 점들이 경계에 가까울 수 있다. 
    // 그래서 박스가 불필요하게 크지 않은지를 함께 평가해야. = convex hull
    const double width  = std::max(0.0, x_max - x_min);
    const double height = std::max(0.0, y_max - y_min);
    const double A_OBB = width * height; // 세타에서의 박스 면적
    const double rho = A_hull / (A_OBB + eps); // A_hull : 점들이 실제로 차지하는 convex hull 면적
    // 박스가 타이트하면 A_OBB가 작아져서 rho 가 커짐
    // 박스가 헐렁하면 A_OBB가 커져서 rho 가 작아짐

    // (d) CH-AWC Score = beta * rho
    out.score = beta * rho; 
    // beta : 점들이 변에 붙어있나? 
    // rho : 박스가 타이트한가? 
    out.x_min = x_min; 
    out.x_max = x_max;
    out.y_min = y_min; 
    out.y_max = y_max;
    // 축 정렬된 좌표계에서 박스 경계

    // -> 원래 좌표계로 만드려면 4개 변 이용해서 점만들고 +theta 하면 됨
    
    out.beta  = beta;
    out.rho   = rho;
    out.A_OBB = A_OBB;
    return out;
}
