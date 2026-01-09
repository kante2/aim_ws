#ifndef LSHAPE_FITTER_HPP
#define LSHAPE_FITTER_HPP

#include <vector>
#include <Eigen/Dense>

// [주의] 메인 모듈과 공유하는 Point2D 및 Rectangle 구조체
struct Point2D {
    double x;
    double y;
};

struct Rectangle {
    Eigen::Vector2d corners[4];
    double heading;
    double score;
};

class LShapeClosenessFitter {
public:
    LShapeClosenessFitter(double delta_deg = 5.0, double d0 = 0.15);

    // [원본 성능 핵심] CH-AWC 기반 L-Shape 피팅 함수
    bool fit(const std::vector<Point2D>& cluster, Rectangle& rect);

private:
    double delta_theta_;
    double d0_;

    void project(const std::vector<Point2D>& pts, const Eigen::Vector2d& e1, const Eigen::Vector2d& e2,
                 std::vector<double>& C1, std::vector<double>& C2);

    double calculateCloseness(const std::vector<double>& C1, const std::vector<double>& C2);

    void computeRectangle(const std::vector<Point2D>& pts, double theta, double score, Rectangle& rect);

    // Convex Hull 및 CH-AWC 관련 보조 함수
    std::vector<Point2D> findConvexHull(const std::vector<Point2D>& pts);
    double calculateArea(const std::vector<Point2D>& hull);
    double calculateCHAWCScore(const std::vector<Point2D>& cluster, double theta, double a_hull);
};

#endif