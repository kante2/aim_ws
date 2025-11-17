#include "roscpp_morai_2/control_compute.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

//
namespace roscpp_morai_2 {

double compute_WrapAngle(double a){
  constexpr double TWO_PI = 6.28318530717958647692;
  while (a >  M_PI) a -= TWO_PI;
  while (a < -M_PI) a += TWO_PI;
  return a;
}

// 경로에서 최근접 인덱스
// std::vector<std::pair<double,double>> g_path_xy;
// std::vector<T> --> T 타입의 가변 길이 배열(동적 배열)
// std::pair<A,B> --> A 타입과 B 타입의 두 값을 묶은 쌍
// 결과적으로 (x,y) 점들을 담은 동적 배열이다.
int compute_NearestIdx(const std::vector<std::pair<double,double>>& path,
                       double x, double y) {
  if (path.empty()) return -1;
  int best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i=0, N=(int)path.size(); i<N; ++i){
    const double dx = path[i].first  - x;
    const double dy = path[i].second - y;
    const double d2 = dx*dx + dy*dy;
    if (d2 < best_d2){ best_d2 = d2; best = i; }
  }
  return best;
}

double compute_CurvatureAtIndex(const std::vector<std::pair<double,double>>& path,
                                int i) {
  const int N = (int)path.size();
  if (N < 3) return 0.0;
  // i+5, i+10 경계 보호
  // i기준 인덱스 5번째꺼 -> point j1
  // i기준 인덱스 10번째꺼 -> point j2
  const int j1 = std::min(i + 5,  N - 1);
  const int j2 = std::min(i + 10, N - 1);

  const auto [x1,y1] = path[i];
  const auto [x2,y2] = path[j1];
  const auto [x3,y3] = path[j2];

  const double a = std::hypot(x2-x1, y2-y1);
  const double b = std::hypot(x3-x2, y3-y2);
  const double c = std::hypot(x3-x1, y3-y1);
  if (a<=1e-6 || b<=1e-6 || c<=1e-6) return 0.0;

  const double cross = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
  const double A = 0.5 * std::fabs(cross);
  if (A <= 1e-9) return 0.0;

  const double R = (a*b*c) / (4.0*A);
  if (R <= 1e-6) return 0.0;

  const double kappa = 1.0 / R;
  return (cross >= 0.0 ? +1.0 : -1.0) * kappa;
}

double compute_PurePursuitSteering(double lx, double ly,
                                   double wheelbase_L, double lfd) {
  const double theta = std::atan2(ly, lx);
  return std::atan2(2.0 * wheelbase_L * std::sin(theta), lfd);
}

double compute_StanleySteering(int nearest_idx,
                               double yaw, double ego_speed_ms, double k_stanley,
                               double enu_x, double enu_y,
                               const std::vector<std::pair<double,double>>& path) {
  const int N = (int)path.size();
  const double c = std::cos(yaw), s = std::sin(yaw);

  const double dxn = path[nearest_idx].first  - enu_x;
  const double dyn = path[nearest_idx].second - enu_y;
  const double y_local = -s*dxn + c*dyn;         // 좌(+)
  const double e_y = y_local;

  const int idx2 = (nearest_idx + 1 < N) ? nearest_idx + 1
                : (nearest_idx - 1 >= 0) ? nearest_idx - 1
                                          : nearest_idx;
  const double dxt = path[idx2].first  - path[nearest_idx].first;
  const double dyt = path[idx2].second - path[nearest_idx].second;
  const double psi_path = std::atan2(dyt, dxt);

  const double psi_err = compute_WrapAngle(psi_path - yaw);
  const double v_safe  = std::max(ego_speed_ms, 0.1); // 0근처 보호
  return psi_err + std::atan2(k_stanley * e_y, v_safe);
}

} // namespace roscpp_morai_2
