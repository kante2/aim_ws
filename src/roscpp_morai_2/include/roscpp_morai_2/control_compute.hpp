#pragma once
#include <vector>
#include <utility>

namespace roscpp_morai_2 {

// [-pi, pi]
double compute_WrapAngle(double a);

// 경로에서 최근접 인덱스
int compute_NearestIdx(const std::vector<std::pair<double,double>>& path,
                       double x, double y);

// i에서 곡률
double compute_CurvatureAtIndex(const std::vector<std::pair<double,double>>& path,
                                int i);

// Pure Pursuit 조향 (lx,ly는 차량로컬)
double compute_PurePursuitSteering(double lx, double ly,
                                   double wheelbase_L, double lfd);

// Stanley 조향
double compute_StanleySteering(int nearest_idx,
                               double yaw, double ego_speed_ms, double k_stanley,
                               double enu_x, double enu_y,
                               const std::vector<std::pair<double,double>>& path);

} // namespace roscpp_morai_2
