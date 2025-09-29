#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>

// ======= 글로벌 설정값 =======
static double g_lat0 = 37.0;
static double g_lon0 = 127.0;
static double g_alt0 = 0.0;
static double g_map_to_true_north_yaw_deg = 0.0;

static GeographicLib::LocalCartesian g_lc;
static bool g_lc_inited = false;

// map(x,y,z) -> ENU(E,N,U)로 회전 정렬
static inline void MapXYToENU(double x, double y, double z,
                              double& E, double& N, double& U)
{
  const double th = g_map_to_true_north_yaw_deg * M_PI / 180.0;
  const double c = std::cos(th), s = std::sin(th);
  E =  c*x - s*y;
  N =  s*x + c*y;
  U =  z;
}

// 미션1: WGS -> UTM ==================
// (lat,lon,alt) 입력, (e,n,zone,northp) 출력
void wgs_to_utm(double lat_deg, double lon_deg, double alt_m,
                double& easting, double& northing, int& zone, bool& northp)
{
  double x, y;
  GeographicLib::UTMUPS::Forward(lat_deg, lon_deg, zone, northp, x, y);
  easting  = x;
  northing = y;
  (void)alt_m; // UTM 변환 자체는 고도 영향 없음
}

// 미션2: WGS -> ENU ==================
// (lat,lon,alt) 입력, (E,N,U) 출력
void wgs_to_enu(double lat_deg, double lon_deg, double alt_m,
                double& E, double& N, double& U)
{
  if(!g_lc_inited) return;
  g_lc.Forward(lat_deg, lon_deg, alt_m, E, N, U); // LLA -> ENU
}

//  ego(map) -> WGS84(LLA) ==================
// (ego_x,y,z) 입력, (lat,lon,alt) 출력
void ego_to_wgs84(double ego_x, double ego_y, double ego_z,
                  double& lat_deg, double& lon_deg, double& alt_m)
{
  if(!g_lc_inited) return;
  double E,N,U;
  MapXYToENU(ego_x, ego_y, ego_z, E, N, U);
  g_lc.Reverse(E, N, U, lat_deg, lon_deg, alt_m); // ENU -> LLA
}

// 미션3: ego -> WGS84 -> ENU =================
// (ego ptr) 입력, (E,N,U) 출력
void ego_to_wgs84_to_enu(const morai_msgs::EgoVehicleStatus::ConstPtr& ego,
                         double& E, double& N, double& U)
{
  if(!g_lc_inited) return;

  // 1) ego(map) -> WGS84
  double lat, lon, alt;
  ego_to_wgs84(ego->position.x, ego->position.y, ego->position.z, lat, lon, alt);
  // 2) WGS84 -> ENU
  wgs_to_enu(lat, lon, alt, E, N, U);
}

// 콜백 =================
void CB_ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego_msg)
{
  const double ego_x = ego_msg->position.x;
  const double ego_y = ego_msg->position.y;
  const double ego_z = ego_msg->position.z;

  // ego -> WGS84
  double lat, lon, alt;
  ego_to_wgs84(ego_x, ego_y, ego_z, lat, lon, alt);

  // WGS -> UTM
  double utm_e, utm_n; int utm_zone; bool utm_northp;
  wgs_to_utm(lat, lon, alt, utm_e, utm_n, utm_zone, utm_northp);

  // WGS -> ENU
  double e1, n1, u1;
  wgs_to_enu(lat, lon, alt, e1, n1, u1);

  // ego -> WGS -> ENU (검증 파이프라인, 동일 origin)
  double e2, n2, u2;
  ego_to_wgs84_to_enu(ego_msg, e2, n2, u2);

  // map -> ENU 직접 회전(참값)
  double e_ref, n_ref, u_ref;
  MapXYToENU(ego_x, ego_y, ego_z, e_ref, n_ref, u_ref);

  // 출력
  ROS_INFO_STREAM(std::fixed << std::setprecision(6)
    << "\n[Ego map] x=" << ego_x << ", y=" << ego_y << ", z=" << ego_z
    << "\n[WGS84]   lat=" << lat << ", lon=" << lon << ", alt=" << alt
    << "\n[UTM]     E=" << utm_e << ", N=" << utm_n
    << ", zone=" << utm_zone << (utm_northp ? "N" : "S")
    << "\n[ENU from WGS]      E1=" << e1 << ", N1=" << n1 << ", U1=" << u1
    << "\n[ego->WGS->ENU]     E2=" << e2 << ", N2=" << n2 << ", U2=" << u2
    << "\n[map->ENU (rotate)] E*=" << e_ref << ", N*=" << n_ref << ", U*=" << u_ref
    << "\nDiff(E1-E*)=" << (e1 - e_ref)
    << ", Diff(N1-N*)=" << (n1 - n_ref)
    << ", Diff(U1-U*)=" << (u1 - u_ref)
  );
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coord_convert_node");
  ros::NodeHandle nh("~");

  nh.param("origin_lat", g_lat0, g_lat0); // 맵의 (0,0,0)이 실제 지구에서 어디인지(WGS-84)
  nh.param("origin_lon", g_lon0, g_lon0);
  nh.param("origin_alt", g_alt0, g_alt0);
  // 맵의 +Y축이 ‘진북’과 몇 도 다른지
  nh.param("map_to_true_north_yaw_deg", g_map_to_true_north_yaw_deg, g_map_to_true_north_yaw_deg);

  g_lc.Reset(g_lat0, g_lon0, g_alt0);
  g_lc_inited = true;

  ros::Subscriber sub = nh.subscribe("/Ego_topic", 10, CB_ego);
  ros::spin();
  return 0;
}
