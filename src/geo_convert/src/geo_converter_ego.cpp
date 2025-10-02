#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>     // 타입 호환 위해 포함(미사용)
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>

// ======= 글로벌 설정값 =======
// 원점 (launch/param으로 덮어씀)
static double g_lat0 = 37.239375;
static double g_lon0 = 126.7731828;
static double g_alt0 = 29.2783124182;
static double g_map_to_true_north_yaw_deg = 0.0;

static GeographicLib::LocalCartesian g_lc;
static bool g_lc_inited = false;

// ======= map(x, y, z) → ENU 회전 정렬 =======
static inline void MapXYToENU(double x, double y, double z,
                              double& E, double& N, double& U)
{
  const double th = g_map_to_true_north_yaw_deg * M_PI / 180.0;
  const double c = std::cos(th), s = std::sin(th);
  E =  c*x - s*y;
  N =  s*x + c*y;
  U =  z;
}

// ======= 미션1: WGS → UTM ===================
void wgs_to_utm(double lat_deg, double lon_deg, double alt_m,
                double& easting, double& northing, int& zone, bool& northp)
{
  double x, y;
  GeographicLib::UTMUPS::Forward(lat_deg, lon_deg, zone, northp, x, y);
  easting  = x;
  northing = y;
  (void)alt_m; // 고도는 무시
}

// ======= 미션2: WGS → ENU ===================
void wgs_to_enu(double lat_deg, double lon_deg, double alt_m,
                double& E, double& N, double& U)
{
  if(!g_lc_inited) return;
  g_lc.Forward(lat_deg, lon_deg, alt_m, E, N, U); // LLA → ENU
}

// ======= 미션3: map → WGS84 =================
void ego_to_wgs84(double ego_x, double ego_y, double ego_z,
                  double& lat_deg, double& lon_deg, double& alt_m)
{
  if(!g_lc_inited) return;
  double E, N, U;
  MapXYToENU(ego_x, ego_y, ego_z, E, N, U);       // map → ENU
  g_lc.Reverse(E, N, U, lat_deg, lon_deg, alt_m); // ENU → LLA
}

// ======= 미션4: map → WGS84 → ENU ===========
void ego_to_wgs84_to_enu(const morai_msgs::EgoVehicleStatus::ConstPtr& ego,
                         double& E, double& N, double& U)
{
  if(!g_lc_inited) return;
  double lat, lon, alt;
  ego_to_wgs84(ego->position.x, ego->position.y, ego->position.z, lat, lon, alt);
  wgs_to_enu(lat, lon, alt, E, N, U);
}

// ======= 콜백: Ego ==========================
void CB_ego(const morai_msgs::EgoVehicleStatus::ConstPtr& ego_msg)
{
  const double ego_x = ego_msg->position.x;
  const double ego_y = ego_msg->position.y;
  const double ego_z = ego_msg->position.z;

  // 미션3: map → WGS84
  double lat = std::numeric_limits<double>::quiet_NaN();
  double lon = std::numeric_limits<double>::quiet_NaN();
  double alt = std::numeric_limits<double>::quiet_NaN();
  ego_to_wgs84(ego_x, ego_y, ego_z, lat, lon, alt);
  if(!g_lc_inited || !std::isfinite(lat) || !std::isfinite(lon)) return;

  // 미션1: WGS → UTM
  double utm_e, utm_n; int utm_zone; bool utm_northp;
  wgs_to_utm(lat, lon, alt, utm_e, utm_n, utm_zone, utm_northp);

  // 미션2: WGS → ENU
  double e1, n1, u1;
  wgs_to_enu(lat, lon, alt, e1, n1, u1);

  // 미션4: map → WGS → ENU
  double e2, n2, u2;
  ego_to_wgs84_to_enu(ego_msg, e2, n2, u2);

  // map 직접 회전 (참값)
  double e_ref, n_ref, u_ref;
  MapXYToENU(ego_x, ego_y, ego_z, e_ref, n_ref, u_ref);

  ROS_INFO_STREAM(std::fixed << std::setprecision(6)
    << "\n========== [Mission: Conversion Based on Ego] =========="
    << "\n[Ego map]             x=" << ego_x << ", y=" << ego_y << ", z=" << ego_z
    << std::setprecision(6)
    << "\n[WGS84 converted]     lat=" << lat << ", lon=" << lon << ", alt=" << alt
    << std::setprecision(6)
    << "\n[UTM converted]       E=" << utm_e << ", N=" << utm_n
    << ", zone=" << utm_zone << (utm_northp ? "N" : "S")
    << std::setprecision(6)
    << "\n[WGS → ENU]           E1=" << e1 << ", N1=" << n1 << ", U1=" << u1
    << "\n[map → WGS → ENU]     E2=" << e2 << ", N2=" << n2 << ", U2=" << u2
    << "\n[Map rotation (ref)]  E*=" << e_ref << ", N*=" << n_ref << ", U*=" << u_ref
    << "\n[Diff (E1 - E*)]      dE=" << (e1 - e_ref)
    << ", dN=" << (n1 - n_ref) << ", dU=" << (u1 - u_ref)
    << "\n=========================================================");
}

// ======= main ==================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "coord_convert_ego_node");
  ros::NodeHandle nh("~");

  nh.param("origin_lat", g_lat0, g_lat0);
  nh.param("origin_lon", g_lon0, g_lon0);
  nh.param("origin_alt", g_alt0, g_alt0);
  nh.param("map_to_true_north_yaw_deg", g_map_to_true_north_yaw_deg, g_map_to_true_north_yaw_deg);

  g_lc.Reset(g_lat0, g_lon0, g_alt0);
  g_lc_inited = true;

  ROS_INFO_STREAM("[coord_convert_ego_node] origin = ("
    << std::fixed << std::setprecision(8)
    << g_lat0 << ", " << g_lon0 << ", " << std::setprecision(3) << g_alt0
    << "), map_to_true_north_yaw_deg = " << g_map_to_true_north_yaw_deg);

  ros::Subscriber sub_ego = nh.subscribe("/Ego_topic", 10, CB_ego);

  ros::spin();
  return 0;
}
