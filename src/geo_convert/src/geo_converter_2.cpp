#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <morai_msgs/GPSMessage.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>

// ======= 글로벌 설정값 =======
// 원점
static double g_lat0 = 37.239375;
static double g_lon0 = 126.7731828;
static double g_alt0 = 29.2783124182;
static double g_map_to_true_north_yaw_deg = 0.0;

static GeographicLib::LocalCartesian g_lc;
static bool g_lc_inited = false;

// ======= map(x, y, z) -> ENU 회전 정렬 =======
static inline void MapXYToENU(double x, double y, double z,
                              double& E, double& N, double& U)
{
  const double th = g_map_to_true_north_yaw_deg * M_PI / 180.0;
  const double c = std::cos(th), s = std::sin(th);
  E =  c*x - s*y;
  N =  s*x + c*y;
  U =  z;
}

// ======= 미션1: WGS -> UTM ===================
void wgs_to_utm(double lat_deg, double lon_deg, double alt_m,
                double& easting, double& northing, int& zone, bool& northp)
{
  double x, y;
  GeographicLib::UTMUPS::Forward(lat_deg, lon_deg, zone, northp, x, y);
  easting  = x;
  northing = y;
  (void)alt_m; // 고도는 무시
}

// ======= 미션2: WGS -> ENU ===================
void wgs_to_enu(double lat_deg, double lon_deg, double alt_m,
                double& E, double& N, double& U)
{
  if(!g_lc_inited) return;
  g_lc.Forward(lat_deg, lon_deg, alt_m, E, N, U); // LLA → ENU
}

// ======= 미션3: map -> WGS84 =================
void ego_to_wgs84(double ego_x, double ego_y, double ego_z,
                  double& lat_deg, double& lon_deg, double& alt_m)
{
  if(!g_lc_inited) return;
  double E, N, U;
  MapXYToENU(ego_x, ego_y, ego_z, E, N, U);       // map → ENU
  g_lc.Reverse(E, N, U, lat_deg, lon_deg, alt_m); // ENU → LLA
}

// ======= 미션4: map -> WGS84 -> ENU ===========
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

    // 미션3: map -> WGS84
    double lat, lon, alt;
    ego_to_wgs84(ego_x, ego_y, ego_z, lat, lon, alt);

    // 미션1: WGS -> UTM
    double utm_e, utm_n; int utm_zone; bool utm_northp;
    wgs_to_utm(lat, lon, alt, utm_e, utm_n, utm_zone, utm_northp);

    // 미션2: WGS -> ENU
    double e1, n1, u1;
    wgs_to_enu(lat, lon, alt, e1, n1, u1);

    // 미션4: map -> WGS -> ENU
    double e2, n2, u2;
    ego_to_wgs84_to_enu(ego_msg, e2, n2, u2);

    // map 직접 회전 (참값)
    double e_ref, n_ref, u_ref;
    MapXYToENU(ego_x, ego_y, ego_z, e_ref, n_ref, u_ref);

}

// ======= 콜백: GPS ==========================
void CB_gps(const morai_msgs::GPSMessage::ConstPtr& gps_msg)
{
  if (!g_lc_inited) return;

  double lat = gps_msg->latitude;
  double lon = gps_msg->longitude;
  double alt = gps_msg->altitude;

  // 미션1: WGS -> UTM
  double utm_e, utm_n; int utm_zone; bool utm_northp;
  wgs_to_utm(lat, lon, alt, utm_e, utm_n, utm_zone, utm_northp);

  // 미션2: WGS -> ENU
  double e, n, u;
  wgs_to_enu(lat, lon, alt, e, n, u);

  // 출력
  // 출력
  ROS_INFO_STREAM(std::fixed << std::setprecision(8)
    << "\n========== [Mission: Conversion Based on GPS] =========="
    << "\n[GPS WGS84]       lat=" << lat << ", lon=" << lon << ", alt=" << alt
    << "\n[UTM converted]   E=" << utm_e << ", N=" << utm_n << ", zone=" << utm_zone << (utm_northp ? "N" : "S")
    << "\n[ENU converted]   E=" << e << ", N=" << n << ", U=" << u
    << "\n========================================================="
  );

}

// ======= main ==================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "coord_convert_node");
  ros::NodeHandle nh("~");

  nh.param("origin_lat", g_lat0, g_lat0);
  nh.param("origin_lon", g_lon0, g_lon0);
  nh.param("origin_alt", g_alt0, g_alt0);
  nh.param("map_to_true_north_yaw_deg", g_map_to_true_north_yaw_deg, g_map_to_true_north_yaw_deg);

  g_lc.Reset(g_lat0, g_lon0, g_alt0); // 원점을 인식
  g_lc_inited = true;

  ros::Subscriber sub_ego = nh.subscribe("/Ego_topic", 10, CB_ego);
  ros::Subscriber sub_gps = nh.subscribe("/gps", 10, CB_gps);

  ros::spin();
  return 0;
}

/*

static double g_lat0 = 37.239375;
static double g_lon0 = 126.7731828;
static double g_alt0 = 29.2783124182;

*/
// '''
// [INFO] [1759345384.579275577]: 
// ========== [Mission: Conversion Based on Ego] ==========
// [Ego map]             x=41.178219, y=1160.998169, z=-0.293651
// [ego -> WGS84 converted]     lat=37.249836, lon=126.773647, alt=29.090782
// [ego -> UTM converted]       E=302544.456393, N=4124910.822032, zone=52N
// [WGS -> ENU]           E1=41.178219, N1=1160.998169, U1=-0.293651
// [ego(map) -> WGS -> ENU]     E2=41.178219, N2=1160.998169, U2=-0.293651 //이문제


// '''
// ========== [Mission: Conversion Based on GPS] ==========
// [GPS WGS84]       lat=37.23976672, lon=126.77343014, alt=29.31350410
// [WGS84 -> UTM converted]   E=302498.93474450, N=4123793.97498647, zone=52N
// [WGS84 -> ENU converted]   E=21.94730483, N=43.47402276, U=0.03500536
// =========================================================

// '''