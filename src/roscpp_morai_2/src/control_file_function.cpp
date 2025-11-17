#include "roscpp_morai_2/control_file.hpp"
#include <ros/ros.h>
#include <fstream>
#include <sstream>

namespace roscpp_morai_2 {

bool loadOrigin(const std::string& file,
                GeographicLib::LocalCartesian& lc,
                bool& have_origin) {
  std::ifstream in(file);
  if (!in.is_open()) return false;

  double lat0, lon0, alt0;
  in >> lat0 >> lon0 >> alt0;
  lc.Reset(lat0, lon0, alt0);
  have_origin = true;

  ROS_INFO("[pp_fixed] ENU origin: lat=%.15f lon=%.15f alt=%.3f",
           lat0, lon0, alt0);
  return true;
}

bool loadPath(const std::string& file,
              std::vector<std::pair<double,double>>& path_xy) {
  std::ifstream in(file);
  if (!in.is_open()) return false;

  path_xy.clear();
  std::string line;
  double x, y, z;

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    if (!(iss >> x >> y >> z)) continue;
    path_xy.emplace_back(x, y);
  }

  ROS_INFO("[pp_fixed] Path points loaded: %zu", path_xy.size());
  return !path_xy.empty();
}

} // namespace roscpp_morai_2
