#pragma once
#include <string>
#include <vector>
#include <utility>
#include <GeographicLib/LocalCartesian.hpp>

namespace roscpp_morai_2 {

// ref.txt → ENU 원점 세팅
bool loadOrigin(const std::string& file,
                GeographicLib::LocalCartesian& lc,
                bool& have_origin);

// Path.txt → (x,y) 경로 로딩
bool loadPath(const std::string& file,
              std::vector<std::pair<double,double>>& path_xy);

} // namespace roscpp_morai_2
