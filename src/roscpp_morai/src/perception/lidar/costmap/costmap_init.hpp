static inline void initCostmap(nav_msgs::OccupancyGrid &cm,
                               const std_msgs::Header &hdr)
{
  cm.header = hdr;
  cm.info.resolution = g_params.map_resolution;

  const int w = (int)std::round(g_params.map_width / g_params.map_resolution);
  const int h = (int)std::round(g_params.map_height / g_params.map_resolution);

  cm.info.width = w;
  cm.info.height = h;

  // 로컬 costmap: 원점이 프레임(ego) 기준 (0,0)을 중앙에 두도록
  cm.info.origin.position.x = -g_params.map_width * 0.5;
  cm.info.origin.position.y = -g_params.map_height * 0.5;
  cm.info.origin.position.z = 0.0;
  cm.info.origin.orientation.w = 1.0;

  cm.data.assign(w * h, g_params.unknown_cost);
}

// Helper: world to grid conversion
static inline bool worldToGrid(const nav_msgs::OccupancyGrid &cm,
                               float x, float y,
                               int &gx, int &gy)
{
  gx = (int)std::floor((x - cm.info.origin.position.x) / cm.info.resolution);
  gy = (int)std::floor((y - cm.info.origin.position.y) / cm.info.resolution);

  if (gx < 0 || gx >= (int)cm.info.width) return false;
  if (gy < 0 || gy >= (int)cm.info.height) return false;
  return true;
}

static inline int gridIndex(const nav_msgs::OccupancyGrid &cm, int gx, int gy)
{
  if (gx < 0 || gx >= (int)cm.info.width) return -1;
  if (gy < 0 || gy >= (int)cm.info.height) return -1;
  return gy * (int)cm.info.width + gx;
}
