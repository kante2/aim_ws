static void filterByHeight(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                           float zmin, float zmax)
{
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(zmin, zmax);
  pass.filter(*cloud);
}

static void filterByRange(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                          float range_max)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
  out->reserve(cloud->size());
  for (auto &p : cloud->points) {
    float r = std::sqrt(p.x*p.x + p.y*p.y);
    if (r <= range_max) out->push_back(p);
  }
  out->width = (uint32_t)out->size();
  out->height = 1;
  cloud.swap(out);
}

static void voxelDownsample(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float leaf)
{
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*cloud);
}

static pcl::PointCloud<pcl::PointXYZI>::Ptr removeEgoROI(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float s)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);
  out->reserve(cloud->size());
  for (auto &p : cloud->points) {
    if (std::fabs(p.x) <= s && std::fabs(p.y) <= s) continue;
    out->push_back(p);
  }
  out->width = (uint32_t)out->size();
  out->height = 1;
  return out;
}

static pcl::PointCloud<pcl::PointXYZI>::Ptr ransacRemoveGround(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
  float dist_thresh, float eps_angle_deg, int max_iter,
  pcl::ModelCoefficients::Ptr coeff_out = nullptr)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>);

  if (!cloud || cloud->empty()) {
    return out;
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(dist_thresh);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));  // z축에 수직(=지면)
  seg.setEpsAngle(static_cast<float>(eps_angle_deg * M_PI / 180.0));
  seg.setMaxIterations(max_iter);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (coeff_out) {
    *coeff_out = *coefficients;
  }

  if (inliers->indices.empty()) {
    *out = *cloud;
    return out;
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);   // outlier(비지면)만 남김
  extract.filter(*out);

  return out;
}

static std::vector<pcl::PointIndices> euclideanCluster(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
  float tol, int min_sz, int max_sz)
{
  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setInputCloud(cloud);
  ec.setSearchMethod(kdtree);
  ec.setClusterTolerance(tol);
  ec.setMinClusterSize(min_sz);
  ec.setMaxClusterSize(max_sz);

  std::vector<pcl::PointIndices> indices;
  ec.extract(indices);
  return indices;
}

static std::vector<Detection> buildDetections(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
  const std::vector<pcl::PointIndices> &clusters,
  const ros::Time &stamp)
{
  std::vector<Detection> dets;
  dets.reserve(clusters.size());

  for (int i = 0; i < (int)clusters.size(); ++i) {
    const auto &idxs = clusters[i].indices;
    if (idxs.empty()) continue;

    double sum_x = 0, sum_y = 0, sum_z = 0;

    float min_x =  std::numeric_limits<float>::infinity();
    float min_y =  std::numeric_limits<float>::infinity();
    float min_z =  std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();

    int count = 0;
    for (int idx : idxs) {
      if (idx < 0 || idx >= (int)cloud->size()) continue;
      const auto &p = cloud->points[idx];

      sum_x += p.x; sum_y += p.y; sum_z += p.z;

      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      min_z = std::min(min_z, p.z);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
      max_z = std::max(max_z, p.z);
      ++count;
    }
    if (count == 0) continue;

    Detection d;
    d.id = i;
    d.stamp = stamp;
    d.num_points = count;

    const float cx = (float)(sum_x / count);
    const float cy = (float)(sum_y / count);
    const float cz = (float)(sum_z / count);

    d.centroid = Eigen::Vector3f(cx, cy, cz);
    d.min_pt = Eigen::Vector3f(min_x, min_y, min_z);
    d.max_pt = Eigen::Vector3f(max_x, max_y, max_z);
    d.size   = d.max_pt - d.min_pt;
    d.range  = std::sqrt(cx*cx + cy*cy);
    dets.push_back(d);
  }

  return dets;
}

static bool isCavCandidate(const Detection &d)
{
  const float dx = d.size.x();
  const float dy = d.size.y();
  const float dz = d.size.z();

  if (d.num_points < g_params.cav_min_points) return false;
  if (dz < g_params.cav_dz_min || dz > g_params.cav_dz_max) return false;
  if (dx < g_params.cav_dx_min || dx > g_params.cav_dx_max) return false;
  if (dy < g_params.cav_dy_min || dy > g_params.cav_dy_max) return false;
  return true;
}
static void publishColoredClustersAll(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
  const std::vector<pcl::PointIndices> &clusters,
  const std_msgs::Header &hdr)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.reserve(cloud->points.size());
  colored->is_dense = true;

  int cluster_id = 0;
  for (const auto &c : clusters) {
    uint8_t r=0,g=0,b=0;
    switch (cluster_id % 6) {
      case 0: r=255; g= 80; b= 80; break;
      case 1: r= 80; g=255; b= 80; break;
      case 2: r= 80; g= 80; b=255; break;
      case 3: r=255; g=255; b= 80; break;
      case 4: r=255; g= 80; b=255; break;
      case 5: r= 80; g=255; b=255; break;
    }

    for (int idx : c.indices) {
      if (idx < 0 || idx >= (int)cloud->size()) continue;
      const auto &src = cloud->points[idx];
      pcl::PointXYZRGB p;
      p.x = src.x; p.y = src.y; p.z = src.z;
      p.r = r; p.g = g; p.b = b;
      colored->points.push_back(p);
    }
    cluster_id++;
  }

  colored->width = (uint32_t)colored->points.size();
  colored->height = 1;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*colored, msg);
  msg.header = hdr;
  pub_cluster_all.publish(msg);
}