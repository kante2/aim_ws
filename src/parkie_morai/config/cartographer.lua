include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 프레임 이름
  map_frame = "map",
  tracking_frame = "base_link",        -- IMU 없다면 base_link로 OK
  published_frame = "odom",            -- 로컬 포즈를 어느 프레임 기준으로 낼지
  odom_frame = "odom",

  -- odom을 외부(EKF/휠)가 퍼블리시한다고 가정
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  -- 센서/입력
  use_odometry = true,                 -- nav_msgs/Odometry 사용 시 true
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- 타이밍/샘플링
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 라이다 설정
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 5.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
MAP_BUILDER.num_background_threads = 4

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.global_sampling_ratio = 0.001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.001
POSE_GRAPH.optimize_every_n_nodes = 50

return options
