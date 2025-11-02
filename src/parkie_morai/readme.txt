roslaunch gmapping slam_gmapping_pr2.launch

# cartographer 로 맵을 생성한다. 
# 워크스페이스 환경 로드
source /root/ws/devel/setup.bash

# Cartographer + OccupancyGrid + RViz 실행
roslaunch parkie_morai cartographer.launch open_rviz:=true use_sim_time:=true scan_topic:=/scan


# 맵이 그려졌으면 
rosrun map_server map_saver -f /root/ws/src/parkie_morai/maps/cartographer_map
으로 맵을 저장한다 --> cartographer_map.pgm + cartographer_map.yaml 생성.