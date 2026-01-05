1.
roscore

2. bridge
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090

3. gps follow
source devel/setup.bash
source /root/aim_ws/devel/setup.bash
rosrun roscpp_morai hybrid

-------------------------------------------


# 1. 코드 수정 (방법 1, 2, 3 중 선택)
# 방법 1: 가장 쉬움 (RViz에서만 조정)
# 방법 2: 약간 더 안전
# 방법 3: 가장 명확함 (추천!)

# 2. 빌드
cd /root/aim_ws && catkin_make

# 3. 환경 설정
source devel/setup.bash

# 4. 실행
rosrun roscpp_morai lidar_aabb_ver2          # 터미널 1
rosrun roscpp_morai only_lattice_planner_node_ver2  # 터미널 2
rviz             

# 5.
source devel/setup.bash
rosrun roscpp_morai lidar_aabb_costmap_node_ver3
-------------------------------------------












4. lattice_planner_morai
source /root/aim_ws/devel/setup.bash
rosrun roscpp_morai only_lattice_planner_node_ver2




------------------------------------------------------------------------------------

4. path Recorder
source devel/setup.bash && rosrun roscpp_morai path_recorder


5. aabb (not good * have to fix using cluster_car_detection ** WON) + costmap 
source /root/aim_ws/devel/setup.bash
roslaunch roscpp_morai launch_aabb_costmap_node.launch

source /root/aim_ws/devel/setup.bash
rosrun roscpp_morai lidar_aabb_ver2


6. AABB GOOD ** cost map fixed final?
# 1. Source the environment
source /root/aim_ws/devel/setup.bash

# 2. Run the node
rosrun roscpp_morai lidar_aabb_ver2





7.
# 빌드
cd /root/aim_ws && catkin_make

# 실행
rosrun roscpp_morai only_lattice_planner_node

# RViz 시각화
rviz
  → Add Path: /local_path (파란색)
  → Add Marker: /candidate_paths (녹색/빨강)



===================================


source devel/setup.bash
roslaunch lattice_planner_morai lidar_costmap_generator.launch

source devel/setup.bash
rosrun roscpp_mo


===================================


cd ~/Downloads/MoraiLauncher_Lin

__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
__VK_LAYER_NV_optimus=NVIDIA_only \
./MoraiLauncher_Lin.x86_64

===================================

how to use vendor_fix.sh
1️⃣ 서브모듈 해제
   git submodule deinit -f <패키지>

2️⃣ .gitmodules 정리
   git config -f .gitmodules --remove-section

3️⃣ 캐시에서 제거
   git rm --cached <패키지>

4️⃣ 중첩 .git 삭제
   rm -rf <패키지>/.git

5️⃣ .gitignore 예외 추가
   echo "!<패키지>/**" >> .gitignore

6️⃣ 자동 커밋 & 푸시
   git commit -m "Vendor packages: ..."
   git push origin <현재브랜치>