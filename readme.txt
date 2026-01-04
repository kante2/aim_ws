1.
roscore

2.
source devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090

3.
source devel/setup.bash
source /root/aim_ws/devel/setup.bash
rosrun roscpp_morai hybrid


4. path Recorder
source devel/setup.bash && rosrun roscpp_morai path_recorder


5. aabb (not good * have to fix using cluster_car_detection ) + costmap 
source /root/aim_ws/devel/setup.bash
roslaunch roscpp_morai launch_aabb_costmap_node.launch

source /root/aim_ws/devel/setup.bash
rosrun roscpp_morai lidar_aabb_ver2


6. AABB GOOD 
source /root/aim_ws/devel/setup.bash
rosrun roscpp_morai cluster_car_detection



===================================


source devel/setup.bash
roslaunch lattice_planner_morai lidar_costmap_generator.launch


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