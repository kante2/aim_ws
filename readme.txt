AIM_2025_2 cpp 개발과정 진행을 위한 ws


# ----------------------------------------------------


MORAI-DriveExample_ROS
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src && catkin_init_workspace
$ git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_ROS.git
$ cd MORAI-DriveExample_ROS
$ git submodule update --init --recursive
$ sudo chmod -R a+x morai_standard/
$ find -name 'requirements.txt' | xargs -L 1 sudo pip install -U -r
$ cd ~/catkin_ws
$ rosdep install --from-paths . --ignore-src -r -y
$ catkin_make
$ source devel/setup.bash


# -----------------
sudo apt-get install ros-noetic-rosbridge-server

roslaunch rosbridge_server rosbridge_websocket.launch


wk1.

과제

1. Framework 만들기
- 초기 틀 만들기 (ROS Package 공부 철저히 한 이후에, 진행)
* 원하는대로 만들어도 되지만 분류에 따라서 cpp, h, hpp 나누기
-> 만든 틀 가지고 추후 세미나, 과제 진행 예정

​

2. 좌표계공부
- wgs84(gps) / ecef / utm, utm52n / enu, ned / body(LiDAR)

3. tracking 
- pure pursuit 이용
-> morai ego 이용
-> path도 ego 이용해서 만들기

morai를 사용하기 위해 host에 vulkan이라는걸 설치해야 한다고 한다.
vulkaninfo를 하면 vulkan version체크가 되면 확인이 된다.
-morai sim 공홈 설치 가이드 참고
