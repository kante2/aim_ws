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