mkdir -p ~/docker-morai
# 위 3개 파일을 그대로 저장
cd ~/docker-morai
chmod +x run.sh
./run.sh

# 접속 후 rosbridge 실행
docker exec -it morai-noetic bash
roslaunch rosbridge_server rosbridge_websocket.launch

