#1. X11 접근 허용
xhost +local:docker

#2. 빌드/ 실행
cd ~/aim_ws/docker-noetic
docker compose -f docker-compose.pc.yaml build --no-cache
docker compose -f docker-compose.pc.yaml up -d --force-recreate
docker ps -a --filter "name=aim-noetic-pc"

#3. 접속 
docker exec -it aim-noetic-pc bash
# (컨테이너 안)
source /opt/ros/noetic/setup.bash
# 네가 마운트한 워크스페이스에 catkin_ws가 있다면:
# source /root/ws/devel/setup.bash   # (catkin_make 완료된 상태일 때)
roscore


# 4. 종료/ 정리
docker compose -f docker-compose.pc.yaml down
# X11 권한 원복
xhost -local:docker
