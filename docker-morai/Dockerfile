# ROS1 Noetic + GUI + rosbridge
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# 기본 도구 + rosbridge + GUI에 필요한 라이브러리들
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-server \
    ros-noetic-tf2-tools \
    net-tools iproute2 iputils-ping dnsutils \
    python3-pip git vim less \
    libgl1-mesa-glx libglib2.0-0 libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

# catkin 워크스페이스 기본 디렉토리
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root

# ROS 환경 자동 로드
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# 컨테이너 기본 진입은 bash
CMD ["/bin/bash"]
