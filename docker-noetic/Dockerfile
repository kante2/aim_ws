# 더 안정적인 베이스: ros:noetic (Ubuntu 20.04)
FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

# 타임존/로케일
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales && \
    ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata && \
    locale-gen en_US.UTF-8 ko_KR.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# 데스크탑 풀 + 개발 유틸 + OpenGL 점검 도구
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    git curl wget nano vim tmux sudo bash-completion \
    build-essential cmake \
    python3-pip python3-dev \
    iputils-ping net-tools mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# 편의 bash 설정
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "export LANG=en_US.UTF-8" >> /root/.bashrc && \
    echo "export LC_ALL=en_US.UTF-8" >> /root/.bashrc

# 워크스페이스(호스트에서 마운트 예정)
WORKDIR /root/ws

# X11 공유메모리 이슈 회피
ENV QT_X11_NO_MITSHM=1

CMD ["bash"]
