#!/usr/bin/env bash
set -euo pipefail

# 스크립트 위치 기준으로 동작
cd "$(dirname "$0")"

# X11 접근 허용 (없어도 되지만 GUI 쓸 때 편의)
xhost +local: || true

# 빌드 & 실행
if command -v docker compose >/dev/null 2>&1; then
  docker compose build
  docker compose up -d
else
  # 구버전(플러그인 미설치)일 경우
  docker-compose build
  docker-compose up -d
fi

echo
echo "[OK] 컨테이너가 올라갔습니다: morai-noetic"
echo "쉘 접속:  docker exec -it morai-noetic bash"
echo "rosbridge: 컨테이너 안에서  roslaunch rosbridge_server rosbridge_websocket.launch"
echo
echo "※ MORAI 런처 폴더는 컨테이너에서 /mnt/MoraiLauncher_Lin (읽기전용) 로 보입니다."
