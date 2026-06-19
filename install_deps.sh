#!/bin/bash
# RODI 프로젝트 의존성 전체 설치 스크립트
set -e

echo "========================================"
echo "  RODI 의존성 설치 시작"
echo "========================================"

# 1. pip 및 오디오 도구 설치
echo ""
echo "[1/4] pip, mpg123, ffmpeg 설치 중..."
sudo apt-get update -qq
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    mpg123 \
    ffmpeg \
    python3-colcon-common-extensions

# 2. Nav2 전체 스택 설치
echo ""
echo "[2/4] ROS2 Nav2 전체 스택 설치 중..."
sudo apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-common \
    ros-humble-nav2-core \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-smoother \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-dwb-controller \
    ros-humble-slam-toolbox \
    ros-humble-tf2-geometry-msgs

# 3. Python 패키지 설치 (ultralytics, gtts)
echo ""
echo "[3/4] Python 패키지 설치 중 (ultralytics, gtts)..."
pip3 install --upgrade pip
pip3 install \
    ultralytics \
    gtts \
    opencv-python-headless

# 4. YOLOv8 모델 미리 다운로드
echo ""
echo "[4/4] YOLOv8n 모델 사전 다운로드 중..."
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt'); print('YOLOv8n 다운로드 완료')"

echo ""
echo "========================================"
echo "  모든 의존성 설치 완료!"
echo "  다음 명령어로 빌드하세요:"
echo "  cd ~/RODI && colcon build --symlink-install"
echo "========================================"
