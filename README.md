# RODI — 대형 마트 미아 방지 에스코트 로봇
> ROS2 Humble · Gazebo Classic · YOLOv8 · Nav2
---
## 팀 정보
| 항목 | 내용 |
|---|---|
| 팀명 | RODI팀 |
| 팀원 | 이승현, 양시은 |
---
## 프로젝트 소개
마트 특성상 아이는 항상 보호자와 동행하는 것이 정상 상태임을 전제로, 혼자 감지된 아이를 미아로 즉시 판단하여 고객센터까지 에스코트하는 자율주행 로봇입니다.

**시나리오 흐름:**
1. 로봇이 마트 내부를 자율 순찰
2. YOLOv8 + Depth 카메라로 어린이 감지
3. 보호자가 5초 이상 주변에 없으면 미아 판정
4. TTS 음성 안내와 함께 고객센터까지 에스코트
5. 도착 후 직원 인계 → 순찰 재개

---
## 역할 분담
| 구분 | 이승현 | 양시은 |
|---|---|---|
| 담당 | 자율주행 + 환경 | 감지 + 판단 + 안내 |
| 세부 내용 | Gazebo 맵 제작, Nav2 순찰 주행, 감지 위치 접근, 에스코트 + 복귀 주행 | YOLOv8 어린이 감지, 보호자 부재 판단 로직, Depth 카메라 거리 추정, TTS 음성 안내, State Machine 통합 |
| 공통 | ROS2 기초 학습, 통합 테스트, 발표 준비 | |

---
## 데모 영상
[![YouTube](https://img.shields.io/badge/YouTube-데모영상-red)](https://www.youtube.com/링크입력)

> **📢 양해 말씀드립니다**
> 발표 영상 녹화 당시 팀원(양시은) 컴퓨터 기기 문제로 인해 화면 내 음성 녹음이 정상적으로 이루어지지 않았습니다.
> 그러나 **TTS(gTTS) 음성 안내 기능 자체는 정상 동작**하고 있으며, 실제 시뮬레이션 환경에서 한국어 음성 출력이 잘 작동함을 확인하였습니다.
> 불편을 드려 죄송합니다. 양해해 주시면 감사하겠습니다.

---
## 기술 스택
| 구성 요소 | 사용 기술 |
|---|---|
| 시뮬레이터 | Gazebo Classic |
| 지도 생성 | SLAM Toolbox |
| 자율 주행 | Nav2 (NavFn Planner) |
| 사람 감지 | YOLOv8n (Ultralytics) |
| 어린이 판별 | Depth Camera + 카메라 내부 파라미터 |
| 음성 안내 | gTTS (Google TTS, 한국어) |
| 좌표 변환 | TF2 |
| 상태 관리 | ROS2 토픽 기반 State Machine |

---
## 빌드 및 실행 방법

### 1단계 — 빌드 (터미널 1개)

```bash
cd ~/RODI
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH=~/RODI/src/myrobot/models:$GAZEBO_MODEL_PATH
colcon build --symlink-install
source install/setup.bash
```

> **💡 참고:** `--symlink-install`로 빌드하면 Python 노드(`guardian_logic`, `model_mover` 등)는 소스 수정 시 **재빌드 없이 바로 반영**됩니다. world 파일도 Gazebo 재시작만으로 적용됩니다.

---

### 2단계 — 실행 (터미널 3개)

**터미널 1 — Gazebo 시뮬레이터**
```bash
source /opt/ros/humble/setup.bash && source ~/RODI/install/setup.bash
export GAZEBO_MODEL_PATH=~/RODI/src/myrobot/models:$GAZEBO_MODEL_PATH
ros2 launch myrobot my_world.launch.py
```

**터미널 2 — Nav2 + RViz**
```bash
source /opt/ros/humble/setup.bash && source ~/RODI/install/setup.bash
ros2 launch myrobot nav2.launch.py
```

**터미널 3 — 비전 파이프라인**
```bash
source /opt/ros/humble/setup.bash && source ~/RODI/install/setup.bash
ros2 launch myrobot vision.launch.py
```

---
## AI 사용 여부
본 프로젝트 개발 과정에서 **Claude (Anthropic)** 를 활용하였습니다.
- `myrobot_vision` 패키지 코드 작성 지원 (child_detector, guardian_logic, child_response, patrol, model_mover, tts 노드)
- 디버깅 및 파라미터 튜닝 (`CHILD_HEIGHT_M`, `grace_sec`, `alert_hold_sec` 등)
- Nav2 설정 및 launch 파일 구성

---
## 참고 자료
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Nav2 공식 문서](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [gTTS (Google Text-to-Speech)](https://gtts.readthedocs.io/)
- [Gazebo Classic](https://classic.gazebosim.org/)
- [cv_bridge ROS2](https://github.com/ros-perception/vision_opencv)
- [message_filters ApproximateTimeSynchronizer](https://docs.ros.org/en/humble/p/message_filters/)
