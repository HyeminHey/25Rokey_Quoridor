# 25Rokey_Quoridor

## 📌 프로젝트 소개
- Quoridor 보드게임을 플레이하는 AI 로봇 시스템 구현
- AI가 게임 규칙을 이해하고 전략적으로 수를 계산하여 로봇 팔이 실제 보드 위에서 말을 이동
- 초보자를 위한 튜토리얼/연습용 AI 모드 제공
  
## 🛠 기술 스택
- **Language**
  - Python 3.10.12

- **Framework / Middleware**
  - ROS2 Humble
 
- **OS / Development Environment**
  - Ubuntu 22.04 (ROS 2 Humble 기반)

- **AI / LLM**
  - OpenAI API
  - LangChain
  - Ultralytics YOLO

- **Robot Hardware**
  - Doosan Robotics M0609
  - End-Effector (Gripper)
  - Intel Realsense Depth Camera

- **Software / Library**
  - Pygame
  - PyAudio Analysis

- **Collaboration Tools**
  - GitHub
  - Slack


## ⚙️ 주요 기능
- 음성인식으로 Quori 실행 및 난이도 설정
- AI가 게임 규칙을 이해하고 전략적으로 수를 계산하여 로봇 팔이 실제 보드 위에서 말을 이동
- 게임 종료 시 게임 시작 초기 상태로 로봇 팔이 보드, 장벽 및 말 정리

## 🧠 시스템 구조 / 흐름
<img width="1190" height="290" alt="Image" src="https://github.com/user-attachments/assets/03de4e21-0e1d-407c-93d0-c609f3f4edb2" />

## 👤 담당 역할
- Quoridor game computing algorithm과 ROS2 연동
- Game UI 제작
- 게임 종료 후 정리 시퀀스 구조 제작
- 로봇 움직임 최적화 작업
- Object Detection 최적화 작업 (x y 좌표 및 orientation)
- Integration 작업 진행

## 📈 결과 및 성과
- 성공적 작동 확인

## 🚀 실행 방법
- M0609 Launch 실행  
  : $ ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
- Realsense Launch 실행  
  : $ ros2 launch realsense2_camera rs_align_depth_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 initial_reset:=true align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true
- quoridor.launch.py 실행  
  : $ ros2 launch quoridor_main quoridor.launch.py
