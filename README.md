# 25Rokey_Quoridor

## 📌 프로젝트 소개
- Quoridor 보드게임을 플레이하는 **AI 로봇 시스템** 구현
- AI가 게임 규칙을 이해하고 전략적으로 수를 계산하여,  
  로봇 팔이 실제 보드 위에서 말을 이동
- 초보자를 위한 **튜토리얼 및 연습용 AI 모드** 제공

---

## 🛠 기술 스택

### **Language**
- Python 3.10.12

### **Framework / Middleware**
- ROS 2 Humble

### **OS / Development Environment**
- Ubuntu 22.04 (ROS 2 Humble 기반)

### **AI / Vision**
- OpenAI API
- LangChain
- Ultralytics YOLO

### **Robot Hardware**
- Doosan Robotics M0609
- End-Effector (Gripper)
- Intel RealSense Depth Camera

### **Software / Library**
- Pygame
- PyAudio

### **Collaboration Tools**
- GitHub
- Slack

---

## ⚙️ 주요 기능
- 음성 인식을 통한 Quoridor 게임 실행 및 난이도 설정
- AI가 게임 규칙을 이해하고 전략적으로 수를 계산하여  
  로봇 팔이 실제 보드 위에서 말 이동
- 게임 종료 시, 보드·장벽·말을 초기 상태로 정리하는 자동 정리 시퀀스 수행

---

## 🧠 시스템 구조 / 흐름
<img width="1190" height="290" alt="System Architecture" src="https://github.com/user-attachments/assets/03de4e21-0e1d-407c-93d0-c609f3f4edb2" />

---

## 👤 담당 역할
- Quoridor 게임 로직 및 AI 전략 알고리즘 구현
- ROS 2 기반 게임 로직–로봇 제어 연동
- Game UI 제작
- 게임 종료 후 정리(Reset) 시퀀스 구조 설계
- 로봇 모션 최적화
- Object Detection 성능 최적화  
  (좌표 보정 및 Orientation 계산)
- 전체 시스템 Integration 담당

---

## 📈 결과 및 성과
- 실제 로봇 환경에서 Quoridor 게임의 **안정적인 자동 플레이** 구현
- AI 판단 → 로봇 제어 → 보드 상호작용까지의 **엔드투엔드 시스템 검증 완료**

---

## 🚀 실행 방법
- M0609 Launch 실행  
  : $ ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609
- Realsense Launch 실행  
  : $ ros2 launch realsense2_camera rs_align_depth_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 initial_reset:=true align_depth.enable:=true enable_rgbd:=true pointcloud.enable:=true
- quoridor.launch.py 실행  
  : $ ros2 launch quoridor_main quoridor.launch.py
