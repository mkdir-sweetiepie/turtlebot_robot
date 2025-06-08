# TurtleBot3 스마트 물류 로봇 시스템

<div align="center">

![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)
![Platform](https://img.shields.io/badge/platform-Ubuntu%2022.04-orange.svg)
![Python](https://img.shields.io/badge/python-3.10%2B-blue.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)

**ROS2 Humble 기반 자율 창고 물류 자동화 시스템**

*4개 거점 순차 탐색 → AI 기반 물품 인식 → 자동 픽업 & 배송*

</div>

---

## 목차

- [프로젝트 개요](#-프로젝트-개요)
- [시스템 아키텍처](#-시스템-아키텍처)
- [핵심 기능](#-핵심-기능)
- [환경 구성](#-환경-구성)
- [설치 가이드](#-설치-가이드)
- [빌드 및 실행](#-빌드-및-실행)
- [사용법](#-사용법)
- [코드 구조](#-코드-구조)
- [설정 파일](#-설정-파일)
- [문제 해결](#-문제-해결)
- [개발팀](#-개발팀)

---

## 프로젝트 개요

본 프로젝트는 **TurtleBot3**를 기반으로 한 차세대 스마트 물류 창고 자동화 시스템입니다. 사용자가 Qt GUI를 통해 물품 ID를 입력하면, 로봇이 자율적으로 4개의 사전 정의된 거점을 순회하며 **딥러닝 기반 OCR 시스템**으로 물품을 식별하고, 발견 시 자동으로 픽업하여 배송 완료까지 수행하는 무인 물류 솔루션입니다.

### 프로젝트 목표

- **자율화**: 사용자 입력 후 모든 과정이 자동으로 진행
- **정확한 인식**: 딥러닝 OCR 모델을 통한 높은 정확도의 물품 식별
- **효율적 탐색**: 최적화된 경로로 4개 거점 순차 방문
- **실시간 모니터링**: 전 과정의 상태를 실시간으로 추적

---

## 시스템 아키텍처

### ROS2 노드 구조도

```
┌─────────────────────────────────────────────────────────────┐
│                      사용자 인터페이스 레이어                    │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐                 │
│  │   Master GUI    │    │   Vision GUI    │                 │
│  │  (물품 ID 입력)   │    │  (OCR 상태 표시)  │                 │
│  └─────────────────┘    └─────────────────┘                 │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                        제어 레이어                            │
├─────────────────────────────────────────────────────────────┤
│┌─────────────────┐    ┌─────────────────┐    ┌────────────┐ │
││  robot_master   │◄──►│robot_navigation │◄──►│robot_vision│ │
││  (메인 제어)      │    │ (Nav2 + 경로)    │    │ (OCR 인식)  │ │
││                 │    │                 │    │            │ │
││ • 상태 관리       │    │ • SLAM          │    │ • 카메라    │ │
││ • 리프트 제어     │    │ • 웨이포인트       │    │ • OCR 추론  │ │
││ • 통합 로그       │    │ • 충돌 회피       │    │ • 텍스트 매칭│ │
│└─────────────────┘    └─────────────────┘    └────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                      하드웨어 레이어                           │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │  TurtleBot3  │  │   Dynamixel  │  │   USB Camera     │   │
│  │   Hardware   │  │     Lift     │  │  + OCR Model     │   │
│  └──────────────┘  └──────────────┘  └──────────────────┘   │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Gazebo Simulation                       │   │
│  │         (개발 및 테스트용 가상 환경)                      │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### 워크플로우 다이어그램

```
        시작
         ↓
    [사용자 물품 ID 입력]
         ↓
    [시스템 초기화]
         ↓
    ┌─ [거점 1 이동] ────→ [OCR 스캔] ────→ [일치?] ──Yes──┐
    │                                        │         │
    ├─ [거점 2 이동] ────→ [OCR 스캔] ────→ [일치?] ──Yes──┤
    │                                        │         │
    ├─ [거점 3 이동] ────→ [OCR 스캔] ────→ [일치?] ──Yes──┤
    │                                        │         │
    └─ [거점 4 이동] ────→ [OCR 스캔] ────→ [일치?] ──Yes──┤
                                             │         │
                                        [No] │         │
                                             ↓         │
                                       [탐색 실패]       │
                                             ↓         │
                                         [종료]         │
                                                       │
                                      ┌────────────────┘
                                      ↓
                                [물품 발견!]
                                      ↓
                                [180도 회전]
                                      ↓
                                 [자동 후진]
                                      ↓
                               [리프트 올리기]
                                      ↓
                                [홈 위치 복귀]
                                      ↓
                               [리프트 내리기]
                                      ↓
                                [배송 완료]
```

---

## 핵심 기능

### 🤖 자율 네비게이션
- **SLAM & Mapping**: Cartographer 기반 실시간 지도 생성
- **경로 계획**: Nav2 스택을 활용한 최적 경로 계산
- **장애물 회피**: 동적 장애물 감지 및 회피
- **정밀 위치 제어**: 웨이포인트 기반 정확한 위치 이동

### AI 기반 물품 인식
- **딥러닝 OCR**: PyTorch 기반 텍스트 인식 모델
- **실시간 처리**: 카메라 영상을 실시간으로 분석
- **고정밀 매칭**: Levenshtein 거리 기반 텍스트 유사도 계산
- **신뢰도 평가**: 인식 결과의 신뢰도 점수 제공

### 작업 자동화
- **순차 탐색**: 4개 거점을 체계적으로 방문
- **조건부 동작**: 물품 발견 시 자동 픽업 루틴 실행
- **리프트 제어**: Dynamixel 기반 정밀 리프트 동작
- **홈 복귀**: 작업 완료 후 자동 시작점 복귀

### 모니터링 & 제어
- **Qt GUI**: 직관적인 사용자 인터페이스
- **실시간 로그**: 시스템 전반의 상태 모니터링
- **상태 추적**: 각 단계별 진행 상황 표시
- **오류 처리**: 예외 상황 감지 및 자동 복구

---

## 환경 구성

### 필수 시스템 요구사항

| 구분 | 요구사항 |
|------|----------|
| **운영체제** | Ubuntu 22.04 LTS |
| **ROS 버전** | ROS2 Humble |
| **Python** | 3.10+ |
| **메모리** | 최소 8GB RAM |
| **GPU** | CUDA 지원 GPU (권장) |
| **저장공간** | 최소 20GB |

### 하드웨어 구성

- **TurtleBot3 Burger/Waffle Pi**
- **Dynamixel 서보모터** (리프트용)
- **USB 카메라** (물품 인식용)
- **Raspberry Pi 4** (로봇 제어용)
- **개발용 PC** (원격 제어용)

---

## 설치 가이드

### 1단계: ROS2 Humble 설치

```bash
# 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 GPG 키 및 저장소 추가
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install python3-argcomplete
```

### 2단계: 의존성 패키지 설치

```bash
# 네비게이션 관련 패키지
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

# TurtleBot3 관련 패키지
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-dynamixel-sdk

# Gazebo 시뮬레이션
sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-ros-gz
sudo apt install ros-humble-gz-ros2-control

# Qt 및 GUI 개발 도구
sudo apt install qtcreator qtbase5-dev qt5-qmake
sudo apt install libqt5widgets5 libqt5gui5 libqt5core5a

# 비전 및 OCR 관련
sudo apt install python3-opencv
sudo apt install python3-pil
sudo apt install libzbar-dev
sudo apt install libyaml-cpp-dev

# 딥러닝 라이브러리
sudo apt install python3-pip
pip3 install torch torchvision
pip3 install easyocr
pip3 install albumentations
pip3 install python-Levenshtein

# 빌드 도구
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### 3단계: 워크스페이스 생성 및 소스코드 다운로드

```bash
# 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스 생성
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/

# 필수 패키지 클론
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/mkdir-sweetiepie/turtlebot3.git
git clone https://github.com/mkdir-sweetiepie/turtlebot_robot.git
git clone https://github.com/mkdir-sweetiepie/turtlebot3_simulations.git

# 의존성 해결
cd ~/turtlebot3_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4단계: 환경 변수 설정

```bash
# ~/.bashrc 파일에 다음 내용 추가
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=12' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

# 변경사항 적용
source ~/.bashrc
```

### 5단계: 딥러닝 모델 다운로드

```bash
# 모델 저장 디렉토리 생성
mkdir -p ~/turtlebot3_ws/src/turtlebot_robot/robot_vision/models/

# OCR 모델 다운로드 (Google Drive에서 다운로드 후 이동)
# https://drive.google.com/drive/folders/1EiXxVB3fff_02BRlWfy7Ie57Vk5ZDHDy
# final_model.pth 파일을 다운로드하여 위 경로에 저장
```

---

## 빌드 및 실행

### 빌드 과정

```bash
# 워크스페이스 이동
cd ~/turtlebot3_ws

# 패키지 빌드 (처음 빌드 시 10-15분 소요)
colcon build --symlink-install

# 설치 파일 소싱
source install/setup.bash
```

### 실행 방법

#### 실제 하드웨어 실행

**1단계: TurtleBot3 하드웨어 준비**

```bash
# WiFi 네트워크 연결: BARAM_IPTIME5G
# SSH로 Raspberry Pi 접속
ssh ubuntu@192.168.0.2
# 비밀번호 입력

# TurtleBot3 기본 노드 실행
start
# 또는
ros2 launch turtlebot3_bringup robot.launch.py

# 새 터미널에서 카메라 실행
cam
# 또는
ros2 launch usb_cam camera.launch.py
```

**2단계: 개발용 PC에서 제어 시스템 실행**

```bash
# 터미널 1: 통합 네비게이션 시스템
ros2 launch robot_navigation navigation_launch.py

# 터미널 2: 메인 제어 시스템
ros2 run robot_master robot_master

# 터미널 3: 비전 시스템 (선택사항, GUI 필요시)
ros2 run robot_vision robot_vision
```

#### 🎮 시뮬레이션 실행

```bash
# 터미널 1: Gazebo 시뮬레이션
ros2 launch turtlebot3_gazebo empty_world.launch.py

# 터미널 2: 통합 네비게이션 시스템
ros2 launch robot_navigation navigation_launch.py

# 터미널 3: 메인 제어 시스템
ros2 run robot_master robot_master
```

#### 📊 통합 실행 (권장)

모든 시스템을 한 번에 실행:

```bash
# 모든 노드를 시간차로 자동 실행
ros2 launch robot_master robot_master.launch.py target_item:="초코프렌즈우유"

# 또는 시뮬레이션 모드로
ros2 launch robot_master robot_master.launch.py use_sim:=true target_item:="정우경"
```

---

## 사용법

### 기본 사용 절차

1. **시스템 시작**
   ```bash
   ros2 launch robot_navigation navigation_launch.py
   ```

2. **GUI에서 물품 ID 입력**
   - Master GUI 창에서 찾고자 하는 물품 ID 입력
   - 예시: "초코프렌즈우유", "정우경", "사과"

3. **물품 검색 시작**
   - "물품 검색 시작" 버튼 클릭
   - 시스템이 자동으로 4개 거점 순회 시작

4. **자동 작업 모니터링**
   - 로봇이 각 거점으로 이동하며 OCR 스캔 수행
   - 물품 발견 시 자동으로 픽업 루틴 실행

5. **결과 확인**
   - 성공: 홈 위치로 복귀 후 배송 완료
   - 실패: 모든 거점 탐색 후 "물품 없음" 보고

### GUI 조작 가이드

#### Master GUI
- **물품 ID 입력창**: 검색할 물품의 ID 입력
- **검색 시작/중지**: 미션 시작 및 긴급 중지
- **상태 표시**: 현재 작업 상태 실시간 표시
- **로그 창**: 시스템 전체 로그 메시지 표시

#### Vision GUI (선택사항)
- **카메라 영상**: 실시간 카메라 화면
- **OCR 결과**: 인식된 텍스트 및 신뢰도
- **FPS 표시**: 카메라 및 처리 성능 모니터링

### 커맨드라인 제어

```bash
# 특정 물품 검색 시작
ros2 topic pub /item_search_request std_msgs/msg/String "data: '초코프렌즈우유'" -1

# 미션 취소
ros2 topic pub /item_search_request std_msgs/msg/String "data: 'CANCEL'" -1

# OCR 수동 활성화/비활성화
ros2 topic pub /ocr_control std_msgs/msg/Bool "data: true" -1
ros2 topic pub /ocr_control std_msgs/msg/Bool "data: false" -1

# 시스템 상태 확인
ros2 topic echo /system_log
```

---

## 코드 구조

### 프로젝트 디렉토리 구조

```
turtlebot3_ws/
├── src/
│   ├── robot_master/                    # 메인 제어 패키지
│   │   ├── include/robot_master/
│   │   │   ├── main_window.hpp         # GUI 인터페이스
│   │   │   ├── qnode.hpp               # ROS2 통신 노드
│   │   │   └── lift_controller.hpp     # 리프트 제어
│   │   ├── src/
│   │   │   ├── main.cpp                # 메인 실행 파일
│   │   │   ├── main_window.cpp         # GUI 구현
│   │   │   ├── qnode.cpp               # 노드 로직
│   │   │   └── lift_controller.cpp     # 리프트 제어 로직
│   │   ├── launch/
│   │   │   └── robot_master.launch.py  # 통합 실행 파일
│   │   └── ui/
│   │       └── mainwindow.ui           # Qt UI 파일
│   │
│   ├── robot_vision/                    # 비전 처리 패키지
│   │   ├── include/robot_vision/
│   │   │   ├── main_window.hpp         # 비전 GUI
│   │   │   └── qnode.hpp               # 비전 노드
│   │   ├── src/
│   │   │   ├── main.cpp                # 비전 메인
│   │   │   ├── main_window.cpp         # 비전 GUI 로직
│   │   │   └── qnode.cpp               # 이미지 처리
│   │   ├── scripts/
│   │   │   └── ocr_inference_node.py   # OCR 추론 노드
│   │   └── models/
│   │       └── final_model.pth         # 딥러닝 모델
│   │
│   ├── robot_navigation/                # 네비게이션 패키지
│   │   ├── include/robot_navigation/
│   │   │   └── waypoint_navigator.hpp  # 웨이포인트 헤더
│   │   ├── src/
│   │   │   └── waypoint_navigator.cpp  # 네비게이션 로직
│   │   ├── launch/
│   │   │   └── navigation_launch.py    # 네비게이션 실행
│   │   └── config/
│   │       ├── nav2_params.yaml        # Nav2 설정
│   │       └── waypoints.yaml          # 웨이포인트 정의
│   │
│   ├── robot_msgs/                      # 커스텀 메시지
│   │   ├── msg/
│   │   │   ├── VisionMsg.msg           # 비전 결과
│   │   │   ├── LogMessage.msg          # 로그 메시지
│   │   │   ├── OCRRequest.msg          # OCR 요청
│   │   │   └── OCRResult.msg           # OCR 결과
│   │   ├── srv/
│   │   │   └── ItemInfo.srv            # 물품 정보 서비스
│   │   └── action/
│   │       └── PreciseControl.action   # 정밀 제어 액션
│   │
│   └── turtlebot3_simulations/          # 시뮬레이션 환경
│       ├── turtlebot3_gazebo/
│       │   ├── worlds/
│       │   │   └── empty_world.world   # Gazebo 월드
│       │   └── launch/
│       │       └── empty_world.launch.py
│       └── ...
```
---

## 👥 개발팀

### 🏆 프로젝트 팀

| 이름 | 담당 영역 | GitHub |
|------|-----------|---------|
| **정우경** | OCR 모델, 이미지 처리 | [@Jeongwoogyeong](https://github.com/wooujoa) |
| **오가현** | 시뮬레이션 | [@Ohgahyun](https://github.com/gahyun0425) |
| **임동균** | 리프트 시스템, Dynamixel | [@Yimdonggyun](https://github.com/David02345) |
| **최상준** | 네비게이션 시스템 | [@Choisangjun](https://github.com/Samuel3740) |
| **홍지현** | Qt GUI, 시스템 통합 | [@Hongjihyeon](https://github.com/mkdir-sweetiepie) |

---

## 라이선스

```
Copyright 2025 TurtleBot3 Logistics Robot Team

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## 지원 및 문의

### 연락처
- **프로젝트 이슈**: [GitHub Issues](https://github.com/mkdir-sweetiepie/turtlebot_robot/issues)
- **기술 토론**: [GitHub Discussions](https://github.com/mkdir-sweetiepie/turtlebot_robot/discussions)
- **위키**: [프로젝트 Wiki](https://github.com/mkdir-sweetiepie/turtlebot_robot/wiki)

### 참고 자료
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [OpenCV Documentation](https://docs.opencv.org/)

---

<div align="center">

![Made with Love](https://img.shields.io/badge/Made%20with-❤️-red.svg)
![Powered by ROS2](https://img.shields.io/badge/Powered%20by-ROS2-blue.svg)

</div>
