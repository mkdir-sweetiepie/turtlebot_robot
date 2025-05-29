#!/bin/bash

# 스마트 물류 창고 자동화 시스템 실행 스크립트
# 파일명: run_warehouse_system.sh

echo "================================================"
echo "  스마트 물류 창고 자동화 시스템"
echo "  ROS2 Humble 기반 터틀봇 물품 검색 시스템"
echo "================================================"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 기본 설정
DEFAULT_ITEM="정우경"
ITEM=${1:-$DEFAULT_ITEM}
SIM_MODE=${2:-"false"}

echo -e "${BLUE}검색할 물품:${NC} ${GREEN}$ITEM${NC}"
echo -e "${BLUE}시뮬레이션 모드:${NC} $SIM_MODE"
echo ""

# ROS 환경 확인
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}오류: ROS 환경이 설정되지 않았습니다.${NC}"
    echo "다음 명령을 실행하세요:"
    echo "source /opt/ros/humble/setup.bash"
    echo "source ~/turtlebot3_ws/install/setup.bash"
    exit 1
fi

echo -e "${GREEN}ROS $ROS_DISTRO 환경 확인됨${NC}"

# 필수 패키지 확인
echo -e "${YELLOW}필수 패키지 확인 중...${NC}"

packages=("robot_master" "robot_vision" "robot_navigation" "robot_msgs")
missing_packages=()

for package in "${packages[@]}"; do
    # 2>/dev/null로 stderr 숨기고, 파이프 오류 방지
    if ! ros2 pkg list 2>/dev/null | grep -q "^$package\$" 2>/dev/null; then
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -ne 0 ]; then
    echo -e "${RED}오류: 다음 패키지들이 설치되지 않았습니다:${NC}"
    for pkg in "${missing_packages[@]}"; do
        echo "  - $pkg"
    done
    echo ""
    echo -e "${YELLOW}다음 명령으로 빌드해주세요:${NC}"
    echo "cd ~/turtlebot3_ws"
    echo "colcon build --packages-select ${missing_packages[*]}"
    exit 1
fi

echo -e "${GREEN}모든 필수 패키지가 확인되었습니다.${NC}"

# 실행 모드 선택
echo ""
echo "실행 모드를 선택하세요:"
echo "1) 전체 시스템 자동 실행 (권장)"
echo "2) 개별 노드 수동 실행"
echo "3) 시뮬레이션 모드"
echo ""
read -p "선택 (1-3): " mode

case $mode in
    1)
        echo -e "${GREEN}전체 시스템 자동 실행을 시작합니다...${NC}"
        echo ""
        echo -e "${YELLOW}주의: 다음 사항을 미리 확인해주세요:${NC}"
        echo "- 터틀봇이 켜져 있고 네트워크에 연결되어 있는지"
        echo "- 카메라가 정상 작동하는지"
        echo "- Nav2 지도가 로드되어 있는지"
        echo ""
        read -p "계속하시겠습니까? (y/N): " confirm
        
        if [[ $confirm =~ ^[Yy]$ ]]; then
            echo -e "${GREEN}시스템을 시작합니다...${NC}"
            ros2 launch robot_master robot_master.launch.py target_item:=$ITEM use_sim:=$SIM_MODE
        else
            echo "실행이 취소되었습니다."
            exit 0
        fi
        ;;
    
    2)
        echo -e "${YELLOW}개별 노드 수동 실행 가이드:${NC}"
        echo ""
        echo "다음 순서로 각각 새 터미널에서 실행하세요:"
        echo ""
        echo -e "${GREEN}1. 로봇 마스터:${NC}"
        echo "   ros2 run robot_master robot_master"
        echo ""
        echo -e "${GREEN}2. 비전 시스템:${NC}"
        echo "   ros2 run robot_vision robot_vision"
        echo "   ros2 run robot_vision ocr_inference_node.py"
        echo ""
        echo -e "${GREEN}3. 네비게이션:${NC}"
        echo "   ros2 launch robot_navigation navigation_launch.py"
        echo ""
        echo -e "${GREEN}4. 물품 검색:${NC}"
        echo "   ros2 run robot_navigation waypoint_navigator $ITEM"
        echo ""
        ;;
    
    3)
        echo -e "${GREEN}시뮬레이션 모드를 시작합니다...${NC}"
        echo ""
        echo "Gazebo 시뮬레이션을 먼저 실행합니다..."
        gnome-terminal -- bash -c "ros2 launch robot_navigation navigation_launch.py; exec bash" &
        
        echo "5초 후 시스템을 시작합니다..."
        sleep 5
        
        ros2 launch robot_master robot_master.launch.py target_item:=$ITEM use_sim:=true
        ;;
    
    *)
        echo -e "${RED}잘못된 선택입니다.${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}스마트 물류 창고 자동화 시스템 실행 완료!${NC}"
echo "시스템 상태는 robot_master GUI에서 확인할 수 있습니다."