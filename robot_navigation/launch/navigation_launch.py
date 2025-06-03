#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지 경로
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    robot_navigation_dir = get_package_share_directory("robot_navigation")

    # Config 파일 경로
    map_file = os.path.join(robot_navigation_dir, "config", "map.yaml")
    nav_params_file = os.path.join(
        robot_navigation_dir, "config", "tb3_nav_params.yaml"
    )
    rviz_config = os.path.join(robot_navigation_dir, "config", "tb3_nav.rviz")

    # 시작 메시지
    start_log = LogInfo(msg="통합 창고 자동화 시스템을 시작합니다!")

    # 1. Nav2 스택 (즉시 시작) - 가장 중요!
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav_params_file,
            "use_sim_time": "false",
        }.items(),
    )

    # 2. RViz (1초 후 시작)
    rviz_node = TimerAction(
        period=1.0,
        actions=[
            LogInfo(msg="RViz 시작..."),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ],
    )

    # 3. 웨이포인트 네비게이터 (3초 후 시작)
    waypoint_navigator_node = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="웨이포인트 네비게이터 시작..."),
            Node(
                package="robot_navigation",
                executable="waypoint_navigator",
                name="waypoint_navigator",
                output="screen",
            ),
        ],
    )

    # 4. 마스터 GUI (4초 후 시작)
    master_gui_node = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="마스터 GUI 시작..."),
            Node(
                package="robot_master",
                executable="robot_master",
                name="robot_master",
                output="screen",
            ),
        ],
    )

    # 5. 비전 GUI (5초 후 시작)
    vision_gui_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="비전 GUI 시작..."),
            Node(
                package="robot_vision",
                executable="robot_vision",
                name="robot_vision_gui",
                output="screen",
            ),
        ],
    )

    # 6. OCR 추론 노드 (6초 후 시작)
    ocr_node = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="OCR 시스템 시작..."),
            Node(
                package="robot_vision",
                executable="ocr_inference_node.py",
                name="ocr_inference_node",
                output="screen",
                parameters=[
                    {"use_cuda": True},
                    {"score_threshold": 0.6},
                    {"nms_threshold": 0.3},
                ],
            ),
        ],
    )

    # 완료 메시지
    ready_log = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="모든 시스템이 준비되었습니다!"),
            LogInfo(msg="사용법:"),
            LogInfo(
                msg="   1. 마스터 GUI에서 물품 ID를 입력하세요 (예: 정우경, 초코프렌즈우유)"
            ),
            LogInfo(msg="   2. '물품 검색 시작' 버튼을 클릭하세요"),
            LogInfo(msg="   3. 로봇이 자동으로 4개 거점을 순회하며 물품을 찾습니다"),
            LogInfo(
                msg="   4. 물품 발견 시 자동으로 리프트 동작을 수행하고 홈으로 복귀합니다"
            ),
            LogInfo(msg=""),
            LogInfo(
                msg="팁: RViz에서 초기 위치가 잘못되었다면 '2D Pose Estimate'로 수정하세요"
            ),
        ],
    )

    return LaunchDescription(
        [
            # 로그 메시지
            start_log,
            # 시스템들 (시간차 시작)
            nav2_bringup,  # 0초 - Nav2 스택 (가장 중요!)
            rviz_node,  # 1초 - RViz
            waypoint_navigator_node,  # 3초 - 웨이포인트 네비게이터
            master_gui_node,  # 4초 - 마스터 GUI
            vision_gui_node,  # 5초 - 비전 GUI
            ocr_node,  # 6초 - OCR 시스템
            ready_log,  # 8초 - 완료 메시지
        ]
    )
