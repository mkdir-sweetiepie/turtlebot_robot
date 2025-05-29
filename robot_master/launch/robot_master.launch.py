#!/usr/bin/env python3
"""
스마트 물류 창고 자동화 시스템 통합 Launch 파일
robot_master/launch/robot_master.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Launch 파라미터 선언
    target_item_arg = DeclareLaunchArgument(
        "target_item", default_value="초코프렌즈우유", description="검색할 물품 ID"
    )

    use_sim_arg = DeclareLaunchArgument(
        "use_sim", default_value="false", description="시뮬레이션 모드 사용 여부"
    )

    # 로그 출력
    system_start_log = LogInfo(msg="=== 스마트 물류 창고 자동화 시스템 시작 ===")

    # 1. Robot Master 노드 (즉시 시작)
    robot_master_node = Node(
        package="robot_master",
        executable="robot_master",
        name="robot_master",
        output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim")}],
    )

    # 2. Robot Vision 노드 (2초 후 시작)
    robot_vision_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="robot_vision",
                executable="robot_vision",
                name="robot_vision_gui",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim")}],
            )
        ],
    )

    # 3. OCR 추론 노드 (3초 후 시작)
    ocr_inference_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="robot_vision",
                executable="ocr_inference_node.py",
                name="ocr_inference_node",
                output="screen",
                parameters=[
                    {"use_cuda": True},
                    {"score_threshold": 0.7},
                    {"nms_threshold": 0.3},
                    {"use_sim_time": LaunchConfiguration("use_sim")},
                ],
            )
        ],
    )

    # 4. 네비게이션 시스템 (5초 후 시작)
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="네비게이션 시스템 시작..."),
            # 실제 환경에서는 별도의 launch 파일 실행
            # 여기서는 핵심 노드들만 실행
        ],
    )

    # 5. Waypoint Navigator 노드 (10초 후 시작)
    waypoint_navigator_node = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="웨이포인트 네비게이터 시작..."),
            Node(
                package="robot_navigation",
                executable="waypoint_navigator",
                name="waypoint_navigator",
                output="screen",
                arguments=[LaunchConfiguration("target_item")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim")}],
            ),
        ],
    )

    # 시스템 상태 모니터링 (15초 후 시작)
    system_monitor = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="=== 시스템 초기화 완료 ==="),
            LogInfo(msg="물품 검색을 시작합니다..."),
        ],
    )

    return LaunchDescription(
        [
            # Launch 파라미터
            target_item_arg,
            use_sim_arg,
            # 시스템 시작 로그
            system_start_log,
            # 노드들을 순차적으로 시작
            robot_master_node,  # 0초
            robot_vision_node,  # 2초
            ocr_inference_node,  # 3초
            navigation_launch,  # 5초
            waypoint_navigator_node,  # 10초
            system_monitor,  # 15초
        ]
    )
