#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # robot_master 노드 실행
    robot_master_cmd = Node(
        package="robot_master",
        executable="robot_master",
        name="robot_master",
        output="screen",
    )

    # 런치 설명 생성
    ld = LaunchDescription(
        [
            robot_master_cmd,
        ]
    )

    return ld
