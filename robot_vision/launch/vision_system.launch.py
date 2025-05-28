from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_vision",
                executable="robot_vision",
                name="robot_vision_gui",
                output="screen",
            ),
            Node(
                package="robot_vision",
                executable="ocr_inference_node.py",
                name="ocr_inference_node",
                output="screen",
                parameters=[
                    {"use_cuda": True},
                    {"score_threshold": 0.7},
                    {"nms_threshold": 0.3},
                ],
            ),
        ]
    )
