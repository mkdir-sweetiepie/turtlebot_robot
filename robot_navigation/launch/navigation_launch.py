from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory("turtlebot3_navigation")
    launch_dir = os.path.join(package_dir, "launch")

    # Map configuration
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Nav2 parameters file
    nav2_params_file = LaunchConfiguration("params_file")

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(package_dir, "maps", "my_map.yaml"),
        description="Full path to map yaml file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(package_dir, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    # Start Nav2 stack
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
    )

    # Start our custom navigation node
    navigation_node_cmd = Node(
        package="turtlebot3_navigation",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Return the LaunchDescription
    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_use_sim_time_cmd,
            declare_params_file_cmd,
            nav2_bringup_cmd,
            navigation_node_cmd,
        ]
    )
