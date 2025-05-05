# TurtleBot: Factory Warehouse Parcel Delivery System

## Overview

This project implements a TurtleBot3-based system for a factory warehouse environment, where the robot identifies parcels by scanning QR codes and delivers them to a designated location. The system uses ROS2 Humble, Cartographer for SLAM, Nav2 for navigation, and Dynamixel for lift control. The robot navigates to predefined parcel locations, scans QR codes to match the target item ID, lifts the parcel, and delivers it to a specified drop-off point.

### Features
- **User Interface**: Qt-based UI for inputting target item ID and displaying system status.
- **SLAM & Navigation**: Cartographer for map generation and Nav2 for navigation to parcel locations and drop-off points.
- **QR Code Recognition**: Scans QR codes on parcels to identify the target item using `zbar`.
- **Lift Mechanism**: Dynamixel-based lift to pick up and drop off parcels.
- **Simulation**: Gazebo simulation of the factory warehouse environment.

### Prerequisites
- ROS2 Humble
- TurtleBot3 packages (`turtlebot3`, `turtlebot3_msgs`, `DynamixelSDK`, `turtlebot3_cartographer`, `turtlebot3_navigation`)
- OpenCV
- Qt5
- Gazebo

## Installation

1. **Clone the Repository**
    ```bash
    source /opt/ros/humble/setup.bash
    mkdir -p ~/turtlebot3_ws/src
    cd ~/turtlebot3_ws/src/
    git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
    git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    git clone https://github.com/mkdir-sweetiepie/turtlebot3
    sudo apt install python3-colcon-common-extensions
    cd ~/turtlebot3_ws
    colcon build --symlink-install
    echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
    source ~/.bashrc
    ```

2. **Install Dependencies**
    ```bash
    sudo apt install ros-humble-gazebo-*
    sudo apt install ros-humble-cartographer
    sudo apt install ros-humble-cartographer-ros
    sudo apt install ros-humble-navigation2
    sudo apt install ros-humble-nav2-bringup
    sudo apt install qtcreator
    sudo apt install libzbar-dev libyaml-cpp-dev
    ```


## Execution
- raspberry pi 새 터미널 열어서
    ssh ubuntu@192.168.0.2
    입력해주고 yes 한뒤 비밀번호 입력
    ```bash
    # Hardware
    ros2 launch turtlebot3_bringup robot.launch.py
    ```
- Launch TurtleBot3 Hardware or Simulation
    ros domain 12로 맞추기
    ```bash
    # Master
    ros2 run robot_master robot_master
    # Vision
    ros2 run robot_vision robot_vision
    # Simulation
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
    ```

License
This project is licensed under the Apache License 2.0 - see the LICENSE file for details.
Contributors
[Hongjihyeon]


