cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 헤더 파일 경로 추가
include_directories(include)

# waypoint_navigator 실행 파일 생성
add_executable(waypoint_navigator
  src/waypoint_navigator.cpp
)

ament_target_dependencies(waypoint_navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  robot_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# 설정 파일 및 런치 파일 설치
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 실행 파일 설치
install(TARGETS
  waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  robot_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
