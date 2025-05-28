cmake_minimum_required(VERSION 3.5)
project(robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(navigation_manager
  src/main.cpp
  src/navigation_manager.cpp
)

ament_target_dependencies(navigation_manager
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  robot_msgs
  ament_index_cpp
)

target_link_libraries(navigation_manager
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS
  navigation_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
