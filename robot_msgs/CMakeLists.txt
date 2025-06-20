cmake_minimum_required(VERSION 3.8)
project(robot_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 메시지 파일 설정
set(msg_files
  "msg/VisionMsg.msg"
  "msg/OCRRequest.msg"
  "msg/OCRResult.msg"
  "msg/LogMessage.msg"
)

# 서비스 파일 설정
set(srv_files
  "srv/ItemInfo.srv"
)

# 액션 파일 설정
set(action_files
  "action/PreciseControl.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()