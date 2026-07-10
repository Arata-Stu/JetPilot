cmake_minimum_required(VERSION 3.8)
project(jetpilot_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/BagRequest.msg
  msg/BagStatus.msg
  msg/ControlCommand.msg
  msg/OperationModeRequest.msg
  msg/OperationModeState.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
