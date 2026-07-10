cmake_minimum_required(VERSION 3.8)
project(jetpilot_system_launch)

find_package(ament_cmake_auto REQUIRED)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz
)
