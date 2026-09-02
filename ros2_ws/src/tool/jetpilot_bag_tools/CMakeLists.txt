cmake_minimum_required(VERSION 3.8)
project(jetpilot_bag_tools)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  jetpilot_bag_tools/bag_manager_node.py
  jetpilot_bag_tools/calibrate_throttle_from_bag.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
