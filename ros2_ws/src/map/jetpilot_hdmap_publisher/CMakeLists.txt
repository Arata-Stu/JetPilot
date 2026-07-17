cmake_minimum_required(VERSION 3.8)
project(jetpilot_hdmap_publisher)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  jetpilot_hdmap_publisher/hd_map_publisher_node.py
  jetpilot_hdmap_publisher/hd_map_section_localizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
