cmake_minimum_required(VERSION 3.8)
project(bag_manager_py)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bag_manager_py/bag_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
