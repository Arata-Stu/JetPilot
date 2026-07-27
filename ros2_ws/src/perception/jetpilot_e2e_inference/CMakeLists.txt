cmake_minimum_required(VERSION 3.8)
project(jetpilot_e2e_inference)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  jetpilot_e2e_inference/e2e_control_decoder_node.py
  scripts/build_tensorrt_engine.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
