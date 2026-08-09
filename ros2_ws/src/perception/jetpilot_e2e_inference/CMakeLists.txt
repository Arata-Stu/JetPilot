cmake_minimum_required(VERSION 3.8)
project(jetpilot_e2e_inference)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_control_decoder_node SHARED
  src/e2e_control_decoder_node.cpp
)
target_compile_features(${PROJECT_NAME}_control_decoder_node PUBLIC cxx_std_17)
rclcpp_components_register_node(${PROJECT_NAME}_control_decoder_node
  PLUGIN "jetpilot_e2e_inference::E2EControlDecoderNode"
  EXECUTABLE e2e_control_decoder_node
)

ament_auto_add_library(${PROJECT_NAME}_trajectory_decoder_node SHARED
  src/e2e_trajectory_decoder_node.cpp
)
target_compile_features(${PROJECT_NAME}_trajectory_decoder_node PUBLIC cxx_std_17)
rclcpp_components_register_node(${PROJECT_NAME}_trajectory_decoder_node
  PLUGIN "jetpilot_e2e_inference::E2ETrajectoryDecoderNode"
  EXECUTABLE e2e_trajectory_decoder_node
)

set_target_properties(
  ${PROJECT_NAME}_control_decoder_node
  ${PROJECT_NAME}_trajectory_decoder_node
  PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  jetpilot_e2e_inference/e2e_pytorch_inference_node.py
  scripts/build_tensorrt_engine.sh
  scripts/deploy_tensorrt_tui.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE config launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
