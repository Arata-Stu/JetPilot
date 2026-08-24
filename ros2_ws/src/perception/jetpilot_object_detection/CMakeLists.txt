cmake_minimum_required(VERSION 3.22.1)
project(jetpilot_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(CUDAToolkit REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_decoder SHARED
  src/yolov8_decoder.cpp
)
target_compile_features(${PROJECT_NAME}_decoder PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_image_gate_node SHARED
  src/image_gate_node.cpp
)
target_compile_features(${PROJECT_NAME}_image_gate_node PUBLIC cxx_std_17)
rclcpp_components_register_node(${PROJECT_NAME}_image_gate_node
  PLUGIN "jetpilot_object_detection::ImageGateNode"
  EXECUTABLE object_detection_image_gate_node
)

ament_auto_add_library(${PROJECT_NAME}_decoder_node SHARED
  src/yolov8_decoder_node.cpp
)
target_compile_features(${PROJECT_NAME}_decoder_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_decoder_node
  ${PROJECT_NAME}_decoder
  CUDA::cudart
)
rclcpp_components_register_node(${PROJECT_NAME}_decoder_node
  PLUGIN "jetpilot_object_detection::YoloV8DecoderNode"
  EXECUTABLE yolov8_decoder_node
)

set_target_properties(
  ${PROJECT_NAME}_decoder
  ${PROJECT_NAME}_image_gate_node
  ${PROJECT_NAME}_decoder_node
  PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolov8_decoder test/test_yolov8_decoder.cpp)
  target_link_libraries(test_yolov8_decoder ${PROJECT_NAME}_decoder)
endif()

ament_auto_package(
  INSTALL_TO_SHARE config launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
