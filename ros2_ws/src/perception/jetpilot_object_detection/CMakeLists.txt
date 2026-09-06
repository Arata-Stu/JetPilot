cmake_minimum_required(VERSION 3.22.1)
project(jetpilot_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Threads REQUIRED)
ament_auto_find_build_dependencies()

# TensorRT is already required by the deployed YOLO pipeline. Keep its C++ API
# confined to the backend library so a NITROS adapter can replace it later.
find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES aarch64-linux-gnu x86_64-linux-gnu REQUIRED)
find_library(TENSORRT_LIBRARY nvinfer REQUIRED)
ament_auto_add_library(${PROJECT_NAME}_reid_core SHARED src/reid_core.cpp)
target_compile_features(${PROJECT_NAME}_reid_core PUBLIC cxx_std_17)
ament_auto_add_library(${PROJECT_NAME}_reid_backend SHARED src/reid_tensor_rt_backend.cpp)
target_compile_features(${PROJECT_NAME}_reid_backend PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_reid_backend PRIVATE ${TENSORRT_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_reid_backend CUDA::cudart ${TENSORRT_LIBRARY})
ament_auto_add_library(${PROJECT_NAME}_reid_node SHARED src/reid_node.cpp)
target_compile_features(${PROJECT_NAME}_reid_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_reid_node
  ${PROJECT_NAME}_reid_core ${PROJECT_NAME}_reid_backend Threads::Threads)
rclcpp_components_register_node(${PROJECT_NAME}_reid_node
  PLUGIN "jetpilot_object_detection::ReidNode"
  EXECUTABLE opponent_reid_node)

ament_auto_add_library(${PROJECT_NAME}_decoder SHARED
  src/yolov8_decoder.cpp
  src/byte_tracker.cpp
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
  ${PROJECT_NAME}_reid_core
  ${PROJECT_NAME}_reid_backend
  ${PROJECT_NAME}_reid_node
  PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(BUILD_TESTING)
  add_executable(test_reid_core test/test_reid_core.cpp)
  target_link_libraries(test_reid_core ${PROJECT_NAME}_reid_core)
  target_compile_features(test_reid_core PRIVATE cxx_std_17)
  add_test(NAME test_reid_core COMMAND test_reid_core)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_opponent_geometry test/test_opponent_geometry.py)
  ament_add_pytest_test(test_opponent_views test/test_opponent_views.py)
  add_executable(test_byte_tracker test/test_byte_tracker.cpp)
  target_link_libraries(test_byte_tracker ${PROJECT_NAME}_decoder)
  add_test(NAME test_byte_tracker COMMAND test_byte_tracker)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolov8_decoder test/test_yolov8_decoder.cpp)
  target_link_libraries(test_yolov8_decoder ${PROJECT_NAME}_decoder)
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS jetpilot_object_detection/opponent_projection_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE config launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
