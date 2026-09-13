cmake_minimum_required(VERSION 3.22.1)
project(jetpilot_e2e_inference)

enable_language(CUDA)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(MetavisionSDK COMPONENTS base REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_control_decoder_node SHARED
  src/e2e_control_decoder_node.cpp
)
target_compile_features(${PROJECT_NAME}_control_decoder_node PUBLIC cxx_std_17)
# ament_auto_add_library uses the plain target_link_libraries signature.
# Keep later link additions plain as well; CMake rejects mixed signatures.
target_link_libraries(${PROJECT_NAME}_control_decoder_node CUDA::cudart)
rclcpp_components_register_node(${PROJECT_NAME}_control_decoder_node
  PLUGIN "jetpilot_e2e_inference::E2EControlDecoderNode"
  EXECUTABLE e2e_control_decoder_node
)

ament_auto_add_library(${PROJECT_NAME}_trajectory_decoder_node SHARED
  src/e2e_trajectory_decoder_node.cpp
)
target_compile_features(${PROJECT_NAME}_trajectory_decoder_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_trajectory_decoder_node CUDA::cudart)
rclcpp_components_register_node(${PROJECT_NAME}_trajectory_decoder_node
  PLUGIN "jetpilot_e2e_inference::E2ETrajectoryDecoderNode"
  EXECUTABLE e2e_trajectory_decoder_node
)

ament_auto_add_library(${PROJECT_NAME}_event_tensor_encoder_node SHARED
  src/event_tensor_encoder_node.cpp
  src/event_tensor_cuda_backend.cu
)
target_compile_features(${PROJECT_NAME}_event_tensor_encoder_node PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME}_event_tensor_encoder_node PROPERTIES
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
  CUDA_SEPARABLE_COMPILATION ON
)
target_link_libraries(${PROJECT_NAME}_event_tensor_encoder_node
  CUDA::cudart
  MetavisionSDK::base
)
rclcpp_components_register_node(${PROJECT_NAME}_event_tensor_encoder_node
  PLUGIN "jetpilot_e2e_inference::EventTensorEncoderNode"
  EXECUTABLE event_tensor_encoder_node
)

ament_auto_add_library(${PROJECT_NAME}_latent_state_manager_node SHARED
  src/latent_state_manager_node.cpp
)
target_compile_features(${PROJECT_NAME}_latent_state_manager_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_latent_state_manager_node CUDA::cudart)
rclcpp_components_register_node(${PROJECT_NAME}_latent_state_manager_node
  PLUGIN "jetpilot_e2e_inference::LatentStateManagerNode"
  EXECUTABLE latent_state_manager_node
)

if(BUILD_TESTING)
  add_executable(event_tensor_cuda_backend_test
    test/event_tensor_cuda_backend_test.cpp
  )
  target_compile_features(event_tensor_cuda_backend_test PUBLIC cxx_std_17)
  target_include_directories(event_tensor_cuda_backend_test PRIVATE include)
  target_link_libraries(event_tensor_cuda_backend_test
    ${PROJECT_NAME}_event_tensor_encoder_node
    CUDA::cudart
  )
  add_test(NAME event_tensor_cuda_backend_test COMMAND event_tensor_cuda_backend_test)
endif()

set_target_properties(
  ${PROJECT_NAME}_control_decoder_node
  ${PROJECT_NAME}_trajectory_decoder_node
  ${PROJECT_NAME}_event_tensor_encoder_node
  ${PROJECT_NAME}_latent_state_manager_node
  PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  jetpilot_e2e_inference/event_image_adapter.py
  jetpilot_e2e_inference/e2e_pytorch_inference_node.py
  scripts/build_tensorrt_engine.sh
  scripts/deploy_tensorrt_tui.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE config launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
