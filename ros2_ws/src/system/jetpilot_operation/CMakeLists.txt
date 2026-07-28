cmake_minimum_required(VERSION 3.8)
project(jetpilot_operation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_operation_mode_manager_node SHARED
  src/operation_mode_manager_node.cpp
)
target_compile_features(${PROJECT_NAME}_operation_mode_manager_node PUBLIC cxx_std_17)

ament_auto_add_executable(operation_mode_manager_node
  src/operation_mode_manager_node_main.cpp
)
target_compile_features(operation_mode_manager_node PUBLIC cxx_std_17)
target_link_libraries(
  operation_mode_manager_node
  ${PROJECT_NAME}_operation_mode_manager_node
)

ament_auto_add_library(${PROJECT_NAME}_command_mux_node SHARED
  src/command_mux_node.cpp
)
target_compile_features(${PROJECT_NAME}_command_mux_node PUBLIC cxx_std_17)

ament_auto_add_executable(command_mux_node
  src/command_mux_node_main.cpp
)
target_compile_features(command_mux_node PUBLIC cxx_std_17)
target_link_libraries(command_mux_node ${PROJECT_NAME}_command_mux_node)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
