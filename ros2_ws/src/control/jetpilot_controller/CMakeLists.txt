cmake_minimum_required(VERSION 3.8)
project(jetpilot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/kinematic_mpc.cpp
  src/longitudinal_controller.cpp
  src/map_pursuit.cpp
  src/pure_pursuit.cpp
  src/trailing_controller.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_path_tracking_controller_node SHARED
  src/path_tracking_controller_node.cpp
)
target_compile_features(${PROJECT_NAME}_path_tracking_controller_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_path_tracking_controller_node ${PROJECT_NAME}_core)

ament_auto_add_executable(path_tracking_controller_node
  src/path_tracking_controller_node_main.cpp
)
target_compile_features(path_tracking_controller_node PUBLIC cxx_std_17)
target_link_libraries(
  path_tracking_controller_node
  ${PROJECT_NAME}_path_tracking_controller_node
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller_core test/test_controller_core.cpp)
  if(TARGET test_controller_core)
    target_compile_features(test_controller_core PUBLIC cxx_std_17)
    target_link_libraries(test_controller_core ${PROJECT_NAME}_core)
  endif()
endif()

ament_auto_package()
