cmake_minimum_required(VERSION 3.8)
project(jetpilot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/lane_selection_policy.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_raceline_csv SHARED
  src/raceline_csv.cpp
)
target_compile_features(${PROJECT_NAME}_raceline_csv PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_route_lane_selector_node SHARED
  src/route_lane_selector_node.cpp
)
target_compile_features(${PROJECT_NAME}_route_lane_selector_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_route_lane_selector_node ${PROJECT_NAME}_core)

ament_auto_add_executable(route_lane_selector_node
  src/route_lane_selector_node_main.cpp
)
target_compile_features(route_lane_selector_node PUBLIC cxx_std_17)
target_link_libraries(route_lane_selector_node ${PROJECT_NAME}_route_lane_selector_node)

ament_auto_add_library(${PROJECT_NAME}_raceline_path_publisher_node SHARED
  src/raceline_path_publisher_node.cpp
)
target_compile_features(${PROJECT_NAME}_raceline_path_publisher_node PUBLIC cxx_std_17)
target_link_libraries(
  ${PROJECT_NAME}_raceline_path_publisher_node
  ${PROJECT_NAME}_raceline_csv
)

ament_auto_add_executable(raceline_path_publisher_node
  src/raceline_path_publisher_node_main.cpp
)
target_compile_features(raceline_path_publisher_node PUBLIC cxx_std_17)
target_link_libraries(
  raceline_path_publisher_node
  ${PROJECT_NAME}_raceline_path_publisher_node
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lane_selection_policy test/test_lane_selection_policy.cpp)
  if(TARGET test_lane_selection_policy)
    target_compile_features(test_lane_selection_policy PUBLIC cxx_std_17)
    target_link_libraries(test_lane_selection_policy ${PROJECT_NAME}_core)
  endif()

  ament_add_gtest(test_raceline_csv test/test_raceline_csv.cpp)
  if(TARGET test_raceline_csv)
    target_compile_features(test_raceline_csv PUBLIC cxx_std_17)
    target_link_libraries(test_raceline_csv ${PROJECT_NAME}_raceline_csv)
  endif()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
