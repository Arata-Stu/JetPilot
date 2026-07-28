cmake_minimum_required(VERSION 3.8)
project(jetpilot_localization_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED
  src/pose_validation.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_node_impl SHARED
  src/localization_manager_node.cpp
)
target_compile_features(${PROJECT_NAME}_node_impl PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node_impl ${PROJECT_NAME}_core)

ament_auto_add_executable(jetpilot_localization_manager_node
  src/localization_manager_node_main.cpp
)
target_compile_features(jetpilot_localization_manager_node PUBLIC cxx_std_17)
target_link_libraries(jetpilot_localization_manager_node ${PROJECT_NAME}_node_impl)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_validation test/test_pose_validation.cpp)
  if(TARGET test_pose_validation)
    target_compile_features(test_pose_validation PUBLIC cxx_std_17)
    target_link_libraries(test_pose_validation ${PROJECT_NAME}_core)
  endif()
endif()

ament_auto_package()
