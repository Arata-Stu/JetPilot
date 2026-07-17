cmake_minimum_required(VERSION 3.8)
project(jetpilot_teleop_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(teleop_cmd_node src/teleop_cmd_node.cpp)
target_compile_features(teleop_cmd_node PUBLIC cxx_std_17)

ament_auto_add_executable(teleop_button_manager_node src/teleop_button_manager_node.cpp)
target_include_directories(teleop_button_manager_node PRIVATE include)
target_compile_features(teleop_button_manager_node PUBLIC cxx_std_17)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_button_rising_edge test/test_button_rising_edge.cpp)
  target_include_directories(test_button_rising_edge PRIVATE include)
  target_compile_features(test_button_rising_edge PUBLIC cxx_std_17)
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joy_calibrator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/joy_profile_editor.html
  DESTINATION share/${PROJECT_NAME}/web
)

install(FILES
  scripts/joy_profile_editor.html
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
