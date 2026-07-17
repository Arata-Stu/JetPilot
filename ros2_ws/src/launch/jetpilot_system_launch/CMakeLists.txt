cmake_minimum_required(VERSION 3.8)
project(jetpilot_system_launch)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_bringup_replay_safety
    test/test_bringup_replay_safety.py
  )
  ament_add_pytest_test(
    test_localization_manager_launch
    test/test_localization_manager_launch.py
  )
  ament_add_pytest_test(
    test_bringup_script
    test/test_bringup_script.py
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  rviz
)
