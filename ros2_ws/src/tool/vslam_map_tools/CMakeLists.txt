cmake_minimum_required(VERSION 3.8)
project(vslam_map_tools)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  vslam_map_tools/diagnose_vgl_bag.py
  vslam_map_tools/export_aligned_landmarks_offline.py
  vslam_map_tools/record_vslam_reference_snapshot.py
  vslam_map_tools/rewrite_camera_info_frame_id.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_snapshot_serialization
    test/test_snapshot_serialization.py
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_auto_package()
