cmake_minimum_required(VERSION 3.8)
project(jetpilot_rtp_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)

ament_auto_add_library(image_rtp_sender_component SHARED
  src/image_rtp_sender_component.cpp
)
target_compile_features(image_rtp_sender_component PUBLIC cxx_std_17)
target_link_libraries(image_rtp_sender_component
  PkgConfig::GSTREAMER
)

rclcpp_components_register_node(image_rtp_sender_component
  PLUGIN "jetpilot_rtp_tools::ImageRtpSenderComponent"
  EXECUTABLE image_rtp_sender_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_image_rtp_sender_component
    test/test_image_rtp_sender_component.cpp
  )
  target_link_libraries(test_image_rtp_sender_component
    image_rtp_sender_component
    PkgConfig::GSTREAMER
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
