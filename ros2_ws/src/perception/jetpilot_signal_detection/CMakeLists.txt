cmake_minimum_required(VERSION 3.8)
project(jetpilot_signal_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_core SHARED src/signal_vote_filter.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_node SHARED src/signal_detection_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core)

ament_auto_add_executable(signal_detection_node src/signal_detection_node_main.cpp)
target_compile_features(signal_detection_node PUBLIC cxx_std_17)
target_link_libraries(signal_detection_node ${PROJECT_NAME}_node)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_signal_vote_filter test/test_signal_vote_filter.cpp)
  if(TARGET test_signal_vote_filter)
    target_compile_features(test_signal_vote_filter PUBLIC cxx_std_17)
    target_link_libraries(test_signal_vote_filter ${PROJECT_NAME}_core)
  endif()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
