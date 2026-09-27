cmake_minimum_required(VERSION 3.16)
project(canopy_perception)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/canopy_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(canopy_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# --- libraries -------------------------------------------------------------

add_library(canopy_perception_core src/depth_history.cpp src/depth_image.cpp src/phrase.cpp)
canopy_target_defaults(canopy_perception_core)
target_link_libraries(canopy_perception_core PUBLIC ${sensor_msgs_TARGETS})

add_library(canopy_mock_detector_node src/mock_detector_node.cpp)
canopy_target_defaults(canopy_mock_detector_node)
target_link_libraries(canopy_mock_detector_node PUBLIC
  canopy_perception_core
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  ${canopy_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(mock_detector src/mock_detector_main.cpp)
canopy_target_defaults(mock_detector)
target_link_libraries(mock_detector PUBLIC canopy_mock_detector_node)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS
  canopy_perception_core
  canopy_mock_detector_node
  EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS mock_detector DESTINATION lib/${PROJECT_NAME})
# The model-server client module the scripts below import.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/detector
  scripts/object_describer
  DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_depth_history test/test_depth_history.cpp)
  target_link_libraries(test_depth_history canopy_perception_core)

  ament_add_gmock(test_phrase test/test_phrase.cpp)
  target_link_libraries(test_phrase canopy_perception_core)

  find_package(launch_testing_ament_cmake REQUIRED)

  # Against a stub model server: needs neither a simulator nor a GPU.
  add_launch_test(test/test_detector.launch.py TARGET test_detector TIMEOUT 240)

  canopy_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  canopy_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  vision_msgs
)

ament_package()
