cmake_minimum_required(VERSION 3.16)
project(g1_sensor_relay)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

# --- libraries -------------------------------------------------------------

# Framing and validation, free of ROS and sockets so the wire format tests without a graph.
add_library(frame_reader src/frame_reader.cpp)
g1_target_defaults(frame_reader)

# The conversion itself, split out so it tests without a graph.
add_library(livox_custom_msg src/livox_custom_msg.cpp)
g1_target_defaults(livox_custom_msg)
target_link_libraries(livox_custom_msg PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(g1_sensor_relay src/g1_sensor_relay_node.cpp)
g1_target_defaults(g1_sensor_relay)
target_link_libraries(g1_sensor_relay PUBLIC
  frame_reader
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

# Restates the relay's own output in the format FAST-LIO wants. Separate executable so the
# odometry track can start and stop without touching the sensor path everything else needs.
add_executable(g1_livox_bridge src/g1_livox_bridge_node.cpp)
g1_target_defaults(g1_livox_bridge)
target_link_libraries(g1_livox_bridge PUBLIC livox_custom_msg)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS frame_reader livox_custom_msg EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_sensor_relay g1_livox_bridge DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_frame_reader test/test_frame_reader.cpp)
  target_link_libraries(test_frame_reader frame_reader)
  # The wire-format drift check reads both copies of sensor_frame.h by relative path.
  set_tests_properties(test_frame_reader PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  ament_add_gmock(test_livox_custom_msg test/test_livox_custom_msg.cpp)
  target_link_libraries(test_livox_custom_msg livox_custom_msg)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
