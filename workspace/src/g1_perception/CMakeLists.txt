cmake_minimum_required(VERSION 3.16)
project(g1_perception)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

# --- libraries -------------------------------------------------------------

add_library(g1_perception_object_geometry src/object_geometry.cpp)
g1_target_defaults(g1_perception_object_geometry)

add_library(g1_perception_object_tracker src/object_tracker.cpp)
g1_target_defaults(g1_perception_object_tracker)
target_link_libraries(g1_perception_object_tracker PUBLIC g1_perception_object_geometry)

add_library(g1_perception_depth_history src/depth_history.cpp)
g1_target_defaults(g1_perception_depth_history)
target_link_libraries(g1_perception_depth_history PUBLIC ${sensor_msgs_TARGETS})

add_library(g1_object_geometry_node src/g1_object_geometry_node.cpp)
g1_target_defaults(g1_object_geometry_node)
target_link_libraries(g1_object_geometry_node PUBLIC
  g1_perception_depth_history
  g1_perception_object_tracker
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${g1_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

add_library(g1_mock_detector_node src/mock_detector_node.cpp)
g1_target_defaults(g1_mock_detector_node)
target_link_libraries(g1_mock_detector_node PUBLIC
  g1_perception_depth_history
  g1_perception_object_geometry
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  ${g1_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

add_library(g1_mock_grasp_source_node src/mock_grasp_source_node.cpp)
g1_target_defaults(g1_mock_grasp_source_node)
target_link_libraries(g1_mock_grasp_source_node PUBLIC
  rclcpp::rclcpp
  tf2_geometry_msgs::tf2_geometry_msgs
  ${g1_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(g1_object_geometry src/g1_object_geometry_main.cpp)
g1_target_defaults(g1_object_geometry)
target_link_libraries(g1_object_geometry PUBLIC g1_object_geometry_node)

add_executable(g1_mock_detector src/mock_detector_main.cpp)
g1_target_defaults(g1_mock_detector)
target_link_libraries(g1_mock_detector PUBLIC g1_mock_detector_node)

add_executable(g1_mock_grasp_source src/mock_grasp_source_main.cpp)
g1_target_defaults(g1_mock_grasp_source)
target_link_libraries(g1_mock_grasp_source PUBLIC g1_mock_grasp_source_node)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS
  g1_perception_object_geometry
  g1_perception_object_tracker
  g1_perception_depth_history
  g1_object_geometry_node
  g1_mock_detector_node
  g1_mock_grasp_source_node
  EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_object_geometry g1_mock_detector g1_mock_grasp_source
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/g1_detector scripts/g1_graspgen_adapter
  DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_object_geometry test/test_object_geometry.cpp)
  target_link_libraries(test_object_geometry g1_perception_object_geometry)

  ament_add_gmock(test_object_tracker test/test_object_tracker.cpp)
  target_link_libraries(test_object_tracker g1_perception_object_tracker)

  ament_add_gmock(test_depth_history test/test_depth_history.cpp)
  target_link_libraries(test_depth_history g1_perception_depth_history)

  find_package(launch_testing_ament_cmake REQUIRED)

  # Not a simulator test: it runs the detector against a stub vision server, so it needs neither
  # MuJoCo nor a GPU and CI can run it.
  add_launch_test(test/test_detector.launch.py TARGET test_detector TIMEOUT 240)

  add_launch_test(test/test_graspgen_adapter.launch.py TARGET test_graspgen_adapter TIMEOUT 240)

  add_launch_test(test/test_perception_objects.launch.py TARGET test_perception_objects
    TIMEOUT 500)
  set_tests_properties(test_perception_objects PROPERTIES
    LABELS simulator RESOURCE_LOCK mujoco_sim COST 85)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  g1_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)

ament_package()
