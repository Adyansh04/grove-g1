cmake_minimum_required(VERSION 3.16)
project(g1_locomotion)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

# --- executables -----------------------------------------------------------

# The decision logic is compiled straight into the node and its unit test, so both build it
# under one dialect and nothing else has to link it.
add_executable(g1_base_approach src/g1_base_approach_node.cpp src/approach_planner.cpp)
g1_target_defaults(g1_base_approach)
target_link_libraries(g1_base_approach PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${g1_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

# --- install ---------------------------------------------------------------

install(TARGETS g1_base_approach DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  # The oblique-approach geometry, the overshoot that cannot be undone, and the deliberately
  # asymmetric quantum estimate. All of it is arithmetic the sim cannot check cheaply: a live
  # run exercises one trajectory, this exercises the branches.
  ament_add_gmock(test_approach_planner test/test_approach_planner.cpp src/approach_planner.cpp)
  g1_target_defaults(test_approach_planner)

  g1_add_lint_tests()
endif()

ament_package()
