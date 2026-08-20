cmake_minimum_required(VERSION 3.16)
project(g1_state_estimation)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# --- libraries -------------------------------------------------------------

# Frame and staleness math, deliberately free of ROS so it tests without a node or DDS.
add_library(odom_math src/odom_math.cpp)
g1_target_defaults(odom_math)

add_library(g1_odometry_publisher_node src/g1_odometry_publisher_node.cpp)
g1_target_defaults(g1_odometry_publisher_node)
target_link_libraries(g1_odometry_publisher_node PUBLIC
  odom_math
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
)

# Hardware only: gives the rest of the stack its PointCloud2 while FAST-LIO takes the CustomMsg
# the driver is running in. A library so it tests without a Mid360, since nothing in simulation
# exercises this direction.
add_library(livox_cloud src/livox_cloud.cpp)
g1_target_defaults(livox_cloud)
target_link_libraries(livox_cloud PUBLIC
  ${sensor_msgs_TARGETS}
  ${livox_ros_driver2_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(g1_odometry_publisher src/g1_odometry_publisher_main.cpp)
g1_target_defaults(g1_odometry_publisher)
target_link_libraries(g1_odometry_publisher PUBLIC g1_odometry_publisher_node)

add_executable(g1_livox_pointcloud src/g1_livox_pointcloud_node.cpp)
g1_target_defaults(g1_livox_pointcloud)
target_link_libraries(g1_livox_pointcloud PUBLIC livox_cloud rclcpp::rclcpp)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lio_bench DESTINATION lib/${PROJECT_NAME})
install(TARGETS odom_math livox_cloud g1_odometry_publisher_node
  EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_odometry_publisher g1_livox_pointcloud DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)

  ament_add_gmock(test_odom_math test/test_odom_math.cpp)
  target_link_libraries(test_odom_math odom_math)

  # Hardware-only path: no simulation run reaches it, so this is its only coverage.
  ament_add_gmock(test_livox_cloud test/test_livox_cloud.cpp)
  target_link_libraries(test_livox_cloud livox_cloud)

  # Runs a real node on an isolated ROS_DOMAIN_ID, so it must not share the sim's.
  ament_add_gmock(test_odometry_publisher_node test/test_odometry_publisher_node.cpp)
  target_link_libraries(test_odometry_publisher_node
    g1_odometry_publisher_node ${rosgraph_msgs_TARGETS})

  # The sim FAST-LIO extrinsic is a copy of the URDF mount; this fails when they drift apart.
  # The full-stack version (FAST-LIO vs ground truth on a walk) is g1_bringup's
  # test_fastlio_odometry: it launches sim.launch.py, which would be a cycle from here.
  ament_add_pytest_test(test_sim_extrinsic test/test_sim_extrinsic.py)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle nav_msgs geometry_msgs tf2_ros)

ament_package()
