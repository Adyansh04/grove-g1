cmake_minimum_required(VERSION 3.16)
project(g1_world_model)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(g1_perception REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- libraries -------------------------------------------------------------

# The algorithms, free of ROS so the unit tests run in milliseconds.
add_library(g1_world_model_core
  src/approach_pose.cpp
  src/coverage_map.cpp
  src/grid.cpp
  src/object_map.cpp
  src/room_segmentation.cpp
  src/room_typing.cpp
  src/viewpoint_planner.cpp
  src/world_store.cpp
)
g1_target_defaults(g1_world_model_core)
target_link_libraries(g1_world_model_core PUBLIC
  Eigen3::Eigen
  opencv_core
  opencv_imgproc
  yaml-cpp
)

add_library(g1_world_model_node src/world_model_node.cpp)
g1_target_defaults(g1_world_model_node)
target_link_libraries(g1_world_model_node PUBLIC
  g1_world_model_core
  g1_perception::g1_perception_depth_history
  opencv_imgcodecs
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${g1_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(g1_world_model src/world_model_main.cpp)
g1_target_defaults(g1_world_model)
target_link_libraries(g1_world_model PUBLIC g1_world_model_node)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS g1_world_model_core g1_world_model_node EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_world_model DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_room_segmentation test/test_room_segmentation.cpp)
  target_link_libraries(test_room_segmentation g1_world_model_core opencv_imgcodecs)
  # The committed maps, read from the source tree: g1_navigation is not a build dependency.
  target_compile_definitions(test_room_segmentation PRIVATE
    G1_MAPS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../g1_navigation/maps")

  ament_add_gmock(test_object_map test/test_object_map.cpp TIMEOUT 120)
  target_link_libraries(test_object_map g1_world_model_core)

  ament_add_gmock(test_world_parts test/test_world_parts.cpp)
  target_link_libraries(test_world_parts g1_world_model_core)
  target_compile_definitions(test_world_parts PRIVATE
    G1_WORLD_MODEL_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  ament_add_gmock(test_viewpoint_planner test/test_viewpoint_planner.cpp TIMEOUT 300)
  target_link_libraries(test_viewpoint_planner g1_world_model_core)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  g1_msgs
  g1_perception
  geometry_msgs
  nav_msgs
  OpenCV
  rclcpp
  sensor_msgs
  std_srvs
  tf2_eigen
  tf2_ros
  visualization_msgs
  yaml-cpp
)

ament_package()
