cmake_minimum_required(VERSION 3.16)
project(canopy)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/canopy_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(canopy_msgs REQUIRED)
find_package(canopy_perception REQUIRED)
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
add_library(canopy_core
  src/approach_pose.cpp
  src/coverage_map.cpp
  src/grid.cpp
  src/object_map.cpp
  src/room_segmentation.cpp
  src/room_typing.cpp
  src/viewpoint_planner.cpp
  src/world_render.cpp
  src/world_store.cpp
)
canopy_target_defaults(canopy_core)
target_link_libraries(canopy_core PUBLIC
  Eigen3::Eigen
  opencv_core
  opencv_imgproc
  yaml-cpp
)

add_library(canopy_node src/world_model_node.cpp)
canopy_target_defaults(canopy_node)
target_link_libraries(canopy_node PUBLIC
  canopy_core
  canopy_perception::canopy_perception_core
  opencv_imgcodecs
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  ${canopy_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(world_model src/world_model_main.cpp)
canopy_target_defaults(world_model)
target_link_libraries(world_model PUBLIC canopy_node)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS canopy_core canopy_node EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS world_model DESTINATION lib/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/track_run.py
    scripts/snapshot_map.py
  DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_room_segmentation test/test_room_segmentation.cpp)
  target_link_libraries(test_room_segmentation canopy_core opencv_imgcodecs)
  target_compile_definitions(test_room_segmentation PRIVATE
    CANOPY_TEST_MAPS="${CMAKE_CURRENT_SOURCE_DIR}/test/maps")

  ament_add_gmock(test_object_map test/test_object_map.cpp TIMEOUT 120)
  target_link_libraries(test_object_map canopy_core)

  ament_add_gmock(test_world_parts test/test_world_parts.cpp)
  target_link_libraries(test_world_parts canopy_core)
  target_compile_definitions(test_world_parts PRIVATE
    CANOPY_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")

  ament_add_gmock(test_viewpoint_planner test/test_viewpoint_planner.cpp TIMEOUT 300)
  target_link_libraries(test_viewpoint_planner canopy_core)

  canopy_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  canopy_msgs
  canopy_perception
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
