cmake_minimum_required(VERSION 3.16)
project(g1_vla)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(g1_manipulation REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# --- libraries -------------------------------------------------------------

add_library(g1_vla_chunk_utils src/chunk_utils.cpp)
g1_target_defaults(g1_vla_chunk_utils)
target_link_libraries(g1_vla_chunk_utils PUBLIC ${trajectory_msgs_TARGETS})

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_vla_chunk_utils EXPORT export_${PROJECT_NAME} DESTINATION lib)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  ament_add_gmock(test_chunk_utils test/test_chunk_utils.cpp)
  target_link_libraries(test_chunk_utils g1_vla_chunk_utils)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  control_msgs
  g1_manipulation
  g1_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  vision_msgs
)

ament_package()
