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

add_library(g1_vla_server_node src/g1_vla_server_node.cpp)
g1_target_defaults(g1_vla_server_node)
target_link_libraries(g1_vla_server_node PUBLIC
  g1_vla_chunk_utils
  g1_manipulation::g1_hand_contact
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  moveit_ros_planning::moveit_robot_model_loader
  ${control_msgs_TARGETS}
  ${g1_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
)

add_library(g1_vla_mock_engine_node src/mock_engine_node.cpp)
g1_target_defaults(g1_vla_mock_engine_node)
target_link_libraries(g1_vla_mock_engine_node PUBLIC
  rclcpp::rclcpp
  ${g1_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(g1_vla_server src/g1_vla_server_main.cpp)
g1_target_defaults(g1_vla_server)
target_link_libraries(g1_vla_server PUBLIC g1_vla_server_node)

add_executable(g1_vla_mock_engine src/mock_engine_main.cpp)
g1_target_defaults(g1_vla_mock_engine)
target_link_libraries(g1_vla_mock_engine PUBLIC g1_vla_mock_engine_node)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS g1_vla_chunk_utils g1_vla_server_node g1_vla_mock_engine_node
  EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_vla_server g1_vla_mock_engine DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/g1_vla_groot_adapter DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gmock(test_chunk_utils test/test_chunk_utils.cpp)
  target_link_libraries(test_chunk_utils g1_vla_chunk_utils)

  # Not a simulator test: it runs the adapter against a stub policy server, so it needs neither
  # MuJoCo nor a GPU and CI can run it.
  add_launch_test(test/test_groot_adapter.launch.py TARGET test_groot_adapter TIMEOUT 180)

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
