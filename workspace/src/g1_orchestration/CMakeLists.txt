cmake_minimum_required(VERSION 3.16)
project(g1_orchestration)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
# v4, the same libbehaviortree_cpp Nav2's navigator links, so leaves here and Nav2's share it.
find_package(behaviortree_cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vision_msgs REQUIRED)

# --- library ---------------------------------------------------------------

add_library(g1_bt_nodes
  src/arm_authority.cpp
  src/port_types.cpp
  src/registration.cpp
  src/service_leaf.cpp
  src/skills/approach_object.cpp
  src/skills/arm_authority_leaves.cpp
  src/skills/clear_costmaps.cpp
  src/skills/clear_octomap.cpp
  src/skills/grasp.cpp
  src/skills/look_for.cpp
  src/skills/navigate_to_pose.cpp
  src/skills/pick.cpp
  src/skills/place.cpp
  src/skills/retreat.cpp
  src/skills/set_arm_posture.cpp
  src/skills/world_model_leaves.cpp
)
g1_target_defaults(g1_bt_nodes)
target_link_libraries(g1_bt_nodes PUBLIC
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${controller_manager_msgs_TARGETS}
  ${g1_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  ${std_srvs_TARGETS}
  ${vision_msgs_TARGETS}
)

# --- executables -----------------------------------------------------------

add_executable(g1_bt_executor src/g1_bt_executor_main.cpp)
g1_target_defaults(g1_bt_executor)
target_link_libraries(g1_bt_executor PUBLIC g1_bt_nodes)

# Regenerates the checked-in Groot2 palette. Not run by the build; test_node_model catches drift.
add_executable(g1_bt_node_model src/g1_bt_node_model_main.cpp)
g1_target_defaults(g1_bt_node_model)
target_link_libraries(g1_bt_node_model PUBLIC g1_bt_nodes)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch trees DESTINATION share/${PROJECT_NAME})
install(TARGETS g1_bt_nodes EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_bt_executor g1_bt_node_model DESTINATION lib/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  # Every shipped tree parses against the registered leaves.
  ament_add_gmock(test_tree_loads test/test_tree_loads.cpp)
  target_link_libraries(test_tree_loads g1_bt_nodes)
  target_compile_definitions(test_tree_loads PRIVATE
    G1_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")

  # The action leaf base against a real server; the only test here that needs a ROS graph.
  ament_add_gmock(test_action_leaf test/test_action_leaf.cpp TIMEOUT 120)
  target_link_libraries(test_action_leaf g1_bt_nodes)

  # The checked-in Groot2 palette matches the registered leaves.
  ament_add_gmock(test_node_model test/test_node_model.cpp)
  target_link_libraries(test_node_model g1_bt_nodes)
  target_compile_definitions(test_node_model PRIVATE
    G1_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")

  # controlledParts() against g1_bringup's activate_arm script, the other implementation.
  ament_add_gmock(test_authority_drift test/test_authority_drift.cpp)
  target_link_libraries(test_authority_drift g1_bt_nodes)
  target_compile_definitions(test_authority_drift PRIVATE
    G1_ACTIVATE_ARM_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/scripts/activate_arm")

  # ament_cmake_xmllint only globs *.xml, so the Groot2 project file needs its own check.
  find_program(XMLLINT_EXECUTABLE xmllint)
  if(XMLLINT_EXECUTABLE)
    add_test(
      NAME xmllint_btproj_${PROJECT_NAME}
      COMMAND ${XMLLINT_EXECUTABLE} --noout
        "${CMAKE_CURRENT_SOURCE_DIR}/trees/g1_orchestration.btproj"
    )
  else()
    message(WARNING "xmllint not found; skipping xmllint_btproj_${PROJECT_NAME}")
  endif()

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
