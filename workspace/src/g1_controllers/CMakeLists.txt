cmake_minimum_required(VERSION 3.16)
project(g1_controllers)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

# The shipped onnxruntimeConfig.cmake points at lib64/ and include/onnxruntime/, neither of
# which exists in this tarball's layout, so locate the pieces directly.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h PATHS /opt/onnxruntime/include REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime PATHS /opt/onnxruntime/lib REQUIRED)

# --- libraries -------------------------------------------------------------

add_library(g1_agile_policy SHARED src/agile_policy.cpp)
g1_target_defaults(g1_agile_policy)
target_include_directories(g1_agile_policy SYSTEM PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(g1_agile_policy PUBLIC ${ONNXRUNTIME_LIBRARY})

add_library(g1_freeze_controller SHARED src/g1_freeze_controller.cpp)
g1_target_defaults(g1_freeze_controller)
target_link_libraries(g1_freeze_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

add_library(g1_agile_controller SHARED src/g1_agile_controller.cpp)
g1_target_defaults(g1_agile_controller)
target_link_libraries(g1_agile_controller PUBLIC
  g1_agile_policy
  ament_index_cpp::ament_index_cpp
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_library(g1_safety_controller SHARED src/g1_safety_controller.cpp)
g1_target_defaults(g1_safety_controller)
target_link_libraries(g1_safety_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${controller_manager_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface g1_controllers.xml)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config policy DESTINATION share/${PROJECT_NAME})
install(
  TARGETS g1_agile_policy g1_freeze_controller g1_agile_controller g1_safety_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Discovery through pluginlib's own loader is the proof controller_manager can spawn it;
  # a clean compile does not check the plugin XML or its export.
  ament_add_gmock(test_freeze_pluginlib test/test_freeze_pluginlib.cpp)
  target_link_libraries(test_freeze_pluginlib
    g1_freeze_controller g1_agile_controller g1_safety_controller
    pluginlib::pluginlib controller_interface::controller_interface)

  # The blend and slew are the only safety-critical arithmetic here, so they are tested
  # directly rather than through a controller_manager harness.
  ament_add_gmock(test_safety_blend test/test_safety_blend.cpp)
  target_link_libraries(test_safety_blend g1_safety_controller)

  # Loads the installed policy, so it pins the ONNX contract and the joint tables together.
  ament_add_gmock(test_agile_policy test/test_agile_policy.cpp)
  target_link_libraries(test_agile_policy g1_agile_policy ament_index_cpp::ament_index_cpp)

  # Reads the controller config against g1_description's own joint list: an unowned motor is
  # an unpowered one, and neither package's tests can see the pair on their own.
  ament_add_pytest_test(test_joint_ownership test/test_joint_ownership.py
    ENV
      "G1_CONTROLLERS_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
      "G1_DESCRIPTION_URDF_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_description/urdf"
  )

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(
  g1_agile_policy
  g1_freeze_controller
  g1_agile_controller
  g1_safety_controller
)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  controller_interface
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

ament_package()
