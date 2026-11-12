cmake_minimum_required(VERSION 3.16)
project(g1_inspire_hand)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- library ---------------------------------------------------------------

# The protocol and the serial line carry no ROS, so tests and tools use them on their own.
add_library(g1_inspire_protocol SHARED
  src/inspire_protocol.cpp
  src/serial_port.cpp
  src/inspire_hand.cpp
)
g1_target_defaults(g1_inspire_protocol)

add_library(g1_inspire_system SHARED src/g1_inspire_system.cpp)
g1_target_defaults(g1_inspire_system)
target_link_libraries(g1_inspire_system PUBLIC
  g1_inspire_protocol
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface g1_inspire_hand.xml)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_inspire_protocol g1_inspire_system EXPORT export_${PROJECT_NAME}
  DESTINATION lib)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  # Frames against golden bytes from Unitree's service, both framings.
  ament_add_gmock(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol g1_inspire_protocol)

  # The serial line and the register client against a fake hand on a pseudo-terminal.
  ament_add_gmock(test_serial_hand test/test_serial_hand.cpp test/fake_hand.cpp)
  target_link_libraries(test_serial_hand g1_inspire_protocol)

  # The component through its lifecycle against the same fake, and its plugin export.
  ament_add_gmock(test_system test/test_system.cpp test/fake_hand.cpp)
  target_link_libraries(test_system
    g1_inspire_system pluginlib::pluginlib hardware_interface::hardware_interface)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
