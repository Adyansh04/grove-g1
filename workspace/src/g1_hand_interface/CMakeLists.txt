cmake_minimum_required(VERSION 3.8)
project(g1_hand_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(unitree_hg REQUIRED)

add_library(g1_dex3_system SHARED src/g1_dex3_system.cpp)
target_include_directories(g1_dex3_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_dex3_system PUBLIC cxx_std_20)
ament_target_dependencies(g1_dex3_system PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools unitree_hg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_dex3_system PRIVATE -Wall -Wextra)
endif()

pluginlib_export_plugin_description_file(hardware_interface g1_hand_interface.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_dex3_system EXPORT export_${PROJECT_NAME} DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp realtime_tools unitree_hg)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # The wire format, plus the plugin discovery proof: a clean compile says nothing about
  # whether controller_manager can actually find the class through its XML export.
  ament_add_gmock(test_wire_contract test/test_wire_contract.cpp)
  target_link_libraries(test_wire_contract g1_dex3_system)
  ament_target_dependencies(test_wire_contract pluginlib hardware_interface)
endif()

ament_package()
