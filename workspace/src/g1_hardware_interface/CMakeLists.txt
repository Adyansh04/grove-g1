cmake_minimum_required(VERSION 3.8)
project(g1_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_hg REQUIRED)

add_library(g1_arm_sdk_system SHARED
  src/arm_ramp_engine.cpp
  src/motor_crc_hg.cpp
  src/g1_arm_sdk_system.cpp
)
target_include_directories(g1_arm_sdk_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_arm_sdk_system PUBLIC cxx_std_20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_arm_sdk_system PRIVATE -Wall -Wextra)
endif()
ament_target_dependencies(g1_arm_sdk_system PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  unitree_hg
)

# Makes the plugin discoverable via pluginlib's ament-index lookup -- the
# same path controller_manager uses to load it by name from the URDF.
pluginlib_export_plugin_description_file(hardware_interface g1_hardware_interface.xml)

# The legs and waist as JointState. Split from the node so the mapping tests without a graph.
add_library(lowstate_joint_states src/lowstate_joint_states.cpp)
target_include_directories(lowstate_joint_states PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(lowstate_joint_states PUBLIC cxx_std_20)
ament_target_dependencies(lowstate_joint_states PUBLIC sensor_msgs unitree_hg)

# Its own executable, not part of the plugin: the transform it completes has to exist whether
# or not an arm component is loaded.
add_executable(g1_lowstate_joint_states src/g1_lowstate_joint_states_node.cpp)
target_link_libraries(g1_lowstate_joint_states PRIVATE lowstate_joint_states)
ament_target_dependencies(g1_lowstate_joint_states PUBLIC rclcpp sensor_msgs unitree_hg)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(lowstate_joint_states PRIVATE -Wall -Wextra)
  target_compile_options(g1_lowstate_joint_states PRIVATE -Wall -Wextra)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS g1_arm_sdk_system lowstate_joint_states
  EXPORT export_g1_arm_sdk_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS g1_lowstate_joint_states DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # Loading the plugin by its registered name through pluginlib's own
  # ClassLoader is the actual proof controller_manager will find it -- a
  # clean compile doesn't guarantee the plugin XML/export is wired correctly.
  ament_add_gmock(test_pluginlib_loading test/test_pluginlib_loading.cpp)
  target_link_libraries(test_pluginlib_loading g1_arm_sdk_system)
  ament_target_dependencies(test_pluginlib_loading pluginlib hardware_interface)

  # The safety-critical surface: pure, ROS-free ramp/slew/staleness logic --
  # no pluginlib, no rclcpp node, no sim needed to exercise it.
  ament_add_gmock(test_arm_ramp_engine test/test_arm_ramp_engine.cpp)
  target_link_libraries(test_arm_ramp_engine g1_arm_sdk_system)

  # LowCmd slot assembly (arm/weight slots touched, everything else stays
  # zeroed) -- directly testable without a live hardware component.
  ament_add_gmock(test_assemble_low_cmd test/test_assemble_low_cmd.cpp)
  target_link_libraries(test_assemble_low_cmd g1_arm_sdk_system)
  ament_target_dependencies(test_assemble_low_cmd unitree_hg)

  ament_add_gmock(test_lowstate_joint_states test/test_lowstate_joint_states.cpp)
  target_link_libraries(test_lowstate_joint_states lowstate_joint_states)

  # workspace/ruff.toml's sibling root config; see g1_description/CMakeLists.txt
  # for why this is a plain CTest instead of ament_lint_common's uncrustify.
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_hardware_interface
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_hardware_interface test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_hardware_interface
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_hardware_interface PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_hardware_interface test")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(g1_arm_sdk_system)
ament_export_targets(export_g1_arm_sdk_system)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  unitree_hg
)

ament_package()
