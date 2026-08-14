cmake_minimum_required(VERSION 3.8)
project(g1_sensor_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Framing and validation, free of ROS and sockets so the wire format tests without a
# simulator or a running graph.
add_library(frame_reader src/frame_reader.cpp)
target_include_directories(frame_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(frame_reader PUBLIC cxx_std_20)

add_executable(g1_sensor_relay src/g1_sensor_relay_node.cpp)
target_link_libraries(g1_sensor_relay PUBLIC frame_reader)
ament_target_dependencies(g1_sensor_relay PUBLIC rclcpp sensor_msgs geometry_msgs vision_msgs
  tf2 tf2_geometry_msgs tf2_ros)

# The conversion itself, split out so it tests without a graph.
add_library(livox_custom_msg src/livox_custom_msg.cpp)
target_include_directories(livox_custom_msg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(livox_custom_msg PUBLIC cxx_std_20)
ament_target_dependencies(livox_custom_msg PUBLIC rclcpp sensor_msgs livox_ros_driver2)

# Restates the relay's own output in the format FAST-LIO wants. Separate executable so the
# odometry track can be started and stopped without touching the sensor path everything else
# depends on.
add_executable(g1_livox_bridge src/g1_livox_bridge_node.cpp)
target_link_libraries(g1_livox_bridge PUBLIC livox_custom_msg)
ament_target_dependencies(g1_livox_bridge PUBLIC rclcpp sensor_msgs livox_ros_driver2)

# This package parses length-prefixed binary frames off a socket, so -Wsign-compare and the
# unused-result warnings earn their keep here more than anywhere else in the workspace.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(frame_reader PRIVATE -Wall -Wextra)
  target_compile_options(g1_sensor_relay PRIVATE -Wall -Wextra)
  target_compile_options(livox_custom_msg PRIVATE -Wall -Wextra)
  target_compile_options(g1_livox_bridge PRIVATE -Wall -Wextra)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS frame_reader livox_custom_msg EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_sensor_relay g1_livox_bridge DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_frame_reader test/test_frame_reader.cpp)
  target_link_libraries(test_frame_reader frame_reader)

  ament_add_gmock(test_livox_custom_msg test/test_livox_custom_msg.cpp)
  target_link_libraries(test_livox_custom_msg livox_custom_msg)
  # The wire-format drift check reads both copies of sensor_frame.h by relative path.
  set_tests_properties(test_frame_reader PROPERTIES
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

  # Lint checks use workspace-root configs (see g1_description/CMakeLists.txt).
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_sensor_relay
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_sensor_relay test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_sensor_relay
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_sensor_relay PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_sensor_relay test")
  endif()
endif()

ament_package()
