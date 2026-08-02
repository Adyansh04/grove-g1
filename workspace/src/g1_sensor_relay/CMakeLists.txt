cmake_minimum_required(VERSION 3.8)
project(g1_sensor_relay)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Framing and validation, free of ROS and sockets so the wire format tests without a
# simulator or a running graph.
add_library(frame_reader src/frame_reader.cpp)
target_include_directories(frame_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(frame_reader PUBLIC cxx_std_17)

add_executable(g1_sensor_relay src/g1_sensor_relay_node.cpp)
target_link_libraries(g1_sensor_relay PUBLIC frame_reader)
ament_target_dependencies(g1_sensor_relay PUBLIC rclcpp sensor_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS frame_reader EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_sensor_relay DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_frame_reader test/test_frame_reader.cpp)
  target_link_libraries(test_frame_reader frame_reader)
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
endif()

ament_package()
