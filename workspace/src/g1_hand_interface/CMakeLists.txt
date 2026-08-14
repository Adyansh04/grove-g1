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
      NAME clang_format_check_g1_hand_interface
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_hand_interface test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_hand_interface
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_hand_interface PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_hand_interface test")
  endif()
endif()

ament_package()
