cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(g1_hardware_interface REQUIRED)

add_library(blend_math src/blend_math.cpp)
target_include_directories(blend_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(blend_math PUBLIC cxx_std_17)

add_executable(arm_sdk_sim_bridge
  src/arm_sdk_sim_bridge_node.cpp
  src/arm_sdk_sim_bridge_main.cpp
)
target_include_directories(arm_sdk_sim_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(arm_sdk_sim_bridge PUBLIC cxx_std_17)
target_link_libraries(arm_sdk_sim_bridge
  blend_math
  g1_hardware_interface::g1_arm_sdk_system
)
ament_target_dependencies(arm_sdk_sim_bridge rclcpp unitree_hg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(arm_sdk_sim_bridge PRIVATE -Wall -Wextra)
  target_compile_options(blend_math PRIVATE -Wall -Wextra)
endif()

install(
  TARGETS blend_math
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS arm_sdk_sim_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # The pure math behind the bridge's weight decay/resume and q/kp/kd blend
  # -- directly testable without a live sim or DDS.
  ament_add_gmock(test_blend_math test/test_blend_math.cpp)
  target_link_libraries(test_blend_math blend_math)

  # workspace/ruff.toml and .clang-format are the workspace-root configs
  # bind-mounted alongside src/ (see g1_description/CMakeLists.txt for why
  # this is a plain CTest instead of ament_lint_common's uncrustify/flake8).
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_bringup
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_bringup test")
  endif()
endif()

ament_package()
