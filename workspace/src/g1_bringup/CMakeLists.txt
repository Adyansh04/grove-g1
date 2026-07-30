cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(g1_hardware_interface REQUIRED)

add_library(blend_math src/blend_math.cpp)
target_include_directories(blend_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(blend_math PUBLIC cxx_std_17)
ament_target_dependencies(blend_math PUBLIC unitree_hg)

add_executable(motion_service_sim
  src/motion_service_sim_node.cpp
  src/motion_service_sim_main.cpp
)
target_include_directories(motion_service_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(motion_service_sim PUBLIC cxx_std_17)
target_link_libraries(motion_service_sim
  blend_math
  g1_hardware_interface::g1_arm_sdk_system
)
ament_target_dependencies(motion_service_sim rclcpp unitree_hg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(motion_service_sim PRIVATE -Wall -Wextra)
  target_compile_options(blend_math PRIVATE -Wall -Wextra)
endif()

# blend_math is an internal build artifact (statically linked into
# motion_service_sim and test_blend_math/test_assemble_sim_low_cmd only) --
# no install rule: installing the bare archive with neither its headers nor
# an exported target would be unusable downstream anyway.
install(TARGETS motion_service_sim DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch mjcf DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/activate_arm
    scripts/deactivate_arm
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # The pure math behind the bridge's weight decay/resume and q/kp/kd blend
  # -- directly testable without a live sim or DDS.
  ament_add_gmock(test_blend_math test/test_blend_math.cpp)
  target_link_libraries(test_blend_math blend_math)

  # publishTick()'s /lowcmd assembly, extracted into assembleSimLowCmd() --
  # leg/waist stiff-hold slots+gains, arm blend, weight-slot echo -- same
  # directly-testable-without-a-live-node treatment as test_blend_math.
  ament_add_gmock(test_assemble_sim_low_cmd test/test_assemble_sim_low_cmd.cpp)
  target_link_libraries(test_assemble_sim_low_cmd blend_math)

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

  find_program(RUFF_EXECUTABLE ruff)
  set(_ruff_config "${CMAKE_CURRENT_SOURCE_DIR}/../../ruff.toml")
  if(RUFF_EXECUTABLE AND EXISTS "${_ruff_config}")
    # scripts/activate_arm and scripts/deactivate_arm are extensionless
    # (ros2 run expects no .py suffix): ruff's directory-mode file discovery
    # silently skips extensionless files ("No Python files found"), so they
    # must be named explicitly -- only launch/ and test/ are safe to hand
    # ruff as bare directories.
    add_test(
      NAME ruff_check_g1_bringup
      COMMAND ${RUFF_EXECUTABLE} check --config ${_ruff_config}
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/activate_arm
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/deactivate_arm
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_bringup test")
  endif()

  # Headless sim integration suites -- both exercise the real unitree_mujoco
  # sim end to end (see each file's own docstring for what it asserts).
  # Generous timeouts: sim.launch.py's own settle window plus the test
  # bodies' own bounded waits (ramp, trajectory, rogue-guard poll).
  #
  # Both hard-require ROS_DOMAIN_ID=1 and bind the same headless Xvfb
  # display -- RESOURCE_LOCK makes ctest serialize them explicitly (rather
  # than relying on ctest's default parallelism happening to be 1), so a
  # parallel run can never let one test's simulator/DDS graph bleed into
  # the other's.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_bringup.launch.py TARGET test_sim_bringup TIMEOUT 120)
  set_tests_properties(test_sim_bringup PROPERTIES RESOURCE_LOCK g1_sim)
  add_launch_test(test/test_arm_command.launch.py TARGET test_arm_command TIMEOUT 220)
  set_tests_properties(test_arm_command PROPERTIES RESOURCE_LOCK g1_sim)
endif()

ament_package()
