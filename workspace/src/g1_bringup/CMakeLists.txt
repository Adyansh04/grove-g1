cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(g1_hardware_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(unitree_go REQUIRED)

# ONNX Runtime has no CMake config — find by path (installed by Dockerfile).
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h HINTS /opt/onnxruntime/include)
find_library(ONNXRUNTIME_LIBRARY onnxruntime HINTS /opt/onnxruntime/lib)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found under /opt/onnxruntime. It is installed by "
    ".devcontainer/Dockerfile -- run './scripts/manage.sh recreate' from the host.")
endif()

add_library(blend_math src/blend_math.cpp)
target_include_directories(blend_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(blend_math PUBLIC cxx_std_17)
ament_target_dependencies(blend_math PUBLIC unitree_hg)

# Walking policy observation/action math — pure C++, no ROS or ONNX.
add_library(walk_policy src/walk_policy.cpp)
target_include_directories(walk_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(walk_policy PUBLIC cxx_std_17)
ament_target_dependencies(walk_policy PUBLIC unitree_hg)

# ONNX Runtime session wrapper, separate from math for testability.
add_library(walk_policy_session src/walk_policy_session.cpp)
target_include_directories(walk_policy_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_compile_features(walk_policy_session PUBLIC cxx_std_17)
target_link_libraries(walk_policy_session walk_policy ${ONNXRUNTIME_LIBRARY})

# Sim-only FSM legality table for /api/sport/request.
add_library(loco_fsm src/loco_fsm.cpp)
target_include_directories(loco_fsm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(loco_fsm PUBLIC cxx_std_17)

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
  loco_fsm
  walk_policy_session
  g1_hardware_interface::g1_arm_sdk_system
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(motion_service_sim rclcpp unitree_hg unitree_api unitree_go ament_index_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(motion_service_sim PRIVATE -Wall -Wextra)
  target_compile_options(blend_math PRIVATE -Wall -Wextra)
  target_compile_options(loco_fsm PRIVATE -Wall -Wextra)
endif()

# blend_math is internal (statically linked) — no install rule.
install(TARGETS motion_service_sim DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch mjcf DESTINATION share/${PROJECT_NAME})

# walker.onnx and .data must be co-located (relative weight file resolution).
install(DIRECTORY policy DESTINATION share/${PROJECT_NAME})

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

  # Pure math tests — no live sim or DDS needed.
  ament_add_gmock(test_blend_math test/test_blend_math.cpp)
  target_link_libraries(test_blend_math blend_math)

  # /lowcmd assembly tests — no live node needed.
  ament_add_gmock(test_assemble_sim_low_cmd test/test_assemble_sim_low_cmd.cpp)
  target_link_libraries(test_assemble_sim_low_cmd blend_math)

  # FSM legality table tests.
  ament_add_gmock(test_loco_fsm test/test_loco_fsm.cpp)
  target_link_libraries(test_loco_fsm loco_fsm)

  # Walking policy wire contract tests — no ONNX Runtime needed.
  ament_add_gmock(test_walk_policy test/test_walk_policy.cpp)
  target_link_libraries(test_walk_policy walk_policy)

  # ONNX Runtime session tests.
  ament_add_gmock(test_walk_policy_session test/test_walk_policy_session.cpp)
  target_link_libraries(test_walk_policy_session walk_policy_session)
  ament_target_dependencies(test_walk_policy_session ament_index_cpp)

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
    # ruff skips extensionless files — name them explicitly.
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

  # LOAD SENSITIVITY: sim suites are CPU-time-sensitive (MuJoCo syncs to CPU
  # time; walking policy uses wall timers). Run individually if results are
  # flaky. RESOURCE_LOCK serializes them; COST ordering puts heavy suites
  # first and the balance test (test_walk_stand) last.
  # Headless sim integration suites. RESOURCE_LOCK prevents DDS graph
  # collisions across parallel ctest runs.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_bringup.launch.py TARGET test_sim_bringup TIMEOUT 120)
  # Converged sensor track. Own MuJoCo instance, so it shares the g1_bringup lock.
  add_launch_test(test/test_lidar_geometry.launch.py TARGET test_lidar_geometry TIMEOUT 300)
  set_tests_properties(test_sim_bringup PROPERTIES RESOURCE_LOCK g1_sim)
  set_tests_properties(test_lidar_geometry PROPERTIES RESOURCE_LOCK g1_sim)
  add_launch_test(test/test_arm_command.launch.py TARGET test_arm_command TIMEOUT 220)
  set_tests_properties(test_arm_command PROPERTIES RESOURCE_LOCK g1_sim)
  add_launch_test(test/test_loco.launch.py TARGET test_loco TIMEOUT 180)
  set_tests_properties(test_loco PROPERTIES RESOURCE_LOCK g1_sim)

  # Core milestone claim: walking policy keeps the robot upright.
  add_launch_test(test/test_walk_stand.launch.py TARGET test_walk_stand TIMEOUT 240)
  set_tests_properties(test_walk_stand PROPERTIES RESOURCE_LOCK g1_sim)

  # Full LocoClient authority path with rejection and randomized commands.
  add_launch_test(test/test_walk_teleop.launch.py TARGET test_walk_teleop TIMEOUT 400)
  set_tests_properties(test_walk_teleop PROPERTIES RESOURCE_LOCK g1_sim)

  # Acceptance bar: walking + arm trajectory in one session.
  add_launch_test(test/test_walk_and_arm.launch.py TARGET test_walk_and_arm TIMEOUT 400)
  set_tests_properties(test_walk_and_arm PROPERTIES RESOURCE_LOCK g1_sim)

  # DDS drain gap before the critical balance test.
  set(_SIM_SETTLE_S 5)
  add_test(NAME sim_settle_gap COMMAND ${CMAKE_COMMAND} -E sleep ${_SIM_SETTLE_S})
  set_tests_properties(sim_settle_gap PROPERTIES RESOURCE_LOCK g1_sim COST 50)

  # COST descending = execution order.
  set_tests_properties(test_walk_teleop PROPERTIES COST 100)
  set_tests_properties(test_walk_and_arm PROPERTIES COST 90)
  set_tests_properties(test_arm_command PROPERTIES COST 80)
  set_tests_properties(test_loco PROPERTIES COST 70)
  set_tests_properties(test_sim_bringup PROPERTIES COST 60)
  set_tests_properties(test_walk_stand PROPERTIES COST 10)
endif()

ament_package()
