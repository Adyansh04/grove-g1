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

# ONNX Runtime ships no CMake config package in its prebuilt tarball, so find it by path. The
# location is fixed by .devcontainer/Dockerfile; failing here rather than at link time keeps the
# error actionable, matching sim.launch.py's own environment preconditions.
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

# Observation/action math for the sim walking policy -- pure C++, no ROS and no ONNX Runtime, so
# the wire contract (joint order, 99-element layout, dead-man) is testable on its own.
add_library(walk_policy src/walk_policy.cpp)
target_include_directories(walk_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(walk_policy PUBLIC cxx_std_17)
ament_target_dependencies(walk_policy PUBLIC unitree_hg)

# The ONNX Runtime session, kept separate from the math above so the observation/action contract
# stays testable without loading a model.
add_library(walk_policy_session src/walk_policy_session.cpp)
target_include_directories(walk_policy_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_INCLUDE_DIR}
)
target_compile_features(walk_policy_session PUBLIC cxx_std_17)
target_link_libraries(walk_policy_session walk_policy ${ONNXRUNTIME_LIBRARY})

# Sim-only LocoClient FSM legality table behind motion_service_sim's /api/sport/request
# responder -- pure C++, no ROS deps, same internal-build-artifact treatment as blend_math.
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

# blend_math is an internal build artifact (statically linked into
# motion_service_sim and test_blend_math/test_assemble_sim_low_cmd only) --
# no install rule: installing the bare archive with neither its headers nor
# an exported target would be unusable downstream anyway.
install(TARGETS motion_service_sim DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch mjcf DESTINATION share/${PROJECT_NAME})

# walker.onnx and walker.onnx.data must land in the SAME directory: the graph names its external
# weight file by relative filename, and ONNX Runtime resolves it against the model's own path.
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

  # The pure math behind the bridge's weight decay/resume and q/kp/kd blend
  # -- directly testable without a live sim or DDS.
  ament_add_gmock(test_blend_math test/test_blend_math.cpp)
  target_link_libraries(test_blend_math blend_math)

  # publishTick()'s /lowcmd assembly, extracted into assembleSimLowCmd() --
  # leg/waist stiff-hold slots+gains, arm blend, weight-slot echo -- same
  # directly-testable-without-a-live-node treatment as test_blend_math.
  ament_add_gmock(test_assemble_sim_low_cmd test/test_assemble_sim_low_cmd.cpp)
  target_link_libraries(test_assemble_sim_low_cmd blend_math)

  # The sim-only LocoClient FSM legality table -- every legal/illegal SET_FSM_ID edge and
  # SET_VELOCITY's Start-only gate, directly testable without a live node or DDS.
  ament_add_gmock(test_loco_fsm test/test_loco_fsm.cpp)
  target_link_libraries(test_loco_fsm loco_fsm)

  # The walking policy's wire contract: DDS joint order, the 99-element observation layout,
  # action->target mapping, and the velocity dead-man -- no ONNX Runtime needed.
  ament_add_gmock(test_walk_policy test/test_walk_policy.cpp)
  target_link_libraries(test_walk_policy walk_policy)

  # Loads the vendored policy through ONNX Runtime: the 99->29 shape contract, determinism,
  # external-weight resolution, and that inference fits the 50 Hz timer's budget.
  ament_add_gmock(test_walk_policy_session test/test_walk_policy_session.cpp)
  target_link_libraries(test_walk_policy_session walk_policy_session)
  ament_target_dependencies(test_walk_policy_session ament_index_cpp)

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
  add_launch_test(test/test_loco.launch.py TARGET test_loco TIMEOUT 180)
  set_tests_properties(test_loco PROPERTIES RESOURCE_LOCK g1_sim)

  # The walking policy holding the robot up with no pelvis weld -- the milestone's core claim.
  add_launch_test(test/test_walk_stand.launch.py TARGET test_walk_stand TIMEOUT 240)
  set_tests_properties(test_walk_stand PROPERTIES RESOURCE_LOCK g1_sim)
endif()

ament_package()
