cmake_minimum_required(VERSION 3.8)
project(g1_bringup)

find_package(ament_cmake REQUIRED)

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

  # The scene overlays, checked as XML. ament_cmake_xmllint only covers package.xml, and
  # MuJoCo's own parser is lenient enough to accept files xmllint rejects -- a "--" inside a
  # comment is illegal XML but loads fine in the simulator, so it survives until something
  # else reads the file. Cheap to check, and it has caught that exact mistake twice.
  find_program(XMLLINT_EXECUTABLE xmllint)
  if(XMLLINT_EXECUTABLE)
    file(GLOB _scene_files "${CMAKE_CURRENT_SOURCE_DIR}/mjcf/*.xml")
    add_test(
      NAME xmllint_scenes_g1_bringup
      COMMAND ${XMLLINT_EXECUTABLE} --noout ${_scene_files}
    )
  else()
    message(WARNING "xmllint not found; skipping xmllint_scenes_g1_bringup")
  endif()

  # No clang-format check: this package builds no C++.
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
    # From the build dir, ruff.toml's relative src= would not resolve and isort finds nothing.
    set_tests_properties(ruff_check_g1_bringup PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_bringup test")
  endif()

  # LOAD SENSITIVITY: sim suites are CPU-time-sensitive (MuJoCo syncs to CPU
  # time; walking policy uses wall timers). Run individually if results are
  # flaky. RESOURCE_LOCK serializes them; COST ordering puts heavy suites
  # first and the balance test (test_walk_stand) last.
  #
  # That sensitivity is also why they carry LABELS simulator: CI runs on two shared cores,
  # where these do not measure the stack so much as the runner. `ctest -LE simulator` skips
  # every one of them, and a new sim suite inherits the exclusion by being labelled rather
  # than by someone remembering to add it to a list in a workflow file.
  #
  # These share one RESOURCE_LOCK, which only serialises within a single ctest invocation --
  # splitting them across packages would silently drop that.
  find_package(launch_testing_ament_cmake REQUIRED)
  # Converged sensor track. Own MuJoCo instance, so it shares the g1_bringup lock.
  add_launch_test(test/test_lidar_geometry.launch.py TARGET test_lidar_geometry TIMEOUT 300)
  set_tests_properties(test_lidar_geometry PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator)

  # FAST-LIO odometry against MuJoCo ground truth on a real walk. Lives here rather than in
  # g1_state_estimation because it launches sim.launch.py, and that dependency edge would be
  # a cycle from there.
  add_launch_test(test/test_fastlio_odometry.launch.py TARGET test_fastlio_odometry TIMEOUT 300)
  set_tests_properties(test_fastlio_odometry PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator COST 90)

  # The rt/lowcmd stack under the AGILE policy: standing on its own balance, then walking.
  add_launch_test(test/test_agile_walk.launch.py TARGET test_agile_walk TIMEOUT 360)
  set_tests_properties(test_agile_walk PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator COST 100)

  # DDS drain gap between simulator suites.
  set(_SIM_SETTLE_S 5)
  add_test(NAME sim_settle_gap COMMAND ${CMAKE_COMMAND} -E sleep ${_SIM_SETTLE_S})
  set_tests_properties(sim_settle_gap PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator COST 50)
endif()

ament_package()
