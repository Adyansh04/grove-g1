cmake_minimum_required(VERSION 3.8)
project(g1_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# Soak test and its recorder. Same arrangement as g1_bringup's activate_arm: run in-container
# via `ros2 run g1_navigation nav_soak`, not from the host.
install(
  PROGRAMS
    scripts/nav_soak
    scripts/nav_diag.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linters that don't conflict with ruff. No pep257, matching g1_bringup, the
  # other launch-heavy packages: its D213 wants the docstring summary on the second line and
  # every launch file in this workspace puts it on the first. ament_lint_auto offers no way to
  # pass an ignore, and one new package is not the place to fork the convention.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # One env block for every pytest test here, not one per test: pytest 8+ imports every module
  # in the directory while collecting any single file, so a sibling's missing variable is a
  # collection error for tests that never read it. test/conftest.py handles the launch files.
  set(_pytest_env
    "G1_NAVIGATION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
    "G1_NAVIGATION_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/launch"
    "G1_BRINGUP_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/config"
    "G1_BRINGUP_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/launch"
  )

  # There is no /clock on this track -- the simulator links no ROS. Every upstream config
  # this package adapts ships use_sim_time: True, and a single missed block gives that node a
  # clock that never advances, which surfaces as TF lookups failing somewhere unrelated.
  ament_add_pytest_test(test_no_sim_time test/test_no_sim_time.py ENV ${_pytest_env})

  # Arguments crossing bringup.launch.py -> nav_sim.launch.py -> sim.launch.py. Lives here
  # rather than in g1_bringup because it reads both packages' launch files, and g1_bringup
  # cannot depend on g1_navigation without a colcon cycle. Needs no simulator.
  ament_add_pytest_test(test_launch_threading test/test_launch_threading.py ENV ${_pytest_env})

  # The sensor display group is duplicated across two RViz configs in two packages, so neither
  # package's own tests can see the pair.
  ament_add_pytest_test(test_rviz_configs test/test_rviz_configs.py ENV ${_pytest_env})

  # Headless sim integration suites. RESOURCE_LOCK matches g1_bringup's: two of these sharing
  # a DDS graph and a simulator is what makes them flaky, so they serialise against the same
  # lock g1_bringup uses rather than a separate one.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_scan_pipeline.launch.py TARGET test_scan_pipeline TIMEOUT 200)
  add_launch_test(test/test_slam_map.launch.py TARGET test_slam_map TIMEOUT 240)
  # The acceptance gate, and the only suite here that drives the robot. Longest timeout and
  # highest COST so it runs first, while the machine is quietest.
  add_launch_test(test/test_navigate_to_pose.launch.py TARGET test_navigate_to_pose TIMEOUT 500)
  set_tests_properties(test_scan_pipeline PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator)
  set_tests_properties(test_slam_map PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator)
  set_tests_properties(test_navigate_to_pose PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator COST 100)
  # COST descending = execution order; the heavier suite goes first.
  set_tests_properties(test_slam_map PROPERTIES COST 60)
  set_tests_properties(test_scan_pipeline PROPERTIES COST 50)

  # Use workspace-root ruff.toml if available.
  find_program(RUFF_EXECUTABLE ruff)
  set(_ruff_config "${CMAKE_CURRENT_SOURCE_DIR}/../../ruff.toml")
  if(RUFF_EXECUTABLE AND EXISTS "${_ruff_config}")
    add_test(
      NAME ruff_check_g1_navigation
      COMMAND ${RUFF_EXECUTABLE} check --config ${_ruff_config}
              ${CMAKE_CURRENT_SOURCE_DIR}/launch ${CMAKE_CURRENT_SOURCE_DIR}/test
              ${CMAKE_CURRENT_SOURCE_DIR}/scripts
    )
    # From the build dir, ruff.toml's relative src= would not resolve and isort finds nothing.
    set_tests_properties(ruff_check_g1_navigation PROPERTIES
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_navigation test")
  endif()
endif()

ament_package()
