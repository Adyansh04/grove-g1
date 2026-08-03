cmake_minimum_required(VERSION 3.8)
project(g1_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linters that don't conflict with ruff. No pep257, matching g1_bringup and g1_sim, the two
  # other launch-heavy packages: its D213 wants the docstring summary on the second line and
  # every launch file in this workspace puts it on the first. ament_lint_auto offers no way to
  # pass an ignore, and one new package is not the place to fork the convention.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # There is no /clock on this track -- the simulator links no ROS. Every upstream config
  # this package adapts ships use_sim_time: True, and a single missed block gives that node a
  # clock that never advances, which surfaces as TF lookups failing somewhere unrelated.
  ament_add_pytest_test(test_no_sim_time test/test_no_sim_time.py
    ENV "G1_NAVIGATION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
  )

  # Headless sim integration suites. RESOURCE_LOCK matches g1_bringup's: two of these sharing
  # a DDS graph and a simulator is what makes them flaky, so they serialise against the same
  # lock g1_bringup uses rather than a separate one.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_scan_pipeline.launch.py TARGET test_scan_pipeline TIMEOUT 200)
  add_launch_test(test/test_slam_map.launch.py TARGET test_slam_map TIMEOUT 240)
  set_tests_properties(test_scan_pipeline PROPERTIES RESOURCE_LOCK g1_sim)
  set_tests_properties(test_slam_map PROPERTIES RESOURCE_LOCK g1_sim)
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
    )
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_navigation test")
  endif()
endif()

ament_package()
