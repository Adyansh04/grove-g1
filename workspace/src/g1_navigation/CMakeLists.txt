cmake_minimum_required(VERSION 3.16)
project(g1_navigation)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

find_package(ament_cmake REQUIRED)

# --- install ---------------------------------------------------------------

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
# Run in-container via `ros2 run g1_navigation nav_soak`, not from the host.
install(
  PROGRAMS scripts/nav_soak scripts/nav_diag.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  # No pep257: its D213 wants the docstring summary on the second line and every launch file
  # here puts it on the first. ament_lint_auto offers no way to pass an ignore.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # One env block for every pytest test, not one per test: pytest 8+ imports every module in
  # the directory while collecting any single file, so a sibling's missing variable becomes a
  # collection error for tests that never read it.
  set(_pytest_env
    "G1_NAVIGATION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
    "G1_NAVIGATION_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/launch"
    "G1_BRINGUP_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/config"
    "G1_BRINGUP_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/launch"
  )

  # There is no /clock on this track. Every upstream config this package adapts ships
  # use_sim_time: True, and a single missed block gives that node a clock that never advances,
  # which surfaces as TF lookups failing somewhere unrelated.
  ament_add_pytest_test(test_no_sim_time test/test_no_sim_time.py ENV ${_pytest_env})

  # Arguments crossing bringup -> nav_sim -> sim. Lives here rather than in g1_bringup because
  # it reads both packages' launch files, and g1_bringup cannot depend on this one.
  ament_add_pytest_test(test_launch_threading test/test_launch_threading.py ENV ${_pytest_env})

  # The sensor display group is duplicated across two RViz configs in two packages, so neither
  # package's own tests can see the pair.
  ament_add_pytest_test(test_rviz_configs test/test_rviz_configs.py ENV ${_pytest_env})

  # Simulator suites. Same RESOURCE_LOCK name as g1_bringup's: two of these sharing a DDS
  # graph and a simulator is what makes them flaky. COST descending is execution order.
  add_launch_test(test/test_scan_pipeline.launch.py TARGET test_scan_pipeline TIMEOUT 200)
  set_tests_properties(test_scan_pipeline PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 50)

  add_launch_test(test/test_slam_map.launch.py TARGET test_slam_map TIMEOUT 240)
  set_tests_properties(test_slam_map PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 60)

  # The acceptance gate, and the only suite that drives the robot. Highest COST so it runs
  # first, while the machine is quietest.
  add_launch_test(test/test_navigate_to_pose.launch.py TARGET test_navigate_to_pose TIMEOUT 500)
  set_tests_properties(test_navigate_to_pose PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 100)

  g1_add_lint_tests()
endif()

ament_package()
