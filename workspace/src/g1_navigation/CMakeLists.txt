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
  # No pep257: its D213 wants docstring summaries on the second line and cannot be ignored.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # One env block for every pytest test: pytest 8+ imports the whole directory while collecting
  # any one file, so a sibling's missing variable breaks collection.
  set(_pytest_env
    "G1_NAVIGATION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
    "G1_NAVIGATION_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/launch"
    "G1_BRINGUP_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/config"
    "G1_BRINGUP_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/launch"
  )

  # No /clock on this track, and every upstream config adapted here ships use_sim_time: True.
  ament_add_pytest_test(test_no_sim_time test/test_no_sim_time.py ENV ${_pytest_env})

  # Arguments crossing bringup.launch.py and nav_sim.launch.py into their includes. Lives here
  # because it reads both packages' launch files, and g1_bringup cannot depend on this one.
  ament_add_pytest_test(test_launch_threading test/test_launch_threading.py ENV ${_pytest_env})

  # The Sensors group is duplicated in two packages' RViz configs, so neither can check it alone.
  ament_add_pytest_test(test_rviz_configs test/test_rviz_configs.py ENV ${_pytest_env})

  # Simulator suites share g1_bringup's RESOURCE_LOCK, since two sharing a DDS graph and a
  # simulator flake. Higher COST runs first.
  add_launch_test(test/test_scan_pipeline.launch.py TARGET test_scan_pipeline TIMEOUT 200)
  set_tests_properties(test_scan_pipeline PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 50)

  add_launch_test(test/test_slam_map.launch.py TARGET test_slam_map TIMEOUT 240)
  set_tests_properties(test_slam_map PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 60)

  # The acceptance gate and the only suite that drives the robot; first, on the quietest machine.
  add_launch_test(test/test_navigate_to_pose.launch.py TARGET test_navigate_to_pose TIMEOUT 500)
  set_tests_properties(test_navigate_to_pose PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 100)

  g1_add_lint_tests()
endif()

ament_package()
