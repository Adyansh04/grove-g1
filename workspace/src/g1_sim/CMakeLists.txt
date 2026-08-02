cmake_minimum_required(VERSION 3.8)
project(g1_sim)

find_package(ament_cmake REQUIRED)

# No compiled targets: this package is MJCF, URDF, config, launch and tests.
install(DIRECTORY mjcf urdf config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # The MJCF and URDF necessarily carry the same mount numbers; this guards the drift.
  # No sim and no ROS -- it just parses both files, plus the vendored G1 URDF they derive from.
  ament_add_pytest_test(test_sensor_mount_consistency test/test_sensor_mount_consistency.py)

  # Headless sim integration. RESOURCE_LOCK g1_sim is the SAME lock name g1_bringup uses:
  # both tracks start a real MuJoCo and starve each other on CPU if run concurrently.
  # Note ctest parallelises across packages, so a full-workspace sweep still needs
  # `colcon test --parallel-workers 1`.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_perception_sim_bringup.launch.py
    TARGET test_perception_sim_bringup TIMEOUT 240)
  set_tests_properties(test_perception_sim_bringup PROPERTIES RESOURCE_LOCK g1_sim)
endif()

ament_package()
