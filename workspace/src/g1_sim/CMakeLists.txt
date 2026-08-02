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
endif()

ament_package()
