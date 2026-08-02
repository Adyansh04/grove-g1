cmake_minimum_required(VERSION 3.8)
project(g1_sim)

find_package(ament_cmake REQUIRED)

# No compiled targets: this package is MJCF, URDF, config, launch and tests.
install(DIRECTORY mjcf urdf config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Lint checks use workspace-root configs (see g1_description/CMakeLists.txt). This
  # package is the most Python-heavy in the repo, so its absence here was hiding real
  # errors behind a green suite.
  find_program(RUFF_EXECUTABLE ruff)
  set(_ruff_config "${CMAKE_CURRENT_SOURCE_DIR}/../../ruff.toml")
  if(RUFF_EXECUTABLE AND EXISTS "${_ruff_config}")
    add_test(
      NAME ruff_check_g1_sim
      COMMAND ${RUFF_EXECUTABLE} check --config ${_ruff_config}
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_sim test")
  endif()

  find_package(ament_cmake_pytest REQUIRED)

  # The MJCF and URDF necessarily carry the same mount numbers; this guards the drift.
  # No sim and no ROS -- it just parses both files, plus the vendored G1 URDF they derive from.
  ament_add_pytest_test(test_sensor_mount_consistency test/test_sensor_mount_consistency.py)

  # Headless sim integration. RESOURCE_LOCK g1_sim is the SAME lock name g1_bringup uses:
  # both tracks start a real MuJoCo and starve each other on CPU if run concurrently.
  # Note ctest parallelises across packages, so a full-workspace sweep still needs
  # `colcon test --parallel-workers 1`.
  find_package(launch_testing_ament_cmake REQUIRED)
  # Each of these starts its own MuJoCo, so they share the lock rather than overlap.
  add_launch_test(test/test_perception_sim_bringup.launch.py
    TARGET test_perception_sim_bringup TIMEOUT 240)
  add_launch_test(test/test_lidar_stream.launch.py
    TARGET test_lidar_stream TIMEOUT 300)
  add_launch_test(test/test_camera_stream.launch.py
    TARGET test_camera_stream TIMEOUT 300)
  add_launch_test(test/test_odom_ground_truth.launch.py
    TARGET test_odom_ground_truth TIMEOUT 300)
  set_tests_properties(test_perception_sim_bringup test_lidar_stream test_camera_stream
    test_odom_ground_truth PROPERTIES RESOURCE_LOCK g1_sim)
endif()

ament_package()
