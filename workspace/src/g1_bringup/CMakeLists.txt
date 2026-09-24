cmake_minimum_required(VERSION 3.16)
project(g1_bringup)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

find_package(ament_cmake REQUIRED)

# --- install ---------------------------------------------------------------

install(DIRECTORY config launch mjcf DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS scripts/activate_arm scripts/deactivate_arm
  DESTINATION lib/${PROJECT_NAME}
)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)

  # ament_cmake_xmllint only covers package.xml, and MuJoCo loads scenes that are not valid XML,
  # such as a "--" inside a comment.
  find_program(XMLLINT_EXECUTABLE xmllint)
  if(XMLLINT_EXECUTABLE)
    file(GLOB _scene_files "${CMAKE_CURRENT_SOURCE_DIR}/mjcf/*.xml")
    add_test(
      NAME xmllint_scenes_${PROJECT_NAME}
      COMMAND ${XMLLINT_EXECUTABLE} --noout ${_scene_files}
    )
  else()
    message(WARNING "xmllint not found; skipping xmllint_scenes_${PROJECT_NAME}")
  endif()

  # MuJoCo syncs to CPU time and the policy runs on wall timers, so the simulator suites are load
  # sensitive: RESOURCE_LOCK serialises them within this ctest run and COST starts the heavy ones
  # first. CI skips the `simulator` label, since on shared cores they measure the runner.
  add_launch_test(test/test_lidar_geometry.launch.py TARGET test_lidar_geometry TIMEOUT 300)
  set_tests_properties(test_lidar_geometry PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator)

  # Here rather than in g1_state_estimation, which this package depends on: launching
  # sim.launch.py from there would be a cycle.
  add_launch_test(test/test_fastlio_odometry.launch.py TARGET test_fastlio_odometry TIMEOUT 300)
  set_tests_properties(test_fastlio_odometry PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 90)

  # The rt/lowcmd stack under the AGILE policy: standing on its own balance, then walking.
  add_launch_test(test/test_agile_walk.launch.py TARGET test_agile_walk TIMEOUT 360)
  set_tests_properties(test_agile_walk PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 100)

  # DDS drain gap between simulator suites.
  add_test(NAME sim_settle_gap COMMAND ${CMAKE_COMMAND} -E sleep 5)
  set_tests_properties(sim_settle_gap PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 50)

  g1_add_lint_tests()
endif()

ament_package()
