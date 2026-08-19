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

  # ament_cmake_xmllint only covers package.xml, and MuJoCo's parser accepts files xmllint
  # rejects -- a "--" inside a comment is illegal XML but loads fine, so it survives until
  # something else reads the file. Caught that exact mistake twice.
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

  # Simulator suites. MuJoCo syncs to CPU time and the walking policy uses wall timers, so these
  # are CPU-time-sensitive: RESOURCE_LOCK serialises them and COST orders the heavy ones
  # first. The lock only serialises within one ctest invocation, so splitting these across
  # packages would silently drop it; g1_navigation's suites share this same lock name.
  #
  # LABELS simulator is what lets CI skip them: on two shared cores they measure the runner
  # rather than the stack, and `ctest -LE simulator` excludes any suite that carries the label
  # without anyone having to maintain a list in a workflow file.
  add_launch_test(test/test_lidar_geometry.launch.py TARGET test_lidar_geometry TIMEOUT 300)
  set_tests_properties(test_lidar_geometry PROPERTIES RESOURCE_LOCK mujoco_sim LABELS simulator)

  # FAST-LIO against MuJoCo ground truth on a real walk. Lives here rather than in
  # g1_state_estimation because it launches sim.launch.py, which would be a cycle from there.
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
