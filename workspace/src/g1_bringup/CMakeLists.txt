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

  # No clang-format check: this package builds no C++. motion_service_sim's sources, and
  # their formatting check, live in g1_motion_service_sim.
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
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_bringup test")
  endif()

  # LOAD SENSITIVITY: sim suites are CPU-time-sensitive (MuJoCo syncs to CPU
  # time; walking policy uses wall timers). Run individually if results are
  # flaky. RESOURCE_LOCK serializes them; COST ordering puts heavy suites
  # first and the balance test (test_walk_stand) last.
  #
  # These stay here rather than moving with motion_service_sim's source: every one of
  # them includes this package's sim.launch.py, and g1_bringup depends on
  # g1_motion_service_sim, so hosting them there would be a dependency cycle. They also
  # share one RESOURCE_LOCK, which only serialises within a single ctest invocation --
  # splitting them across packages would silently drop that.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sim_bringup.launch.py TARGET test_sim_bringup TIMEOUT 120)
  # Converged sensor track. Own MuJoCo instance, so it shares the g1_bringup lock.
  add_launch_test(test/test_lidar_geometry.launch.py TARGET test_lidar_geometry TIMEOUT 300)
  set_tests_properties(test_sim_bringup PROPERTIES RESOURCE_LOCK mujoco_sim)
  set_tests_properties(test_lidar_geometry PROPERTIES RESOURCE_LOCK mujoco_sim)
  add_launch_test(test/test_arm_command.launch.py TARGET test_arm_command TIMEOUT 220)
  set_tests_properties(test_arm_command PROPERTIES RESOURCE_LOCK mujoco_sim)
  add_launch_test(test/test_loco.launch.py TARGET test_loco TIMEOUT 180)
  set_tests_properties(test_loco PROPERTIES RESOURCE_LOCK mujoco_sim)

  # Core milestone claim: walking policy keeps the robot upright.
  add_launch_test(test/test_walk_stand.launch.py TARGET test_walk_stand TIMEOUT 240)
  set_tests_properties(test_walk_stand PROPERTIES RESOURCE_LOCK mujoco_sim)

  # Full LocoClient authority path with rejection and randomized commands.
  add_launch_test(test/test_walk_teleop.launch.py TARGET test_walk_teleop TIMEOUT 400)
  set_tests_properties(test_walk_teleop PROPERTIES RESOURCE_LOCK mujoco_sim)

  # Acceptance bar: walking + arm trajectory in one session.
  add_launch_test(test/test_walk_and_arm.launch.py TARGET test_walk_and_arm TIMEOUT 400)
  set_tests_properties(test_walk_and_arm PROPERTIES RESOURCE_LOCK mujoco_sim)

  # DDS drain gap before the critical balance test.
  set(_SIM_SETTLE_S 5)
  add_test(NAME sim_settle_gap COMMAND ${CMAKE_COMMAND} -E sleep ${_SIM_SETTLE_S})
  set_tests_properties(sim_settle_gap PROPERTIES RESOURCE_LOCK mujoco_sim COST 50)

  # COST descending = execution order.
  set_tests_properties(test_walk_teleop PROPERTIES COST 100)
  set_tests_properties(test_walk_and_arm PROPERTIES COST 90)
  set_tests_properties(test_arm_command PROPERTIES COST 80)
  set_tests_properties(test_loco PROPERTIES COST 70)
  set_tests_properties(test_sim_bringup PROPERTIES COST 60)
  set_tests_properties(test_walk_stand PROPERTIES COST 10)
endif()

ament_package()
