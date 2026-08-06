cmake_minimum_required(VERSION 3.8)
project(g1_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Linters that don't conflict with ruff. No pep257, matching g1_bringup, g1_sim and
  # g1_navigation, the other launch-heavy packages.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # Reads three packages' config at once: MoveIt's arm, the controller's arm and the bridge's
  # speed clamp all have to agree, and no single package's tests can see the disagreement.
  ament_add_pytest_test(test_moveit_config_drift test/test_moveit_config_drift.py
    ENV "G1_MOVEIT_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
        "G1_BRINGUP_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/config"
        "G1_DESCRIPTION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_description/config"
        "G1_SIM_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_motion_service_sim/config"
  )

  # bringup's moveit branch names this package's share, so its threading assertions cannot
  # live in g1_navigation alongside the navigation ones. Source directories rather than
  # installed shares, same as above: these read launch files, not built artifacts.
  ament_add_pytest_test(test_launch_threading test/test_launch_threading.py
    ENV "G1_MOVEIT_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/launch"
        "G1_BRINGUP_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/launch"
  )

  # move_group plans against the expanded xacro, not a checked-in URDF, so the model test has
  # to expand it too -- otherwise it validates a robot nobody runs. Regenerated whenever the
  # xacro or the gains it loads change.
  find_package(moveit_core REQUIRED)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)

  set(_expanded_urdf "${CMAKE_CURRENT_BINARY_DIR}/g1_expanded.urdf")
  set(_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/../g1_description")
  add_custom_command(
    OUTPUT ${_expanded_urdf}
    COMMAND xacro ${_description_dir}/urdf/g1_arm_sdk.urdf.xacro -o ${_expanded_urdf}
    DEPENDS ${_description_dir}/urdf/g1_arm_sdk.urdf.xacro
            ${_description_dir}/urdf/g1_29dof_with_hand_rev_1_0.urdf
            ${_description_dir}/config/arm_sdk_params.yaml
    COMMENT "Expanding g1_arm_sdk.urdf.xacro for test_robot_model"
    VERBATIM
  )
  add_custom_target(g1_moveit_config_expanded_urdf ALL DEPENDS ${_expanded_urdf})

  ament_add_gmock(test_robot_model test/test_robot_model.cpp)
  if(TARGET test_robot_model)
    add_dependencies(test_robot_model g1_moveit_config_expanded_urdf)
    target_compile_definitions(test_robot_model PRIVATE
      G1_TEST_URDF_PATH="${_expanded_urdf}"
      G1_TEST_SRDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/g1.srdf")
    ament_target_dependencies(test_robot_model moveit_core urdf srdfdom)
  endif()

  # Headless sim suite. Shares g1_bringup's RESOURCE_LOCK: two suites on one DDS graph and one
  # simulator is what makes them flaky.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_moveit_plan_execute.launch.py
    TARGET test_moveit_plan_execute TIMEOUT 400)
  set_tests_properties(test_moveit_plan_execute PROPERTIES RESOURCE_LOCK g1_sim COST 90)

  # Runs the sensor track on the perception scene: the octomap has to fill from the LiDAR
  # before anything can be asserted. Note SIM_SETTLE_S inside that file cannot be raised to
  # cover the fill -- launch_testing's own startup timeout fires first.
  add_launch_test(test/test_octomap_blocks_a_plan.launch.py
    TARGET test_octomap_blocks_a_plan TIMEOUT 500)
  set_tests_properties(test_octomap_blocks_a_plan PROPERTIES RESOURCE_LOCK g1_sim COST 95)

  # Use workspace-root ruff.toml if available.
  find_program(RUFF_EXECUTABLE ruff)
  set(_ruff_config "${CMAKE_CURRENT_SOURCE_DIR}/../../ruff.toml")
  if(RUFF_EXECUTABLE AND EXISTS "${_ruff_config}")
    add_test(
      NAME ruff_check_g1_moveit_config
      COMMAND ${RUFF_EXECUTABLE} check --config ${_ruff_config}
              ${CMAKE_CURRENT_SOURCE_DIR}/launch ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_moveit_config test")
  endif()
endif()

ament_package()
