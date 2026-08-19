cmake_minimum_required(VERSION 3.16)
project(g1_moveit_config)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

find_package(ament_cmake REQUIRED)

# --- install ---------------------------------------------------------------

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  # No pep257: its D213 wants the docstring summary on the second line and every launch file
  # here puts it on the first.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(moveit_core REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(urdf REQUIRED)

  # One env block for every pytest test, not one per test: pytest 8+ imports every module in
  # the directory while collecting any single file, so a sibling's missing variable becomes a
  # collection error for tests that never read it.
  set(_pytest_env
    "G1_MOVEIT_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
    "G1_MOVEIT_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/launch"
    "G1_BRINGUP_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/launch"
    "G1_DESCRIPTION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_description/config"
    "G1_CONTROLLERS_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_controllers/config"
  )

  # Reads three packages' config at once: MoveIt's arm, the controller's arm and the bridge's
  # speed clamp all have to agree, and no single package's tests can see the disagreement.
  ament_add_pytest_test(test_moveit_config_drift test/test_moveit_config_drift.py
    ENV ${_pytest_env})

  # bringup's moveit branch names this package's share, so its threading assertions cannot live
  # in g1_navigation alongside the navigation ones.
  ament_add_pytest_test(test_launch_threading test/test_launch_threading.py ENV ${_pytest_env})

  # move_group plans against the expanded xacro, not a checked-in URDF, so the model test has
  # to expand it too, or it validates a robot nobody runs.
  set(_expanded_urdf "${CMAKE_CURRENT_BINARY_DIR}/g1_expanded.urdf")
  set(_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/../g1_description")
  add_custom_command(
    OUTPUT ${_expanded_urdf}
    COMMAND xacro ${_description_dir}/urdf/g1_lowcmd.urdf.xacro -o ${_expanded_urdf}
    DEPENDS ${_description_dir}/urdf/g1_lowcmd.urdf.xacro
            ${_description_dir}/urdf/g1_common.xacro
            ${_description_dir}/urdf/g1_29dof_with_hand_rev_1_0.urdf
            ${_description_dir}/config/lowcmd_params.yaml
    COMMENT "Expanding g1_lowcmd.urdf.xacro for test_robot_model"
    VERBATIM
  )
  add_custom_target(g1_moveit_config_expanded_urdf ALL DEPENDS ${_expanded_urdf})

  ament_add_gmock(test_robot_model test/test_robot_model.cpp)
  if(TARGET test_robot_model)
    target_compile_features(test_robot_model PUBLIC cxx_std_20)
    add_dependencies(test_robot_model g1_moveit_config_expanded_urdf)
    target_compile_definitions(test_robot_model PRIVATE
      G1_TEST_URDF_PATH="${_expanded_urdf}"
      G1_TEST_SRDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/g1.srdf")
    target_link_libraries(test_robot_model
      moveit_core::moveit_robot_model
      moveit_core::moveit_robot_state
      moveit_core::moveit_planning_scene
      urdf::urdf
      srdfdom::srdfdom)
  endif()

  # Simulator suites. Same RESOURCE_LOCK name as g1_bringup's: two suites on one DDS graph and
  # one simulator is what makes them flaky.
  #
  # The whole path, with the balance policy holding the robot up rather than a pinned pelvis.
  # Long budget: the policy has to settle before the arms are touched.
  add_launch_test(test/test_moveit_lowcmd.launch.py TARGET test_moveit_lowcmd TIMEOUT 500)
  set_tests_properties(test_moveit_lowcmd PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 95)

  # Runs the sensor track on the perception scene: the octomap has to fill from the LiDAR
  # before anything can be asserted. SIM_SETTLE_S inside that file cannot be raised to cover
  # the fill; launch_testing's own startup timeout fires first.
  add_launch_test(test/test_octomap_blocks_a_plan.launch.py
    TARGET test_octomap_blocks_a_plan TIMEOUT 500)
  set_tests_properties(test_octomap_blocks_a_plan PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator COST 95)

  g1_add_lint_tests()
endif()

ament_package()
