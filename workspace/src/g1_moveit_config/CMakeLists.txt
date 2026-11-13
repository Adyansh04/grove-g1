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

  # One env block for every pytest test: pytest 8+ imports every module in the directory while
  # collecting any single file, so a sibling's missing variable breaks collection.
  set(_pytest_env
    "G1_MOVEIT_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/config"
    "G1_BRINGUP_LAUNCH_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/launch"
    "G1_DESCRIPTION_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_description/config"
    "G1_CONTROLLERS_CONFIG_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../g1_controllers/config"
  )

  # Reads three packages' config at once: MoveIt's arm, the controllers' arm and the hand
  # component's speed clamp have to agree, and no single package's tests can see that.
  ament_add_pytest_test(test_moveit_config_drift test/test_moveit_config_drift.py
    ENV ${_pytest_env})

  # bringup's moveit branch names this package's share, so its threading assertions cannot live
  # in g1_navigation alongside the navigation ones.
  ament_add_pytest_test(test_launch_threading test/test_launch_threading.py ENV ${_pytest_env})

  # move_group plans against the expanded xacros, so the model test expands them too, once per
  # hand model, and runs once per model.
  set(_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/../g1_description")
  set(_model_inputs
    ${_description_dir}/urdf/g1_lowcmd.urdf.xacro
    ${_description_dir}/urdf/g1_common.xacro
    ${_description_dir}/urdf/g1_29dof_with_hand_rev_1_0.urdf
    ${_description_dir}/urdf/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
    ${_description_dir}/config/lowcmd_params.yaml
    ${_description_dir}/config/dex3_params.yaml
    ${_description_dir}/config/inspire_params.yaml
    ${CMAKE_CURRENT_SOURCE_DIR}/config/g1.srdf.xacro
  )
  foreach(_hands dex3 inspire)
    set(_urdf "${CMAKE_CURRENT_BINARY_DIR}/g1_${_hands}.urdf")
    set(_srdf "${CMAKE_CURRENT_BINARY_DIR}/g1_${_hands}.srdf")
    add_custom_command(
      OUTPUT ${_urdf} ${_srdf}
      COMMAND xacro ${_description_dir}/urdf/g1_lowcmd.urdf.xacro hand_model:=${_hands}
              -o ${_urdf}
      COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/config/g1.srdf.xacro hand_model:=${_hands}
              -o ${_srdf}
      DEPENDS ${_model_inputs}
      COMMENT "Expanding the description and its semantics with hand_model:=${_hands}"
      VERBATIM
    )
    add_custom_target(g1_moveit_config_expanded_${_hands} ALL DEPENDS ${_urdf} ${_srdf})

    ament_add_gmock(test_robot_model_${_hands} test/test_robot_model.cpp)
    if(TARGET test_robot_model_${_hands})
      target_compile_features(test_robot_model_${_hands} PUBLIC cxx_std_20)
      add_dependencies(test_robot_model_${_hands} g1_moveit_config_expanded_${_hands})
      target_compile_definitions(test_robot_model_${_hands} PRIVATE
        G1_TEST_URDF_PATH="${_urdf}"
        G1_TEST_SRDF_PATH="${_srdf}"
        G1_TEST_HAND_MODEL="${_hands}")
      target_link_libraries(test_robot_model_${_hands}
        moveit_core::moveit_robot_model
        moveit_core::moveit_robot_state
        moveit_core::moveit_planning_scene
        urdf::urdf
        srdfdom::srdfdom)
    endif()
  endforeach()

  # Simulator suites share g1_bringup's RESOURCE_LOCK: two on one DDS graph and one simulator
  # are flaky.
  #
  # The balance policy holds the robot up, not a pinned pelvis, and has to settle first.
  add_launch_test(test/test_moveit_lowcmd.launch.py TARGET test_moveit_lowcmd TIMEOUT 500)
  set_tests_properties(test_moveit_lowcmd PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator
    # Tuned on the Dex3; the Inspire hands have their own grasp evaluation.
    ENVIRONMENT "G1_HAND_MODEL=dex3" COST 95)

  # The octomap has to fill from the LiDAR before anything is asserted, and the tests wait for
  # it themselves: launch_testing's startup timeout caps SIM_SETTLE_S.
  add_launch_test(test/test_octomap_blocks_a_plan.launch.py
    TARGET test_octomap_blocks_a_plan TIMEOUT 500)
  set_tests_properties(test_octomap_blocks_a_plan PROPERTIES
    RESOURCE_LOCK mujoco_sim LABELS simulator
    # Tuned on the Dex3; the Inspire hands have their own grasp evaluation.
    ENVIRONMENT "G1_HAND_MODEL=dex3" COST 95)

  g1_add_lint_tests()
endif()

ament_package()
