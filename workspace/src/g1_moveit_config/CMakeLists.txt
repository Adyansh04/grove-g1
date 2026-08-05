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
  )

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
