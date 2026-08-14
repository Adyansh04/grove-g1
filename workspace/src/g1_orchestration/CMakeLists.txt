cmake_minimum_required(VERSION 3.8)
project(g1_orchestration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# v4. Nav2 depends on the separate behaviortree_cpp_v3 package and keeps it: the two install
# side by side with disjoint files and sonames, and nothing links both. See README.
find_package(behaviortree_cpp REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# One file per leaf, under src/skills, the layout nav2_behavior_tree uses. Listed rather than
# globbed so a file that fails to get added fails the build instead of vanishing silently.
add_library(g1_bt_nodes
  src/arm_authority.cpp
  src/port_types.cpp
  src/registration.cpp
  src/service_leaf.cpp
  src/skills/approach_object.cpp
  src/skills/arm_authority_leaves.cpp
  src/skills/clear_costmaps.cpp
  src/skills/clear_octomap.cpp
  src/skills/navigate_to_pose.cpp
  src/skills/pick.cpp
  src/skills/place.cpp
  src/skills/retreat.cpp
  src/skills/set_arm_posture.cpp
)
target_include_directories(g1_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_bt_nodes PUBLIC cxx_std_20)
ament_target_dependencies(g1_bt_nodes PUBLIC
  rclcpp rclcpp_action behaviortree_cpp g1_msgs nav2_msgs geometry_msgs
  controller_manager_msgs lifecycle_msgs std_srvs tf2 tf2_geometry_msgs)

add_executable(g1_bt_executor src/g1_bt_executor_main.cpp)
target_link_libraries(g1_bt_executor PUBLIC g1_bt_nodes)

# Regenerates the Groot2 palette. Not run by the build: the model is checked in so Groot2 can
# open the project straight from a clone, and test_node_model fails if the two drift.
add_executable(g1_bt_node_model src/g1_bt_node_model_main.cpp)
target_link_libraries(g1_bt_node_model PUBLIC g1_bt_nodes)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_bt_nodes EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_bt_executor g1_bt_node_model DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch trees DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # Every shipped tree parses with every node type registered. Catches a leaf renamed in C++
  # but not in XML, which otherwise surfaces only when the mission is run.
  ament_add_gmock(test_tree_loads test/test_tree_loads.cpp)
  target_link_libraries(test_tree_loads g1_bt_nodes)
  target_compile_definitions(test_tree_loads PRIVATE
    G1_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")

  # ament_cmake_xmllint only globs *.xml, so the Groot2 project file is not covered by it and a
  # malformed comment in it reached a commit. Checked explicitly.
  find_program(XMLLINT_EXECUTABLE xmllint)
  if(XMLLINT_EXECUTABLE)
    add_test(
      NAME xmllint_btproj_g1_orchestration
      COMMAND ${XMLLINT_EXECUTABLE} --noout
        "${CMAKE_CURRENT_SOURCE_DIR}/trees/g1_orchestration.btproj"
    )
  else()
    message(WARNING "xmllint not found; skipping xmllint_btproj_g1_orchestration")
  endif()

  # A stale Groot2 palette invites trees that cannot load.
  ament_add_gmock(test_node_model test/test_node_model.cpp)
  target_link_libraries(test_node_model g1_bt_nodes)
  target_compile_definitions(test_node_model PRIVATE
    G1_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/trees")

  # The acquire sequence against g1_bringup's script, which is the other implementation of it.
  ament_add_gmock(test_authority_drift test/test_authority_drift.cpp)
  target_link_libraries(test_authority_drift g1_bt_nodes)
  target_compile_definitions(test_authority_drift PRIVATE
    G1_ACTIVATE_ARM_SCRIPT="${CMAKE_CURRENT_SOURCE_DIR}/../g1_bringup/scripts/activate_arm")

  # Lint checks use workspace-root configs (see g1_description/CMakeLists.txt).
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_orchestration
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_orchestration test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_orchestration
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_orchestration PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_orchestration test")
  endif()
endif()

ament_package()
