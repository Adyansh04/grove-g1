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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(g1_bt_nodes
  src/skill_nodes.cpp
  src/arm_authority.cpp
)
target_include_directories(g1_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_bt_nodes PUBLIC cxx_std_20)
ament_target_dependencies(g1_bt_nodes PUBLIC
  rclcpp rclcpp_action behaviortree_cpp g1_msgs nav2_msgs geometry_msgs
  controller_manager_msgs lifecycle_msgs tf2 tf2_geometry_msgs)

add_executable(g1_bt_executor src/g1_bt_executor_main.cpp)
target_link_libraries(g1_bt_executor PUBLIC g1_bt_nodes)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_bt_nodes EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_bt_executor DESTINATION lib/${PROJECT_NAME})
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
endif()

ament_package()
