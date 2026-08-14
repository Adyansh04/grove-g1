cmake_minimum_required(VERSION 3.8)
project(g1_manipulation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(g1_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(g1_object_pose_source_node src/g1_object_pose_source_node.cpp)
target_include_directories(g1_object_pose_source_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_object_pose_source_node PUBLIC cxx_std_20)
ament_target_dependencies(g1_object_pose_source_node PUBLIC
  rclcpp rclcpp_lifecycle vision_msgs visualization_msgs geometry_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(g1_object_pose_source src/g1_object_pose_source_main.cpp)
target_link_libraries(g1_object_pose_source PUBLIC g1_object_pose_source_node)

add_library(g1_manipulation_server_node src/g1_manipulation_server_node.cpp)
target_include_directories(g1_manipulation_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_manipulation_server_node PUBLIC cxx_std_20)
ament_target_dependencies(g1_manipulation_server_node PUBLIC
  rclcpp rclcpp_action g1_msgs geometry_msgs vision_msgs
  moveit_ros_planning_interface moveit_msgs shape_msgs tf2 tf2_geometry_msgs)

add_executable(g1_manipulation_server src/g1_manipulation_server_main.cpp)
target_link_libraries(g1_manipulation_server PUBLIC g1_manipulation_server_node)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_object_pose_source_node g1_manipulation_server_node
  EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_object_pose_source g1_manipulation_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_object_pose_source_node test/test_object_pose_source_node.cpp)
  target_link_libraries(test_object_pose_source_node g1_object_pose_source_node)

  ament_add_gmock(test_grasp_geometry test/test_grasp_geometry.cpp)
  target_link_libraries(test_grasp_geometry g1_manipulation_server_node)

  # test/test_pick_place.launch.py is written and runnable BY HAND, but is deliberately not
  # registered here yet. Under ament_add_test it fails at import with "No module named
  # 'g1_msgs'": this workspace installs generated Python bindings under
  # install/<pkg>/local/lib/python3.10/dist-packages, which the sourced environment picks up
  # and the launch test's restricted environment does not. Adding g1_msgs as a test_depend and
  # forcing a reconfigure did not change it.
  #
  # Registering it would put a permanently red test in the suite, which is worse than an
  # unregistered one: it trains people to ignore the result. Run it by hand meanwhile:
  #
  #   source install/setup.bash
  #   python3 -m launch_testing.launch_test src/g1_manipulation/test/test_pick_place.launch.py
  #
  #

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
      NAME clang_format_check_g1_manipulation
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_manipulation test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_manipulation
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_manipulation PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_manipulation test")
  endif()
endif()

ament_package()
