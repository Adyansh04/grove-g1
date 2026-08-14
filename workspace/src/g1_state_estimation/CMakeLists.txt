cmake_minimum_required(VERSION 3.8)
project(g1_state_estimation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Frame and staleness math, deliberately free of ROS so it tests without a node or DDS.
add_library(odom_math src/odom_math.cpp)
target_include_directories(odom_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(odom_math PUBLIC cxx_std_20)

add_library(g1_odometry_publisher_node src/g1_odometry_publisher_node.cpp)
target_include_directories(g1_odometry_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_odometry_publisher_node PUBLIC cxx_std_20)
target_link_libraries(g1_odometry_publisher_node PUBLIC odom_math)
ament_target_dependencies(g1_odometry_publisher_node PUBLIC
  rclcpp rclcpp_lifecycle lifecycle_msgs nav_msgs geometry_msgs tf2_ros tf2_msgs
  unitree_go unitree_hg)

add_executable(g1_odometry_publisher src/g1_odometry_publisher_main.cpp)
target_link_libraries(g1_odometry_publisher PUBLIC g1_odometry_publisher_node)

# Hardware only: gives the rest of the stack its PointCloud2 while FAST-LIO takes the
# CustomMsg the driver is running in. See the file header. The conversion is a library so it
# can be tested without a Mid360 -- nothing in simulation exercises this direction.
add_library(livox_cloud src/livox_cloud.cpp)
target_include_directories(livox_cloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(livox_cloud PUBLIC cxx_std_20)
ament_target_dependencies(livox_cloud PUBLIC sensor_msgs livox_ros_driver2)

add_executable(g1_livox_pointcloud src/g1_livox_pointcloud_node.cpp)
target_link_libraries(g1_livox_pointcloud PUBLIC livox_cloud)
ament_target_dependencies(g1_livox_pointcloud PUBLIC rclcpp sensor_msgs livox_ros_driver2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(odom_math PRIVATE -Wall -Wextra)
  target_compile_options(livox_cloud PRIVATE -Wall -Wextra)
  target_compile_options(g1_odometry_publisher_node PRIVATE -Wall -Wextra)
  target_compile_options(g1_odometry_publisher PRIVATE -Wall -Wextra)
  target_compile_options(g1_livox_pointcloud PRIVATE -Wall -Wextra)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lio_bench DESTINATION lib/${PROJECT_NAME})
install(TARGETS odom_math livox_cloud g1_odometry_publisher_node
  EXPORT export_${PROJECT_NAME} DESTINATION lib)
install(TARGETS g1_odometry_publisher g1_livox_pointcloud DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_lifecycle nav_msgs geometry_msgs tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(test_odom_math test/test_odom_math.cpp)
  target_link_libraries(test_odom_math odom_math)

  # Hardware-only path: no simulation run reaches it, so this is its only coverage.
  ament_add_gmock(test_livox_cloud test/test_livox_cloud.cpp)
  target_link_libraries(test_livox_cloud livox_cloud)

  # Runs a real node on an isolated ROS_DOMAIN_ID, so it must not share the sim's.
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gmock(test_odometry_publisher_node test/test_odometry_publisher_node.cpp)
  target_link_libraries(test_odometry_publisher_node g1_odometry_publisher_node)
  ament_target_dependencies(test_odometry_publisher_node rosgraph_msgs)

  # The sim FAST-LIO extrinsic is a copy of the URDF mount; this fails when they drift apart.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_extrinsic test/test_sim_extrinsic.py)
  # The full-stack acceptance test (FAST-LIO vs ground truth on a walk) is
  # g1_bringup/test/test_fastlio_odometry.launch.py: it launches sim.launch.py, and from this
  # package that dependency edge would be a cycle.

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
      NAME clang_format_check_g1_state_estimation
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_state_estimation test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy-14 run-clang-tidy)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_state_estimation
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_state_estimation PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_state_estimation test")
  endif()
endif()

ament_package()
