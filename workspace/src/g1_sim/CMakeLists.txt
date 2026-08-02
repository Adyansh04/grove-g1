cmake_minimum_required(VERSION 3.8)
project(g1_sim)

find_package(ament_cmake REQUIRED)

# No compiled targets: this package is MJCF, URDF, config, launch and tests.
install(DIRECTORY mjcf urdf config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
