cmake_minimum_required(VERSION 3.8)
project(g1_controllers)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

# The shipped onnxruntimeConfig.cmake points at lib64/ and include/onnxruntime/, neither of which
# exists in this tarball's layout, so locate the pieces directly instead of find_package.
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /opt/onnxruntime/include REQUIRED)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS /opt/onnxruntime/lib REQUIRED)

add_library(g1_agile_policy SHARED src/agile_policy.cpp)
target_include_directories(g1_agile_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(g1_agile_policy SYSTEM PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(g1_agile_policy PUBLIC ${ONNXRUNTIME_LIBRARY})
target_compile_features(g1_agile_policy PUBLIC cxx_std_20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_agile_policy PRIVATE -Wall -Wextra)
endif()

add_library(g1_freeze_controller SHARED src/g1_freeze_controller.cpp)
target_include_directories(g1_freeze_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_freeze_controller PUBLIC cxx_std_20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_freeze_controller PRIVATE -Wall -Wextra)
endif()
ament_target_dependencies(g1_freeze_controller PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(g1_agile_controller SHARED src/g1_agile_controller.cpp)
target_include_directories(g1_agile_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_agile_controller PUBLIC cxx_std_20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_agile_controller PRIVATE -Wall -Wextra)
endif()
target_link_libraries(g1_agile_controller PUBLIC g1_agile_policy)
ament_target_dependencies(g1_agile_controller PUBLIC
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

add_library(g1_safety_controller SHARED src/g1_safety_controller.cpp)
target_include_directories(g1_safety_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_safety_controller PUBLIC cxx_std_20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(g1_safety_controller PRIVATE -Wall -Wextra)
endif()
ament_target_dependencies(g1_safety_controller PUBLIC
  controller_interface
  controller_manager_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(controller_interface g1_controllers.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS g1_agile_policy g1_freeze_controller g1_agile_controller g1_safety_controller
  EXPORT export_g1_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY policy DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # Discovery through pluginlib's own loader is the proof controller_manager can spawn it;
  # a clean compile does not check the plugin XML or its export.
  ament_add_gmock(test_freeze_pluginlib test/test_freeze_pluginlib.cpp)
  target_link_libraries(test_freeze_pluginlib
    g1_freeze_controller g1_agile_controller g1_safety_controller)
  ament_target_dependencies(test_freeze_pluginlib pluginlib controller_interface)

  # The blend and slew are the only safety-critical arithmetic here, so they are tested directly
  # rather than through a controller_manager harness.
  ament_add_gmock(test_safety_blend test/test_safety_blend.cpp)
  target_link_libraries(test_safety_blend g1_safety_controller)

  # Loads the installed policy, so it pins the ONNX contract and the joint tables together.
  find_package(ament_index_cpp REQUIRED)
  ament_add_gmock(test_agile_policy test/test_agile_policy.cpp)
  target_link_libraries(test_agile_policy g1_agile_policy)
  ament_target_dependencies(test_agile_policy ament_index_cpp)

  # See g1_description/CMakeLists.txt for why this is a plain CTest rather than uncrustify.
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_controllers
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_controllers test")
  endif()

  # Minutes rather than milliseconds, so it carries a label: `--ctest-args -LE clang_tidy` skips
  # it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy run-clang-tidy-18 run-clang-tidy-14)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_controllers
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_controllers PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_controllers test")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  g1_agile_policy
  g1_freeze_controller
  g1_agile_controller
  g1_safety_controller
)
ament_export_targets(export_g1_controllers)
ament_export_dependencies(
  controller_interface
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

ament_package()
