cmake_minimum_required(VERSION 3.8)
project(g1_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Its own library rather than a source compiled into the plugin: the bit loop is vendored under
# an external licence, and keeping it separately compiled keeps that boundary visible.
add_library(motor_crc_hg src/motor_crc_hg.cpp)
target_include_directories(motor_crc_hg PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(motor_crc_hg PUBLIC cxx_std_20)

# Plain CMake package baked into the dev image, not an ament one, so it needs the prefix.
set(UNITREE_SDK2_PREFIX /opt/unitree_robotics)
find_package(unitree_sdk2 REQUIRED PATHS ${UNITREE_SDK2_PREFIX}/lib/cmake NO_DEFAULT_PATH)

# The SDK must bind the libddsc/libddscxx it ships, not ROS's newer CycloneDDS: same SONAME,
# different ABI, and mixing them corrupts the heap. Scoped here, not /etc/ld.so.conf.d.
list(APPEND CMAKE_BUILD_RPATH ${UNITREE_SDK2_PREFIX}/lib)
list(APPEND CMAKE_INSTALL_RPATH ${UNITREE_SDK2_PREFIX}/lib)

# DT_RPATH, not the default DT_RUNPATH, which loses to LD_LIBRARY_PATH and its ROS lib dir.
string(APPEND CMAKE_SHARED_LINKER_FLAGS " -Wl,--disable-new-dtags")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--disable-new-dtags")

# The rt/lowcmd mode table and per-motor packing, split out for the same reason
# arm_ramp_engine is: it is the part worth asserting without a live component.
add_library(lowcmd_assembly src/lowcmd_assembly.cpp)
target_include_directories(lowcmd_assembly PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(lowcmd_assembly PUBLIC cxx_std_20)
target_link_libraries(lowcmd_assembly PUBLIC unitree_sdk2 motor_crc_hg)
# SYSTEM so clang-tidy's WarningsAsErrors does not fail us on the SDK's own headers, which still
# use std::result_of.
target_include_directories(lowcmd_assembly SYSTEM PUBLIC
  ${UNITREE_SDK2_PREFIX}/include
  ${UNITREE_SDK2_PREFIX}/include/ddscxx
)

add_library(g1_lowcmd_system SHARED src/g1_lowcmd_system.cpp)
target_include_directories(g1_lowcmd_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(g1_lowcmd_system PUBLIC cxx_std_20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # unitree_sdk2's dds_entity.hpp still builds its queue thread through std::result_of, which
  # InitChannel instantiates unconditionally. Scoped to this one target so the rest of the package
  # keeps the strict gate PR 2 earned back; drop it when the SDK is bumped past the deprecation.
  target_compile_options(g1_lowcmd_system PRIVATE -Wall -Wextra -Wno-deprecated-declarations)
endif()
target_link_libraries(g1_lowcmd_system PUBLIC lowcmd_assembly)
ament_target_dependencies(g1_lowcmd_system PUBLIC
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

# Makes the plugin discoverable via pluginlib's ament-index lookup -- the
# same path controller_manager uses to load it by name from the URDF.
pluginlib_export_plugin_description_file(hardware_interface g1_hardware_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS g1_lowcmd_system motor_crc_hg lowcmd_assembly
  EXPORT export_g1_lowcmd_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # Loading the plugin by its registered name through pluginlib's own
  # ClassLoader is the actual proof controller_manager will find it -- a
  # clean compile doesn't guarantee the plugin XML/export is wired correctly.
  ament_add_gmock(test_pluginlib_loading test/test_pluginlib_loading.cpp)
  target_link_libraries(test_pluginlib_loading g1_lowcmd_system)
  ament_target_dependencies(test_pluginlib_loading pluginlib hardware_interface)

  # The mode table rt/lowcmd resolves interface claims through, plus the release ramp's packing.
  ament_add_gmock(test_lowcmd_assembly test/test_lowcmd_assembly.cpp)
  target_link_libraries(test_lowcmd_assembly lowcmd_assembly)

  # Not the bit loop, which is vendored: which bytes it covers, and that the struct's padding is
  # deterministic. A checksum the firmware rejects means every command is ignored.
  ament_add_gmock(test_motor_crc_hg test/test_motor_crc_hg.cpp)
  target_link_libraries(test_motor_crc_hg lowcmd_assembly)

  # workspace/ruff.toml's sibling root config; see g1_description/CMakeLists.txt
  # for why this is a plain CTest instead of ament_lint_common's uncrustify.
  find_program(CLANG_FORMAT_EXECUTABLE clang-format)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  if(CLANG_FORMAT_EXECUTABLE AND EXISTS "${_clang_format_config}")
    file(GLOB_RECURSE _format_sources
      "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp"
    )
    add_test(
      NAME clang_format_check_g1_hardware_interface
      COMMAND ${CLANG_FORMAT_EXECUTABLE} --style=file:${_clang_format_config} --dry-run --Werror
        ${_format_sources}
    )
  else()
    message(WARNING
      "clang-format and/or workspace .clang-format not found; skipping clang_format_check_g1_hardware_interface test")
  endif()

  # clang-tidy over this package's own translation units, gated by WarningsAsErrors in the
  # workspace .clang-tidy. Minutes rather than the format check's milliseconds, so it carries a
  # label: `--ctest-args -LE clang_tidy` skips it for a fast local loop. CI runs it.
  find_program(RUN_CLANG_TIDY_EXECUTABLE NAMES run-clang-tidy run-clang-tidy-18 run-clang-tidy-14)
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  if(RUN_CLANG_TIDY_EXECUTABLE AND EXISTS "${_clang_tidy_config}"
      AND CMAKE_EXPORT_COMPILE_COMMANDS)
    add_test(
      NAME clang_tidy_check_g1_hardware_interface
      COMMAND ${RUN_CLANG_TIDY_EXECUTABLE} -p ${CMAKE_BINARY_DIR} -quiet -j 4
    )
    set_tests_properties(clang_tidy_check_g1_hardware_interface PROPERTIES LABELS clang_tidy TIMEOUT 900)
  else()
    message(WARNING
      "run-clang-tidy or .clang-tidy not found, or compile commands are off; skipping clang_tidy_check_g1_hardware_interface test")
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(g1_lowcmd_system)
ament_export_targets(export_g1_lowcmd_system)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
