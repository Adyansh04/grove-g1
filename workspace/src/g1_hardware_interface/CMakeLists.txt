cmake_minimum_required(VERSION 3.16)
project(g1_hardware_interface)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# Plain CMake package baked into the dev image, not an ament one, so it needs the prefix.
set(UNITREE_SDK2_PREFIX /opt/unitree_robotics)
find_package(unitree_sdk2 REQUIRED PATHS ${UNITREE_SDK2_PREFIX}/lib/cmake NO_DEFAULT_PATH)

# The SDK must bind the libddsc/libddscxx it ships, not ROS's newer CycloneDDS: same SONAME,
# different ABI, and mixing them corrupts the heap.
list(APPEND CMAKE_BUILD_RPATH ${UNITREE_SDK2_PREFIX}/lib)
list(APPEND CMAKE_INSTALL_RPATH ${UNITREE_SDK2_PREFIX}/lib)

# DT_RPATH, not the default DT_RUNPATH, which loses to LD_LIBRARY_PATH and its ROS lib dir.
string(APPEND CMAKE_SHARED_LINKER_FLAGS " -Wl,--disable-new-dtags")
string(APPEND CMAKE_EXE_LINKER_FLAGS " -Wl,--disable-new-dtags")

# --- libraries -------------------------------------------------------------

# Separately compiled because the bit loop is vendored under an external licence, and keeping
# it its own library keeps that boundary visible.
add_library(motor_crc_hg src/motor_crc_hg.cpp)
g1_target_defaults(motor_crc_hg)

# The rt/lowcmd mode table and per-motor packing, split out because it is the part worth
# asserting without a live component.
add_library(lowcmd_assembly src/lowcmd_assembly.cpp)
g1_target_defaults(lowcmd_assembly)
target_link_libraries(lowcmd_assembly PUBLIC unitree_sdk2 motor_crc_hg)
# SYSTEM so clang-tidy does not fail us on the SDK's own headers.
target_include_directories(lowcmd_assembly SYSTEM PUBLIC
  ${UNITREE_SDK2_PREFIX}/include
  ${UNITREE_SDK2_PREFIX}/include/ddscxx
)

add_library(g1_lowcmd_system SHARED src/g1_lowcmd_system.cpp)
g1_target_defaults(g1_lowcmd_system)
target_link_libraries(g1_lowcmd_system PUBLIC
  lowcmd_assembly
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${diagnostic_msgs_TARGETS}
)
# unitree_sdk2's dds_entity.hpp still builds its queue thread through std::result_of, which
# InitChannel instantiates unconditionally. Scoped to this target so the rest of the package
# keeps the strict gate; drop it when the SDK is bumped past the deprecation.
target_compile_options(g1_lowcmd_system PRIVATE -Wno-deprecated-declarations)

# Makes the plugin discoverable through pluginlib's ament-index lookup, the same path
# controller_manager uses to load it by name from the URDF.
pluginlib_export_plugin_description_file(hardware_interface g1_hardware_interface.xml)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS g1_lowcmd_system motor_crc_hg lowcmd_assembly
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  # Loading the plugin by its registered name through pluginlib's own ClassLoader is the proof
  # controller_manager will find it; a clean compile says nothing about the XML export.
  ament_add_gmock(test_pluginlib_loading test/test_pluginlib_loading.cpp)
  target_link_libraries(test_pluginlib_loading
    g1_lowcmd_system pluginlib::pluginlib hardware_interface::hardware_interface)

  # The mode table rt/lowcmd resolves interface claims through, plus the release ramp's packing.
  ament_add_gmock(test_lowcmd_assembly test/test_lowcmd_assembly.cpp)
  target_link_libraries(test_lowcmd_assembly lowcmd_assembly)

  # Not the bit loop, which is vendored: which bytes it covers, and that the struct's padding
  # is deterministic. A checksum the firmware rejects means every command is ignored.
  ament_add_gmock(test_motor_crc_hg test/test_motor_crc_hg.cpp)
  target_link_libraries(test_motor_crc_hg lowcmd_assembly)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(g1_lowcmd_system)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
