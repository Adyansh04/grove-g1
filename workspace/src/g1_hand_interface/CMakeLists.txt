cmake_minimum_required(VERSION 3.16)
project(g1_hand_interface)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
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

# --- library ---------------------------------------------------------------

add_library(g1_dex3_system SHARED src/g1_dex3_system.cpp)
g1_target_defaults(g1_dex3_system)
# SYSTEM so clang-tidy does not fail us on the SDK's own headers.
target_include_directories(g1_dex3_system SYSTEM PUBLIC
  ${UNITREE_SDK2_PREFIX}/include
  ${UNITREE_SDK2_PREFIX}/include/ddscxx
)
target_link_libraries(g1_dex3_system PUBLIC
  unitree_sdk2
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
)
# unitree_sdk2's dds_entity.hpp still builds its queue thread through std::result_of, which
# InitChannel instantiates unconditionally.
target_compile_options(g1_dex3_system PRIVATE -Wno-deprecated-declarations)

pluginlib_export_plugin_description_file(hardware_interface g1_hand_interface.xml)

# --- install ---------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)
install(TARGETS g1_dex3_system EXPORT export_${PROJECT_NAME} DESTINATION lib)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)

  # The wire format, plus the plugin discovery proof: a clean compile says nothing about
  # whether controller_manager can find the class through its XML export.
  ament_add_gmock(test_wire_contract test/test_wire_contract.cpp)
  target_link_libraries(test_wire_contract
    g1_dex3_system pluginlib::pluginlib hardware_interface::hardware_interface)

  g1_add_lint_tests()
endif()

# --- export ----------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools)

ament_package()
