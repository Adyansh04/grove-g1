cmake_minimum_required(VERSION 3.16)
project(g1_description)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/g1_common.cmake)

find_package(ament_cmake REQUIRED)

# --- install ---------------------------------------------------------------

install(DIRECTORY urdf config DESTINATION share/${PROJECT_NAME})

# Copied at configure time rather than committed
set(G1_VENDOR_MESHES "/opt/unitree_robotics/unitree_mujoco/unitree_robots/g1/meshes"
    CACHE PATH "Directory holding the vendored G1 visual meshes")

if(EXISTS "${G1_VENDOR_MESHES}")
  file(GLOB _stls CONFIGURE_DEPENDS "${G1_VENDOR_MESHES}/*.STL")
  file(COPY ${_stls} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/meshes")

  # Three links are named *_rev_1_0 in this revision of the description but ship unsuffixed in
  # the vendored set. Same links, adjacent revision. Aliased only when the suffixed mesh is
  # genuinely absent: a later vendored set shipping a real one must not be overwritten.
  foreach(_link "torso_link" "waist_roll_link" "waist_yaw_link")
    set(_src "${CMAKE_CURRENT_BINARY_DIR}/meshes/${_link}.STL")
    set(_alias "${CMAKE_CURRENT_BINARY_DIR}/meshes/${_link}_rev_1_0.STL")
    if(EXISTS "${_src}" AND NOT EXISTS "${_alias}")
      configure_file("${_src}" "${_alias}" COPYONLY)
    endif()
  endforeach()

  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/meshes" DESTINATION share/${PROJECT_NAME})
else()
  message(WARNING
    "${G1_VENDOR_MESHES} not found; installing no meshes. RViz will show no robot model.")
endif()

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_lowcmd_xacro test/test_lowcmd_xacro.py
    ENV "G1_LOWCMD_XACRO=${CMAKE_CURRENT_SOURCE_DIR}/urdf/g1_lowcmd.urdf.xacro"
  )
  # The sensor mounts are duplicated in the simulator, which links no ROS and cannot read TF.
  ament_add_pytest_test(test_sensor_mounts test/test_sensor_mounts.py)
  ament_add_pytest_test(test_motor_order test/test_motor_order.py)

  # ament_cmake_xmllint only covers package.xml, and both xacro and MuJoCo accept things
  # xmllint rejects -- notably a "--" inside a comment, which is illegal XML but loads fine,
  # so it survives until something else reads the file. Made three times here now.
  find_program(XMLLINT_EXECUTABLE xmllint)
  if(XMLLINT_EXECUTABLE)
    file(GLOB _description_xml
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro"
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf")
    add_test(
      NAME xmllint_description_${PROJECT_NAME}
      COMMAND ${XMLLINT_EXECUTABLE} --noout ${_description_xml}
    )
  endif()

  g1_add_lint_tests()
endif()

ament_package()
