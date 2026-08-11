cmake_minimum_required(VERSION 3.8)
project(g1_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

# The URDF names its meshes package://g1_description/meshes/..., but the STLs are not ours to
# vendor -- they ship with unitree_mujoco, which the container image installs. Without them RViz
# resolves "meshes/x.STL" as a URL host and emits ~174 "Could not resolve host: meshes" lines per
# session while drawing no robot at all.
#
# Copied at configure time rather than committed: 32 MB of binary STL does not belong in this
# repo when the image already carries it. If the directory is absent (a machine without the sim
# installed), nothing is installed and RViz degrades to exactly the no-robot behaviour it had
# before -- the build does not fail over a visualisation asset.
set(G1_VENDOR_MESHES "/opt/unitree_robotics/unitree_mujoco/unitree_robots/g1/meshes"
    CACHE PATH "Directory holding the vendored G1 visual meshes")
set(_g1_vendor_meshes "${G1_VENDOR_MESHES}")
if(EXISTS "${_g1_vendor_meshes}")
  file(GLOB _g1_stls CONFIGURE_DEPENDS "${_g1_vendor_meshes}/*.STL")
  file(COPY ${_g1_stls} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/meshes")
  # Three links are named *_rev_1_0 in this revision of the description but ship unsuffixed in
  # the vendored set. Same links, adjacent revision; for a visualisation mesh that is fine.
  foreach(_pair "torso_link" "waist_roll_link" "waist_yaw_link")
    # Only alias when the suffixed mesh is genuinely absent. If a later vendored set ships
    # a real *_rev_1_0.STL, copying the unsuffixed one over it would silently substitute the
    # wrong geometry.
    set(_src "${CMAKE_CURRENT_BINARY_DIR}/meshes/${_pair}.STL")
    set(_alias "${CMAKE_CURRENT_BINARY_DIR}/meshes/${_pair}_rev_1_0.STL")
    if(EXISTS "${_src}" AND NOT EXISTS "${_alias}")
      configure_file("${_src}" "${_alias}" COPYONLY)
    endif()
  endforeach()
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/meshes" DESTINATION share/${PROJECT_NAME})
else()
  message(WARNING
    "${_g1_vendor_meshes} not found; installing no meshes. RViz will show no robot model.")
endif()

if(BUILD_TESTING)
  # The description as XML. ament_cmake_xmllint only covers package.xml, and both xacro and
  # MuJoCo accept things xmllint rejects -- notably a "--" inside a comment, which is illegal
  # XML but loads fine, so it survives until something else reads the file. That exact mistake
  # has been made three times here now.
  find_program(XMLLINT_EXECUTABLE xmllint)
  if(XMLLINT_EXECUTABLE)
    file(GLOB _description_xml
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.xacro"
      "${CMAKE_CURRENT_SOURCE_DIR}/urdf/*.urdf")
    add_test(
      NAME xmllint_description_g1_description
      COMMAND ${XMLLINT_EXECUTABLE} --noout ${_description_xml}
    )
  endif()

  # Linters that don't conflict with ruff.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_arm_sdk_xacro test/test_arm_sdk_xacro.py
    ENV "G1_DESCRIPTION_XACRO=${CMAKE_CURRENT_SOURCE_DIR}/urdf/g1_arm_sdk.urdf.xacro"
  )
  # The sensor mounts are duplicated in the simulator, which links no ROS and so cannot read TF.
  ament_add_pytest_test(test_sensor_mounts test/test_sensor_mounts.py)
  ament_add_pytest_test(test_motor_order test/test_motor_order.py)

  # Use workspace-root ruff.toml if available.
  find_program(RUFF_EXECUTABLE ruff)
  set(_ruff_config "${CMAKE_CURRENT_SOURCE_DIR}/../../ruff.toml")
  if(RUFF_EXECUTABLE AND EXISTS "${_ruff_config}")
    add_test(
      NAME ruff_check_g1_description
      COMMAND ${RUFF_EXECUTABLE} check --config ${_ruff_config} ${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  else()
    message(WARNING "ruff and/or workspace ruff.toml not found; skipping ruff_check_g1_description test")
  endif()
endif()

ament_package()
