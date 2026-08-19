cmake_minimum_required(VERSION 3.16)
project(g1_msgs)

# --- dependencies ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- interfaces ------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApproachObject.action"
  "action/Pick.action"
  "action/Place.action"
  "action/Retreat.action"
  "action/SetArmPosture.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

# --- tests -----------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
