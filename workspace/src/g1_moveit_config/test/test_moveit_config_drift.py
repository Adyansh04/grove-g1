"""Numbers that must agree across packages, where neither package's own tests can see the pair.

MoveIt's idea of the arm lives here, the controllers' in g1_controllers and the hand's speed
clamp in g1_description. No simulator, no ROS graph.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import yaml

MOVEIT_CONFIG_DIR = os.environ["G1_MOVEIT_CONFIG_DIR"]
DESCRIPTION_CONFIG_DIR = os.environ["G1_DESCRIPTION_CONFIG_DIR"]
CONTROLLERS_CONFIG_DIR = os.environ["G1_CONTROLLERS_CONFIG_DIR"]

CONTROLLER_FILES = {
    "lowcmd": (CONTROLLERS_CONFIG_DIR, "lowcmd_controllers.yaml"),
}

ARM_JOINTS = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in (
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
        "wrist_roll", "wrist_pitch", "wrist_yaw",
    )
]

# Dex3 wire order. HandCmd.motor_cmd is a positional array, so this order is the contract and
# not a convention: the SRDF group, the controller and the hardware component must all agree.
HAND_JOINTS = {
    side: [
        f"{side}_hand_{joint}_joint"
        for joint in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1",
                      "index_0", "index_1")
    ]
    for side in ("left", "right")
}


def _load(directory, name):
    with open(os.path.join(directory, name)) as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope="module")
def moveit_controllers():
    return _load(MOVEIT_CONFIG_DIR, "moveit_controllers.yaml")


@pytest.fixture(scope="module", params=sorted(CONTROLLER_FILES), ids=sorted(CONTROLLER_FILES))
def controllers(request):
    return _load(*CONTROLLER_FILES[request.param])


@pytest.fixture(scope="module")
def joint_limits():
    return _load(MOVEIT_CONFIG_DIR, "joint_limits.yaml")["joint_limits"]


@pytest.fixture(scope="module")
def kinematics():
    return _load(MOVEIT_CONFIG_DIR, "kinematics.yaml")


@pytest.fixture(scope="module")
def srdf():
    return ET.parse(os.path.join(MOVEIT_CONFIG_DIR, "g1.srdf")).getroot()


def test_the_srdf_is_well_formed_xml(srdf):
    """No linter here reads .srdf, and a double hyphen in a comment kills move_group at launch."""
    assert srdf.tag == "robot"


def test_moveit_drives_the_controller_bringup_actually_runs(moveit_controllers, controllers):
    mine = moveit_controllers["moveit_simple_controller_manager"]["arm_trajectory_controller"]
    theirs = controllers["arm_trajectory_controller"]["ros__parameters"]
    assert mine["joints"] == theirs["joints"], (
        "g1_moveit_config and g1_controllers disagree about which joints arm_trajectory_controller "
        "owns; MoveIt would plan for joints the controller will refuse"
    )


def test_moveit_never_manages_controllers(moveit_controllers):
    """Acquiring the arm is ordered and safety-critical; it belongs to activate_arm alone."""
    assert moveit_controllers["moveit_manage_controllers"] is False


def test_partial_joint_goals_stay_enabled(controllers):
    """The single-arm groups are 7 joints; the controller has 14.

    With this false the JTC rejects any goal that does not name all of them, so every left_arm
    or right_arm plan fails at execution.
    """
    params = controllers["arm_trajectory_controller"]["ros__parameters"]
    assert params["allow_partial_joints_goal"] is True


def test_planned_hand_speed_stays_under_the_clamp(joint_limits):
    """G1Dex3System slew-clamps every finger; plan faster and the motion is silently stretched
    until the controller aborts on its goal-time tolerance.

    The body component has no such clamp, so there is nothing to check the arm against.
    """
    clamp = _load(DESCRIPTION_CONFIG_DIR, "dex3_params.yaml")["system"]["max_joint_velocity_rad_s"]
    for joint in HAND_JOINTS["left"] + HAND_JOINTS["right"]:
        limits = joint_limits[joint]
        assert limits["has_velocity_limits"] is True, joint
        assert limits["max_velocity"] < clamp, (
            f"{joint} plans at {limits['max_velocity']} rad/s against a {clamp} rad/s clamp"
        )


def test_every_arm_joint_has_an_acceleration_limit(joint_limits):
    """The URDF declares none, and time parameterization fails without them.

    The plan comes back successful carrying zero timestamps, and only execution notices.
    """
    for joint, limits in joint_limits.items():
        assert limits["has_acceleration_limits"] is True, joint
        assert limits["max_acceleration"] > 0.0, joint


def test_joint_limits_cover_exactly_what_we_command(joint_limits):
    """Everything MoveIt commands and nothing else: the arms plus both hands.

    A joint missing here is timed against no limit at all; a leg or waist joint here would claim
    a limit on something the balance policy or the waist freeze owns.
    """
    expected = ARM_JOINTS + HAND_JOINTS["left"] + HAND_JOINTS["right"]
    assert sorted(joint_limits) == sorted(expected)


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_hand_agrees_across_srdf_controller_and_moveit(
    side, srdf, controllers, moveit_controllers
):
    """Three files have to name the same seven joints in the same order.

    The JTC remaps by name, but G1Dex3System writes HandCmd.motor_cmd positionally. MoveIt's
    RobotModel sorts a joint-list group, so only the files can pin this order.
    """
    group = next(g for g in srdf.findall("group") if g.get("name") == f"{side}_hand")
    controller = f"{side}_hand_controller"

    assert [j.get("name") for j in group.findall("joint")] == HAND_JOINTS[side]
    assert controllers[controller]["ros__parameters"]["joints"] == HAND_JOINTS[side]
    assert (
        moveit_controllers["moveit_simple_controller_manager"][controller]["joints"]
        == HAND_JOINTS[side]
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_hand_is_driven_as_a_trajectory_not_a_gripper_command(side, moveit_controllers):
    """GripperCommand carries one scalar and expects a one-joint controller.

    A Dex3 has seven independent finger joints, so it is driven as a planning group with named
    postures. Switching this to GripperCommand would leave six of them uncommanded.
    """
    entry = moveit_controllers["moveit_simple_controller_manager"][f"{side}_hand_controller"]
    assert entry["type"] == "FollowJointTrajectory"
    assert entry["default"] is False


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_hand_has_an_open_and_a_closed_posture(side, srdf):
    """The postures a pick and place needs, each naming all seven joints.

    A pick descends in pinch_ready, because at `open` the thumb reaches the table first.
    """
    states = {
        s.get("name"): s
        for s in srdf.findall("group_state")
        if s.get("group") == f"{side}_hand"
    }
    assert set(states) == {"open", "closed", "pinch_ready"}
    for name, state in states.items():
        named = {j.get("name") for j in state.findall("joint")}
        assert named == set(HAND_JOINTS[side]), f"{side} {name} does not cover the hand"


def test_the_home_state_lists_the_arm_joints_in_motor_order(srdf):
    """`home` lists the arm joints in the controllers' motor order."""
    home = next(
        s for s in srdf.findall("group_state")
        if s.get("name") == "home" and s.get("group") == "both_arms"
    )
    assert [j.get("name") for j in home.findall("joint")] == ARM_JOINTS


def test_chain_groups_have_a_solver_and_the_composite_one_does_not(srdf, kinematics):
    """The composite group must not appear in kinematics.yaml.

    pick_ik rejects any group that is not a chain, and MoveIt builds the per-subgroup solver map
    only for groups without a solver of their own, so an entry breaks dual-arm pose goals.
    """
    for group in srdf.findall("group"):
        name = group.get("name")
        if group.find("chain") is not None:
            assert name in kinematics, f"chain group {name} has no kinematics solver"
        if group.find("group") is not None:
            assert name not in kinematics, (
                f"composite group {name} has a kinematics.yaml entry; that suppresses the "
                "subgroup solver map and breaks its IK"
            )


def test_the_dual_arm_group_spans_both_arms(srdf):
    composite = [g for g in srdf.findall("group") if g.find("group") is not None]
    assert len(composite) == 1, "expected exactly one composite group"
    members = {g.get("name") for g in composite[0].findall("group")}
    assert members == {"left_arm", "right_arm"}


def test_no_config_here_claims_simulated_time():
    """There is no /clock: the simulator links no ROS.

    A node given use_sim_time gets a clock that never advances, which surfaces as TF lookups
    failing somewhere unrelated.
    """
    offenders = []
    for name in os.listdir(MOVEIT_CONFIG_DIR):
        if not name.endswith((".yaml", ".rviz")):
            continue
        with open(os.path.join(MOVEIT_CONFIG_DIR, name)) as handle:
            for number, line in enumerate(handle, start=1):
                if "use_sim_time" in line and "true" in line.lower():
                    offenders.append(f"{name}:{number}")
    assert not offenders, f"use_sim_time enabled in {offenders}"


# --- sensors_3d.yaml -------------------------------------------------------------------
#
# The octomap updater fails quietly: a missing key skips the sensor with only an error log, and
# the stack comes up healthy without ever building a map.

POINTCLOUD_PLUGIN = "occupancy_map_monitor/PointCloudOctomapUpdater"
DEPTH_IMAGE_PLUGIN = "occupancy_map_monitor/DepthImageOctomapUpdater"

# Every one is mandatory: setParams() returns false without it and the sensor is dropped.
REQUIRED_POINTCLOUD_KEYS = {
    "sensor_plugin", "point_cloud_topic", "max_range", "point_subsample",
    "padding_offset", "padding_scale", "max_update_rate", "filtered_cloud_topic",
}
# Written as YAML floats. An int here throws InvalidParameterTypeException out of the
# OccupancyMapMonitor constructor, which reads as a launch crash with no mention of this file.
MUST_BE_FLOAT = ("max_range", "padding_offset", "padding_scale", "max_update_rate")


@pytest.fixture(scope="module")
def sensors_3d():
    return _load(MOVEIT_CONFIG_DIR, "sensors_3d.yaml")


def test_the_octomap_resolution_is_set_at_the_top_level(sensors_3d):
    """move_group reads it as a node parameter, via to_dict()'s flat merge. Unset, it falls back
    to 0.1 m with only a warning."""
    assert isinstance(sensors_3d.get("octomap_resolution"), float)
    assert 0.0 < sensors_3d["octomap_resolution"] < 0.5


def test_octomap_frame_is_not_set(sensors_3d):
    """It would be inert: move_group builds the monitor in the planning frame and never reads
    octomap_frame, so setting it would only look like it moved the map."""
    assert "octomap_frame" not in sensors_3d


def test_sensors_is_a_flat_list_of_names(sensors_3d):
    """Not a list of dicts, which ROS 2 parameters cannot represent."""
    names = sensors_3d.get("sensors")
    assert isinstance(names, list) and names
    for name in names:
        assert isinstance(name, str) and name, f"{name!r} is not a sensor name"
        assert isinstance(sensors_3d.get(name), dict), f"no top-level block for sensor {name!r}"


def test_every_sensor_block_is_fully_specified(sensors_3d):
    for name in sensors_3d["sensors"]:
        block = sensors_3d[name]
        plugin = block.get("sensor_plugin")
        assert plugin in (POINTCLOUD_PLUGIN, DEPTH_IMAGE_PLUGIN), f"{name}: {plugin!r}"
        assert plugin != DEPTH_IMAGE_PLUGIN, (
            f"{name} uses the depth-image updater. On 2.5.9 its initialize() runs before "
            "setParams(), so the clipping planes, shadow threshold and padding never reach the "
            "mesh filter -- and padding is the self-filter."
        )
        missing = REQUIRED_POINTCLOUD_KEYS - set(block)
        assert not missing, f"{name} is missing {sorted(missing)}; the sensor is skipped silently"
        for key in MUST_BE_FLOAT:
            assert isinstance(block[key], float), (
                f"{name}.{key} is {type(block[key]).__name__}, not float -- write it with a "
                "decimal point or the monitor constructor throws"
            )
        assert block["point_cloud_topic"].startswith("/"), f"{name}: topic must be absolute"


def test_the_self_filter_padding_is_not_disabled(sensors_3d):
    """Padding is the self-filter margin: without it the robot's own arms get mapped as
    obstacles as soon as a link's TF is a little late."""
    for name in sensors_3d["sensors"]:
        assert sensors_3d[name]["padding_scale"] >= 1.0, f"{name} shrinks the robot's own shapes"
        assert sensors_3d[name]["padding_offset"] >= 0.0
