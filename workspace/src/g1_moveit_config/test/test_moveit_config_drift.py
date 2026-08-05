"""Numbers that must agree across packages, where neither package's own tests can see the pair.

Same blind spot test_gait_coupling and test_rviz_configs exist for: MoveIt's idea of the arm
lives here, the controller's lives in g1_bringup, and the bridge's speed clamp lives in
g1_description. Nothing but a test that reads all three notices when they drift apart.

No simulator, no ROS graph.
"""

import os
import xml.etree.ElementTree as ET

import pytest
import yaml

MOVEIT_CONFIG_DIR = os.environ["G1_MOVEIT_CONFIG_DIR"]
BRINGUP_CONFIG_DIR = os.environ["G1_BRINGUP_CONFIG_DIR"]
DESCRIPTION_CONFIG_DIR = os.environ["G1_DESCRIPTION_CONFIG_DIR"]

ARM_JOINTS = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in (
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
        "wrist_roll", "wrist_pitch", "wrist_yaw",
    )
]


def _load(directory, name):
    with open(os.path.join(directory, name)) as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope="module")
def moveit_controllers():
    return _load(MOVEIT_CONFIG_DIR, "moveit_controllers.yaml")


@pytest.fixture(scope="module")
def controllers():
    return _load(BRINGUP_CONFIG_DIR, "controllers.yaml")


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
    """It has already been unparseable once.

    XML forbids a double hyphen inside a comment, the SRDF is comment-heavy, and no linter in
    this workspace reads .srdf -- ament_xmllint only looks at .xml. The failure surfaces as
    move_group dying at launch with a column number.
    """
    assert srdf.tag == "robot"


def test_moveit_drives_the_controller_bringup_actually_runs(moveit_controllers, controllers):
    mine = moveit_controllers["moveit_simple_controller_manager"]["arm_trajectory_controller"]
    theirs = controllers["arm_trajectory_controller"]["ros__parameters"]
    assert mine["joints"] == theirs["joints"], (
        "g1_moveit_config and g1_bringup disagree about which joints arm_trajectory_controller "
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


def test_planned_speed_stays_under_the_bridge_clamp(joint_limits):
    """The clamp is a backstop, not a controller in the loop.

    g1_hardware_interface slew-clamps every joint; plan faster than that and the motion is
    silently stretched until the controller aborts on a goal-time tolerance instead.
    """
    arm_sdk = _load(DESCRIPTION_CONFIG_DIR, "arm_sdk_params.yaml")
    clamp = arm_sdk["system"]["max_joint_velocity_rad_s"]
    for joint, limits in joint_limits.items():
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


def test_joint_limits_cover_exactly_the_arm(joint_limits):
    assert sorted(joint_limits) == sorted(ARM_JOINTS)


def test_chain_groups_have_a_solver_and_the_composite_one_does_not(srdf, kinematics):
    """The composite group must NOT appear in kinematics.yaml.

    pick_ik rejects any group that is not a chain, and MoveIt only builds the per-subgroup
    solver map for groups that have no solver of their own. Naming both_arms here suppresses
    that map, which is what breaks dual-arm pose goals. It is the most-copied mistake in
    published dual-arm configs, so it is asserted rather than left to a comment.
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
    """There is no /clock on this track: the simulator links no ROS.

    Same trap g1_navigation/test/test_no_sim_time.py exists for. A node given use_sim_time gets
    a clock that never advances, which surfaces as TF lookups failing somewhere unrelated.
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
