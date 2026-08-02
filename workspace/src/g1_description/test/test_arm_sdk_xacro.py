# Validates vendored URDF and xacro wrapper against expected arm joints.
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

# Arms only, in order (G1Arm7JointIndex: left 15-21, right 22-28).
EXPECTED_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


@pytest.fixture(scope="module")
def expanded_urdf_path():
    xacro_path = os.environ["G1_DESCRIPTION_XACRO"]
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        out_path = tmp.name
    subprocess.run(["xacro", xacro_path, "-o", out_path], check=True)
    yield out_path
    os.remove(out_path)


def test_xacro_expands_to_valid_urdf(expanded_urdf_path):
    # check_urdf is urdfdom's own parser/validator -- the same one
    # robot_state_publisher uses, so this is the real acceptance test.
    result = subprocess.run(["check_urdf", expanded_urdf_path], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_ros2_control_exports_exactly_the_arm_joints(expanded_urdf_path):
    root = ET.parse(expanded_urdf_path).getroot()
    ros2_control = root.find("ros2_control")
    assert ros2_control is not None
    assert ros2_control.get("name") == "G1ArmSdkSystem"

    joints = ros2_control.findall("joint")
    names = [j.get("name") for j in joints]
    assert names == EXPECTED_ARM_JOINTS, (
        f"arm joint set/order mismatch against the spike list:\n"
        f"  got:      {names}\n  expected: {EXPECTED_ARM_JOINTS}"
    )

    for joint, expected_motor_index in zip(joints, range(15, 29), strict=True):
        cmd_ifaces = [c.get("name") for c in joint.findall("command_interface")]
        state_ifaces = [s.get("name") for s in joint.findall("state_interface")]
        assert cmd_ifaces == ["position"]
        assert state_ifaces == ["position", "velocity", "effort"]

        params = {p.get("name"): p.text for p in joint.findall("param")}
        assert int(params["motor_index"]) == expected_motor_index
        assert "kp" in params and "kd" in params


def test_ros2_control_hardware_plugin_and_system_params(expanded_urdf_path):
    root = ET.parse(expanded_urdf_path).getroot()
    hardware = root.find("ros2_control/hardware")
    assert hardware.find("plugin").text == "g1_hardware_interface/G1ArmSdkSystem"

    params = {p.get("name"): p.text for p in hardware.findall("param")}
    expected_system_params = {
        "command_publish_rate",
        "blend_ramp_up_s",
        "blend_ramp_down_s",
        "emergency_ramp_down_s",
        "max_joint_velocity_rad_s",
        "lowstate_timeout_ms",
    }
    assert expected_system_params <= params.keys()


def test_no_non_arm_joints_leak_into_ros2_control(expanded_urdf_path):
    # Waist/legs/hands must stay off this component's interfaces -- they
    # belong to the onboard controller. Fourteen is the whole arm-only budget.
    root = ET.parse(expanded_urdf_path).getroot()
    joints = root.find("ros2_control").findall("joint")
    assert len(joints) == 14
