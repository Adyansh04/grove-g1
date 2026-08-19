# Validates the whole-body xacro: the body component, the hands, and their wire order.
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

# SDK motor order, 0-28. The component packs LowCmd.motor_cmd positionally from this.
EXPECTED_BODY_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
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


def _expand(xacro_path):
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        out_path = tmp.name
    subprocess.run(["xacro", xacro_path, "-o", out_path], check=True)
    return out_path


@pytest.fixture(scope="module")
def expanded_urdf_path():
    path = _expand(os.environ["G1_LOWCMD_XACRO"])
    yield path
    os.remove(path)


def component(root, name):
    return next(c for c in root.findall("ros2_control") if c.get("name") == name)


def test_xacro_expands_to_valid_urdf(expanded_urdf_path):
    result = subprocess.run(["check_urdf", expanded_urdf_path], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_component_claims_all_29_body_motors_in_sdk_order(expanded_urdf_path):
    root = ET.parse(expanded_urdf_path).getroot()
    body = component(root, "G1LowCmdSystem")
    assert body.find("hardware/plugin").text == "g1_hardware_interface/G1LowCmdSystem"
    assert [j.get("name") for j in body.findall("joint")] == EXPECTED_BODY_JOINTS


def test_every_body_joint_exports_kp_and_kd(expanded_urdf_path):
    # kp and kd as command interfaces is what lets a controller pick the joint's control mode
    # per tick; without them the policy has no way to send the gains it infers.
    root = ET.parse(expanded_urdf_path).getroot()
    for joint in component(root, "G1LowCmdSystem").findall("joint"):
        commands = [c.get("name") for c in joint.findall("command_interface")]
        assert commands == ["position", "velocity", "effort", "kp", "kd"], joint.get("name")
        params = {p.get("name") for p in joint.findall("param")}
        assert {"position_only_kp", "position_only_kd"} <= params, joint.get("name")


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_hand_is_its_own_component_in_wire_order(expanded_urdf_path, side):
    # The Dex3 is a separate device with its own authority, so it gets its own component
    # rather than extra joints on the body's. Order is load-bearing here and not merely
    # tidy: HandCmd.motor_cmd is positional, and G1Dex3System refuses to init on a mismatch
    # precisely so a reordered list cannot silently close the wrong fingers.
    root = ET.parse(expanded_urdf_path).getroot()
    hand = component(root, f"G1Dex3System{side.capitalize()}")

    assert hand.find("hardware/plugin").text == "g1_hand_interface/G1Dex3System"
    params = {p.get("name"): p.text for p in hand.findall("hardware/param")}
    assert params["side"] == side
    assert {"kp", "kd", "command_publish_rate", "max_joint_velocity_rad_s"} <= params.keys()
    # ChannelFactory::Init is mutex-guarded and later calls are a silent no-op, so whichever of
    # the three components activates first fixes both of these for the whole process and a
    # component that disagreed would sit on a channel that never carries traffic.
    #
    # network_interface is the more dangerous: a non-empty value on the winner makes the SDK
    # build an inline CycloneDDS config and discard CYCLONEDDS_URI for every component.
    body = component(root, "G1LowCmdSystem")
    for shared in ("domain_id", "network_interface"):
        assert params[shared] == body.find(f"hardware/param[@name='{shared}']").text, shared

    joints = hand.findall("joint")
    assert [j.get("name") for j in joints] == [
        f"{side}_hand_{suffix}_joint"
        for suffix in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1",
                       "index_0", "index_1")
    ]
    for joint in joints:
        assert [c.get("name") for c in joint.findall("command_interface")] == ["position"]
        limits = {p.get("name"): float(p.text) for p in joint.findall("param")}
        assert limits["min"] < limits["max"]
