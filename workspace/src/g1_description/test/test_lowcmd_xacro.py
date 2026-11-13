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


def _expand(xacro_path, *args):
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as tmp:
        out_path = tmp.name
    subprocess.run(["xacro", xacro_path, *args, "-o", out_path], check=True)
    return out_path


def _components(*args):
    """Return the expanded description's ros2_control blocks by name, as their hardware params."""
    path = _expand(os.environ["G1_LOWCMD_XACRO"], *args)
    try:
        root = ET.parse(path).getroot()
    finally:
        os.remove(path)
    return {
        c.get("name"): {p.get("name"): p.text for p in c.findall("hardware/param")}
        for c in root.findall("ros2_control")
    }


@pytest.fixture(scope="module")
def expanded_urdf_path():
    # The Dex3 description, which the wire-order tests below are about.
    path = _expand(os.environ["G1_LOWCMD_XACRO"], "hand_model:=dex3")
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
    # kp and kd as command interfaces let a controller pick the joint's control mode per tick;
    # without them the policy cannot send the gains it infers.
    root = ET.parse(expanded_urdf_path).getroot()
    for joint in component(root, "G1LowCmdSystem").findall("joint"):
        commands = [c.get("name") for c in joint.findall("command_interface")]
        assert commands == ["position", "velocity", "effort", "kp", "kd"], joint.get("name")
        params = {p.get("name") for p in joint.findall("param")}
        assert {"position_only_kp", "position_only_kd"} <= params, joint.get("name")


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_hand_is_its_own_component_in_wire_order(expanded_urdf_path, side):
    # Its own component, not joints on the body's. Order matters: HandCmd.motor_cmd is
    # positional, so a reordered list would close the wrong fingers.
    root = ET.parse(expanded_urdf_path).getroot()
    hand = component(root, f"G1Dex3System{side.capitalize()}")

    assert hand.find("hardware/plugin").text == "g1_hand_interface/G1Dex3System"
    params = {p.get("name"): p.text for p in hand.findall("hardware/param")}
    assert params["side"] == side
    assert {"kp", "kd", "command_publish_rate", "max_joint_velocity_rad_s"} <= params.keys()
    # Only the first ChannelFactory::Init in the process applies, so a component that disagreed
    # would sit on a channel that never carries traffic.
    body = component(root, "G1LowCmdSystem")
    for shared in ("domain_id", "network_interface"):
        assert params[shared] == body.find(f"hardware/param[@name='{shared}']").text, shared

    joints = hand.findall("joint")
    assert [j.get("name") for j in joints] == [
        f"{side}_hand_{suffix}_joint"
        for suffix in (
            "thumb_0",
            "thumb_1",
            "thumb_2",
            "middle_0",
            "middle_1",
            "index_0",
            "index_1",
        )
    ]
    for joint in joints:
        assert [c.get("name") for c in joint.findall("command_interface")] == ["position"]
        limits = {p.get("name"): float(p.text) for p in joint.findall("param")}
        assert limits["min"] < limits["max"]


def test_sim_never_releases_an_onboard_controller():
    body = _components("hand_model:=dex3")["G1LowCmdSystem"]
    assert body["release_motion_mode"] == "False"
    assert body["domain_id"] == "1"


def test_hardware_points_the_body_and_both_hands_at_the_robots_domain():
    components = _components("hardware:=true", "hand_model:=dex3")
    assert set(components) == {"G1LowCmdSystem", "G1Dex3SystemLeft", "G1Dex3SystemRight"}
    assert components["G1LowCmdSystem"]["release_motion_mode"] == "True"
    for name, params in components.items():
        assert params["domain_id"] == "0", name


def test_no_hands_leaves_only_the_body_component():
    assert set(_components("hardware:=true", "hands:=false")) == {"G1LowCmdSystem"}


INSPIRE_REGISTER_ORDER = (
    "pinky_proximal_joint",
    "ring_proximal_joint",
    "middle_proximal_joint",
    "index_proximal_joint",
    "thumb_proximal_pitch_joint",
    "thumb_proximal_yaw_joint",
)


def _robot(*args):
    path = _expand(os.environ["G1_LOWCMD_XACRO"], *args)
    try:
        return ET.parse(path).getroot()
    finally:
        os.remove(path)


def test_inspire_hands_replace_the_dex3_components():
    assert set(_components("hand_model:=inspire")) == {
        "G1LowCmdSystem",
        "G1InspireSystemLeft",
        "G1InspireSystemRight",
    }
    assert set(_components("hand_model:=inspire", "hands:=false")) == {"G1LowCmdSystem"}
    # Unless told otherwise, the robot wears the Inspire hands.
    assert "G1InspireSystemLeft" in _components()


@pytest.mark.parametrize("side,prefix", [("left", "L"), ("right", "R")])
def test_each_inspire_hand_is_its_own_component_in_register_order(side, prefix):
    # The hand's registers are positional, so a reordered list would drive the wrong finger.
    root = _robot("hand_model:=inspire")
    hand = component(root, f"G1InspireSystem{side.capitalize()}")
    assert hand.find("hardware/plugin").text == "g1_inspire_hand/G1InspireSystem"
    params = {p.get("name"): p.text for p in hand.findall("hardware/param")}
    assert params["side"] == side
    assert params["protocol"] in ("inspire", "modbus_rtu")

    joints = hand.findall("joint")
    assert [j.get("name") for j in joints] == [f"{prefix}_{s}" for s in INSPIRE_REGISTER_ORDER]
    urdf_joints = {j.get("name"): j for j in root.findall("joint")}
    for joint in joints:
        assert [c.get("name") for c in joint.findall("command_interface")] == ["position"]
        assert [s.get("name") for s in joint.findall("state_interface")] == [
            "position",
            "velocity",
            "effort",
        ]
        values = {p.get("name"): float(p.text) for p in joint.findall("param")}
        assert values["open_rad"] != values["closed_rad"]
        assert values["lever_arm_m"] > 0.0
        # The command range has to fit the joint, or the controller and the hand disagree.
        limit = urdf_joints[joint.get("name")].find("limit")
        low, high = sorted((values["open_rad"], values["closed_rad"]))
        assert float(limit.get("lower")) <= low and high <= float(limit.get("upper"))


def test_each_inspire_hand_has_its_own_line_in_sim_and_on_the_robot():
    for args in ((), ("hardware:=true",)):
        components = _components("hand_model:=inspire", *args)
        ports = {components[f"G1InspireSystem{side}"]["serial_port"] for side in ("Left", "Right")}
        assert len(ports) == 2, args
    sim = _components("hand_model:=inspire")["G1InspireSystemLeft"]["serial_port"]
    robot = _components("hand_model:=inspire", "hardware:=true")["G1InspireSystemLeft"]
    assert sim != robot["serial_port"]


@pytest.mark.parametrize("hand_model", ["dex3", "inspire"])
def test_both_hand_models_name_the_palm_and_grasp_frames_alike(hand_model):
    # MoveIt's arm chains end at the palm and grasp goals target the grasp frame, so both hands
    # must carry them, with the palm where the Dex3's is.
    root = _robot(f"hand_model:={hand_model}")
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    for side, y in (("left", "0.003"), ("right", "-0.003")):
        palm = joints[f"{side}_hand_palm_link"]
        assert palm.find("parent").get("link") == f"{side}_wrist_yaw_link"
        assert [float(v) for v in palm.find("origin").get("xyz").split()] == pytest.approx(
            [0.0415, float(y), 0.0]
        )
        grasp = joints[f"{side}_hand_grasp_frame"]
        assert grasp.find("parent").get("link") == f"{side}_hand_palm_link"


def test_the_hand_models_share_no_hand_links():
    dex3 = {link.get("name") for link in _robot("hand_model:=dex3").findall("link")}
    inspire = {link.get("name") for link in _robot("hand_model:=inspire").findall("link")}
    assert not any(name.startswith(("L_", "R_")) for name in dex3)
    assert not any("_hand_thumb_" in name or "_hand_index_" in name for name in inspire)
