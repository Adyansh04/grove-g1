"""
The DDS motor order is a table of names, and a name that is not a joint fails silently.

`g1_hardware_interface`'s `kG1JointNames` maps URDF joints onto LowCmd/LowState motor indices:
position in it is the motor index on the wire, and a permutation reads downstream as an odometry
or calibration fault. It is checked here because the URDF is the other half of the comparison.
"""

import pathlib
import re
import xml.etree.ElementTree as ET

from test_lowcmd_xacro import EXPECTED_BODY_JOINTS

_SRC = pathlib.Path(__file__).resolve().parents[2]
_MOTOR_TABLE = _SRC / "g1_hardware_interface" / "src" / "g1_lowcmd_system.cpp"
_URDF = (
    pathlib.Path(__file__).resolve().parent.parent
    / "urdf"
    / "g1_29dof_with_hand_rev_1_0.urdf"
)

# 12 legs, 3 waist, 14 arms. The hands are a separate device on their own topics.
_NUM_BODY_MOTORS = 29


def _motor_order():
    """Pull the quoted names out of the kG1JointNames initialiser."""
    text = _MOTOR_TABLE.read_text()
    start = text.index("kG1JointNames")
    body = text[start : text.index("};", start)]
    return re.findall(r'"([^"]+)"', body)


def test_the_table_covers_every_body_motor_exactly_once():
    order = _motor_order()
    assert len(order) == _NUM_BODY_MOTORS
    assert len(set(order)) == _NUM_BODY_MOTORS, "a name appears twice, so one motor is unreachable"


def test_the_table_is_in_sdk_motor_index_order():
    # The whole order, not just the group boundaries: swapping hip roll for hip yaw keeps the
    # set and the counts and still sends every command to the wrong motor.
    assert _motor_order() == EXPECTED_BODY_JOINTS


def test_every_name_is_a_joint_the_urdf_actually_has():
    # A name the URDF lacks is dropped when the component maps joints, and that motor is never
    # driven, with no error anywhere.
    urdf_joints = {j.get("name") for j in ET.parse(_URDF).getroot().findall("joint")}
    missing = [name for name in _motor_order() if name not in urdf_joints]
    assert not missing, f"not joints in {_URDF.name}: {missing}"
