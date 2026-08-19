"""
The DDS motor order is a table of names, and a name that is not a joint fails silently.

`g1_hardware_interface` maps the URDF's joints onto LowCmd/LowState motor indices through one
table, `kG1JointNames`. Index into it is the motor index on the wire, so permuting it lands a
knee angle on a waist joint, which moves every sensor frame hanging off `torso_link` and reads
downstream as an odometry or calibration fault rather than as what it is.

This lives in g1_description rather than beside the table because the URDF is the other half of
the comparison, and the check is only meaningful across the two.
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
    # Position in this table IS the motor index the component packs LowCmd from, so the whole
    # order is load-bearing, not just the leg/waist/arm boundaries. A swap inside one group,
    # hip roll for hip yaw say, keeps the set, the counts and the boundaries intact and still
    # sends every command to the wrong motor.
    assert _motor_order() == EXPECTED_BODY_JOINTS


def test_every_name_is_a_joint_the_urdf_actually_has():
    # A name the URDF does not have is dropped when the component maps joints, so the motor is
    # simply never driven: no error anywhere, just a limb that does not move.
    urdf_joints = {j.get("name") for j in ET.parse(_URDF).getroot().findall("joint")}
    missing = [name for name in _motor_order() if name not in urdf_joints]
    assert not missing, f"not joints in {_URDF.name}: {missing}"
