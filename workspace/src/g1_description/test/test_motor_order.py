"""
The DDS motor order exists twice, and copies drift.

`g1_motion_service_sim` needs it to drive the walking policy; `g1_hardware_interface` needs it to
publish the legs and waist to `/joint_states` on the robot, where no controller owns them. The
two packages cannot share a header -- one is simulation only and the other is what ships -- so
each carries its own table.

Index into either table is the LowState motor index. Permute one and a knee angle lands on a
waist joint, which moves every sensor frame hanging off `torso_link` and reads downstream as an
odometry or calibration fault rather than as what it is.
"""

import pathlib
import re
import xml.etree.ElementTree as ET

_SRC = pathlib.Path(__file__).resolve().parents[2]
_SIM_TABLE = _SRC / "g1_motion_service_sim" / "src" / "walk_policy.cpp"
_HW_TABLE = (
    _SRC / "g1_hardware_interface" / "include" / "g1_hardware_interface"
    / "lowstate_joint_states.hpp"
)
_URDF = (
    pathlib.Path(__file__).resolve().parent.parent
    / "urdf"
    / "g1_29dof_with_hand_rev_1_0.urdf"
)

# The lower body: legs 0-11 then waist 12-14. The arms follow in the simulator's table and are
# deliberately absent from the hardware one, since joint_state_broadcaster publishes those.
_NUM_LOWER = 15


def _string_array(path, declaration):
    """Pull the quoted strings out of the initialiser that follows `declaration`."""
    text = path.read_text()
    start = text.index(declaration)
    body = text[start : text.index("};", start)]
    return re.findall(r'"([^"]+)"', body)


def _sim_order():
    return _string_array(_SIM_TABLE, "kDdsMotorOrder")


def _hardware_order():
    return _string_array(_HW_TABLE, "kLowerMotorJointNames")


def test_the_hardware_table_matches_the_simulators_first_fifteen():
    assert _hardware_order() == _sim_order()[:_NUM_LOWER]


def test_the_hardware_table_stops_before_the_arms():
    # Motor 15 is left_shoulder_pitch, which joint_state_broadcaster owns. Publishing it from
    # both would put two sources on one joint at different rates.
    hardware = _hardware_order()
    assert len(hardware) == _NUM_LOWER
    assert _sim_order()[_NUM_LOWER] not in hardware


def test_every_name_is_a_joint_the_urdf_actually_has():
    # robot_state_publisher silently ignores a name it does not recognise, so a typo here costs
    # a missing transform and no error anywhere.
    urdf_joints = {j.get("name") for j in ET.parse(_URDF).getroot().findall("joint")}
    missing = [name for name in _hardware_order() if name not in urdf_joints]
    assert not missing, f"not joints in {_URDF.name}: {missing}"
