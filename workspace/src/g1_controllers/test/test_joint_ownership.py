"""Every one of the 29 body motors has exactly one owner, in every state the stack can be in.

The invariant this file exists for: G1LowCmdSystem sends a command for all 29 motors on every
tick, and it leaves any joint no controller claimed unpowered. So a joint missing from these
lists is not a configuration nit, it is a limb that goes limp. Two controllers claiming one
joint is the other half: ros2_control refuses the switch, and whichever controller lost the race
never activates.

Read against the description's own joint list rather than a copy, so adding a motor to the robot
and forgetting to give it an owner fails here.
"""

import os
import re

import pytest
import yaml

CONTROLLERS_CONFIG_DIR = os.environ["G1_CONTROLLERS_CONFIG_DIR"]
DESCRIPTION_URDF_DIR = os.environ["G1_DESCRIPTION_URDF_DIR"]

# Active for the whole session, whatever else is going on.
ALWAYS_ACTIVE = ["locomotion_safety_controller", "waist_freeze_controller"]
# Exactly one of these holds the arms at any moment; the bracket trades them in one switch.
ARM_OWNERS = ["arm_freeze_controller", "arm_trajectory_controller"]


@pytest.fixture(scope="module")
def controllers():
    with open(os.path.join(CONTROLLERS_CONFIG_DIR, "lowcmd_controllers.yaml")) as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope="module")
def body_motors():
    path = os.path.join(DESCRIPTION_URDF_DIR, "g1_lowcmd.urdf.xacro")
    with open(path) as handle:
        return re.findall(r'g1_lowcmd_joint name="([^"]+)"', handle.read())


def claimed(controllers, name):
    return set(controllers[name]["ros__parameters"]["joints"])


def test_the_description_still_has_29_body_motors(body_motors):
    """Anchors every count below; a description change should fail here first, not by symptom."""
    assert len(body_motors) == 29
    assert len(set(body_motors)) == 29


@pytest.mark.parametrize("arm_owner", ARM_OWNERS)
def test_all_29_are_claimed_exactly_once(controllers, body_motors, arm_owner):
    owners = ALWAYS_ACTIVE + [arm_owner]
    seen = {}
    for name in owners:
        for joint in claimed(controllers, name):
            assert joint not in seen, (
                f"{joint} is claimed by both {seen.get(joint)} and {name}; ros2_control will "
                "refuse the switch and one of them will not activate"
            )
            seen[joint] = name

    unowned = sorted(set(body_motors) - set(seen))
    assert not unowned, f"unowned with {arm_owner} active: {unowned} -- these motors go unpowered"

    stray = sorted(set(seen) - set(body_motors))
    assert not stray, f"{stray} are claimed but the component does not export them"


def test_the_arm_pair_is_a_straight_swap(controllers):
    """They must claim identical joints, or trading them in one switch leaves a gap."""
    assert claimed(controllers, "arm_freeze_controller") == claimed(
        controllers, "arm_trajectory_controller"
    )


def test_the_emergency_freeze_takes_over_exactly_what_it_replaces(controllers):
    """A wider emergency target cannot activate at all.

    The safety controller's switch deactivates itself and activates this one. Any joint here
    that some other controller already holds makes those resources unavailable, so the switch
    fails and the legs are left with no controller in the one case that matters most.
    """
    safety = claimed(controllers, "locomotion_safety_controller")
    assert claimed(controllers, "locomotion_freeze_controller") == safety


def test_the_configured_emergency_controller_is_the_one_that_exists(controllers):
    configured = controllers["locomotion_safety_controller"]["ros__parameters"][
        "emergency_controller"
    ]
    assert configured in controllers["controller_manager"]["ros__parameters"], (
        f"the safety controller would switch to {configured!r}, which is not a controller this "
        "config declares"
    )
