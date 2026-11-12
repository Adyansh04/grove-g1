"""control.launch.py hardware:=true refuses an environment that could reach the wrong motors."""

import importlib.util
import os

import pytest

SAFE = {"RMW_IMPLEMENTATION": "rmw_zenoh_cpp", "ROS_DOMAIN_ID": "1"}


@pytest.fixture(scope="module")
def control_launch():
    spec = importlib.util.spec_from_file_location("control_launch", os.environ["G1_CONTROL_LAUNCH"])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Unset is Jazzy's default, rmw_fastrtps_cpp: no CycloneDDS either.
@pytest.mark.parametrize("rmw", ["rmw_zenoh_cpp", "rmw_fastrtps_cpp", None])
def test_a_local_address_off_the_robots_domain_may_start(control_launch, rmw):
    environ = {"ROS_DOMAIN_ID": "1", **({"RMW_IMPLEMENTATION": rmw} if rmw else {})}
    assert control_launch._hardware_problems("127.0.0.1", environ) == []


# ROS_DOMAIN_ID unset is domain 0 too.
@pytest.mark.parametrize("domain", [{"ROS_DOMAIN_ID": "0"}, {}])
def test_ros_on_the_robots_domain_is_refused(control_launch, domain):
    environ = {"RMW_IMPLEMENTATION": "rmw_zenoh_cpp", **domain}
    problems = control_launch._hardware_problems("127.0.0.1", environ)
    assert any("ROS_DOMAIN_ID" in problem for problem in problems)


def test_a_cyclonedds_rmw_is_refused(control_launch):
    environ = {**SAFE, "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
    problems = control_launch._hardware_problems("127.0.0.1", environ)
    assert any("RMW_IMPLEMENTATION" in problem for problem in problems)


# 192.0.2.1 is TEST-NET-1 (RFC 5737), never an address of this machine.
@pytest.mark.parametrize("address", ["", "192.0.2.1"])
def test_a_missing_or_foreign_robot_address_is_refused(control_launch, address):
    problems = control_launch._hardware_problems(address, SAFE)
    assert any("robot_address" in problem for problem in problems)
