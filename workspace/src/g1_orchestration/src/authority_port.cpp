/**
 * @file authority_port.cpp
 * @brief The controller_manager implementation of AuthorityPort.
 */

#include "g1_orchestration/authority_port.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <rclcpp/wait_for_message.hpp>
#include <utility>

#include "g1_orchestration/arm_authority.hpp"
#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

ClosedHands closedHands(
    const sensor_msgs::msg::JointState& joints, const std::vector<std::string>& finger_joints,
    double closed_rad)
{
    ClosedHands closed;
    for (const std::string& name : finger_joints)
    {
        const auto at = std::ranges::find(joints.name, name);
        const auto i  = static_cast<std::size_t>(at - joints.name.begin());
        if (at == joints.name.end() || i >= joints.position.size() ||
            std::abs(joints.position[i]) > closed_rad)
        {
            (name.starts_with("left") ? closed.left : closed.right) = true;
        }
    }
    return closed;
}

ControllerManagerAuthority::ControllerManagerAuthority(
    rclcpp::Logger logger, double timeout_s, Fingers fingers)
  : logger_(std::move(logger))
  , timeout_s_(timeout_s)
  , fingers_(std::move(fingers))
{}

Acquired ControllerManagerAuthority::acquire() { return acquireArm(logger_, timeout_s_); }

void ControllerManagerAuthority::release()
{
    // Called on the way out of a mission, so an escaping exception would skip what follows.
    try
    {
        releaseArm(logger_, timeout_s_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "the arms may still be acquired: release threw: %s", e.what());
    }
}

ClosedHands ControllerManagerAuthority::closedHands()
{
    // One message on a node of its own, as the other calls do: the joint states stream at the
    // control rate, too often to keep for a reading taken once per release.
    sensor_msgs::msg::JointState joints;
    try
    {
        if (rclcpp::wait_for_message(
                joints,
                makeClientNode("g1_arm_authority_fingers"),
                "/joint_states",
                std::chrono::duration<double>(fingers_.timeout_s),
                rclcpp::SensorDataQoS()))
        {
            return g1_orchestration::closedHands(joints, fingers_.joints, fingers_.closed_rad);
        }
        RCLCPP_WARN(logger_, "no /joint_states: both hands count as closed");
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "reading the fingers threw: %s; both hands count as closed", e.what());
    }
    return { .left = true, .right = true };
}

}  // namespace g1_orchestration
