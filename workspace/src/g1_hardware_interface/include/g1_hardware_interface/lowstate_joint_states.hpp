#ifndef G1_HARDWARE_INTERFACE__LOWSTATE_JOINT_STATES_HPP_
#define G1_HARDWARE_INTERFACE__LOWSTATE_JOINT_STATES_HPP_

/**
 * @file lowstate_joint_states.hpp
 * @brief The legs and waist out of LowState, as JointState. Hardware only.
 */

#include <array>
#include <cstddef>
#include <tuple>

#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_hardware_interface
{

/// Motors 0-14: twelve leg joints then the three waist joints. The arms and hands have
/// ros2_control components and reach /joint_states through joint_state_broadcaster; these do
/// not, on either track.
inline constexpr std::size_t kNumLowerMotors = 15;

/**
 * @brief The 15 lower-body joint names in Unitree DDS motor-index order.
 *
 * Index into this array IS the LowState motor index, which is what makes the fill below a
 * straight copy. Order follows unitree_mujoco's own g1_joint_index_dds.md 29DOF table; the
 * simulator keeps its own copy in g1_motion_service_sim's kDdsMotorOrder, and
 * g1_description's test_motor_order asserts the two still agree.
 */
inline constexpr std::array<const char*, kNumLowerMotors> kLowerMotorJointNames = {
    "left_hip_pitch_joint",  "left_hip_roll_joint",     "left_hip_yaw_joint",
    "left_knee_joint",       "left_ankle_pitch_joint",  "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint",    "right_hip_yaw_joint",
    "right_knee_joint",      "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint",       "waist_roll_joint",        "waist_pitch_joint",
};

/// motor_state is a fixed array on this message, not a sequence as it is on HandState, so the
/// slots below cannot go missing at runtime -- only if Unitree reshapes LowState.
static_assert(
    std::tuple_size_v<unitree_hg::msg::LowState::_motor_state_type> >= kNumLowerMotors,
    "LowState no longer carries the lower-body motors this indexes");

/// Sizes and labels @p msg once, so the per-sample fill allocates nothing.
void initLowerBodyJointState(sensor_msgs::msg::JointState& msg);

/// Copies motors 0-14 out of @p state into a message prepared by initLowerBodyJointState().
void fillLowerBodyJointState(
    const unitree_hg::msg::LowState& state, sensor_msgs::msg::JointState& msg);

}  // namespace g1_hardware_interface

#endif  // G1_HARDWARE_INTERFACE__LOWSTATE_JOINT_STATES_HPP_
