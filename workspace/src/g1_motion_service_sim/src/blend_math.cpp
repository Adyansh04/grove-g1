/**
 * @file blend_math.cpp
 * @brief Implements the arm_sdk blend-weight ramp and the sim bridge's full-body /lowcmd assembly.
 */
#include "g1_motion_service_sim/blend_math.hpp"

#include <algorithm>

namespace g1_motion_service_sim
{

double stepEffectiveWeight(
    double previous_effective_weight, double raw_weight, bool arm_sdk_stale,
    double timeout_ramp_down_s, double dt_s)
{
    const double target   = arm_sdk_stale ? 0.0 : std::clamp(raw_weight, 0.0, 1.0);
    const double max_step = dt_s / timeout_ramp_down_s;

    double next = previous_effective_weight;
    if (target > next)
    {
        next = std::min(target, next + max_step);
    }
    else if (target < next)
    {
        next = std::max(target, next - max_step);
    }
    return std::clamp(next, 0.0, 1.0);
}

unitree_hg::msg::LowCmd assembleSimLowCmd(
    const std::array<double, kNumBodyMotors>& hold_q,
    const std::array<double, kFirstArmMotor>& lower_q,
    const std::array<double, kFirstArmMotor>& lower_kp,
    const std::array<double, kFirstArmMotor>& lower_kd,
    const std::array<double, kNumArmMotors>&  arm_cmd_q,
    const std::array<double, kNumArmMotors>&  arm_cmd_kp,
    const std::array<double, kNumArmMotors>& arm_cmd_kd, double weight, double arm_hold_kp,
    double arm_hold_kd)
{
    // rosidl-generated: zero-initialized, including reserved slots and mode/mode_pr/mode_machine
    // -- deliberately left untouched (see motion_service_sim_node's README).
    unitree_hg::msg::LowCmd cmd;

    // Legs and waist share one loop now: they are one authority block, owned either by the hold
    // pose or by the walking policy, and the caller has already resolved which.
    for (int i = 0; i < kFirstArmMotor; ++i)
    {
        const auto idx   = static_cast<std::size_t>(i);
        auto&      motor = cmd.motor_cmd[idx];
        motor.q          = static_cast<float>(lower_q[idx]);
        motor.dq         = 0.0F;
        motor.tau        = 0.0F;
        motor.kp         = static_cast<float>(lower_kp[idx]);
        motor.kd         = static_cast<float>(lower_kd[idx]);
    }
    for (int i = 0; i < kNumArmMotors; ++i)
    {
        const auto idx         = static_cast<std::size_t>(i);
        const int  motor_index = kFirstArmMotor + i;
        auto&      motor       = cmd.motor_cmd[static_cast<std::size_t>(motor_index)];
        motor.q                = static_cast<float>(
            blend(hold_q[static_cast<std::size_t>(motor_index)], arm_cmd_q[idx], weight));
        motor.dq  = 0.0F;
        motor.tau = 0.0F;
        motor.kp  = static_cast<float>(blend(arm_hold_kp, arm_cmd_kp[idx], weight));
        motor.kd  = static_cast<float>(blend(arm_hold_kd, arm_cmd_kd[idx], weight));
    }
    cmd.motor_cmd[kWeightMotorIndex].q = static_cast<float>(weight);

    return cmd;
}

}  // namespace g1_motion_service_sim
