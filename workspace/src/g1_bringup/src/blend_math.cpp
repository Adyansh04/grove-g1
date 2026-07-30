/**
 * @file blend_math.cpp
 * @brief Implements the arm_sdk blend-weight ramp and the sim bridge's full-body /lowcmd assembly.
 */
#include "g1_bringup/blend_math.hpp"

#include <algorithm>

namespace g1_bringup
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
    const std::array<double, kNumArmMotors>&  arm_cmd_q,
    const std::array<double, kNumArmMotors>&  arm_cmd_kp,
    const std::array<double, kNumArmMotors>& arm_cmd_kd, double weight, double leg_kp,
    double leg_kd, double waist_kp, double waist_kd, double arm_hold_kp, double arm_hold_kd)
{
    /*
     * rosidl-generated: zero-initialized, including reserved slots and mode/mode_pr/mode_machine
     * -- deliberately left untouched (see motion_service_sim_node's README).
     */
    unitree_hg::msg::LowCmd cmd;

    for (int i = 0; i < kNumLegMotors; ++i)
    {
        auto& motor = cmd.motor_cmd[static_cast<std::size_t>(i)];
        motor.q     = static_cast<float>(hold_q[static_cast<std::size_t>(i)]);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = static_cast<float>(leg_kp);
        motor.kd    = static_cast<float>(leg_kd);
    }
    for (int i = kNumLegMotors; i < kFirstArmMotor; ++i)
    {
        auto& motor = cmd.motor_cmd[static_cast<std::size_t>(i)];
        motor.q     = static_cast<float>(hold_q[static_cast<std::size_t>(i)]);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = static_cast<float>(waist_kp);
        motor.kd    = static_cast<float>(waist_kd);
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

}  // namespace g1_bringup
