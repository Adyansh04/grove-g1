/**
 * @file walk_policy.cpp
 * @brief Observation assembly and action decoding for the sim-only walking policy.
 */

#include "g1_bringup/walk_policy.hpp"

#include <algorithm>
#include <cmath>

namespace g1_bringup
{

std::array<double, 3> gravityOrientation(const std::array<double, 4>& quat_wxyz)
{
    const double qw = quat_wxyz[0];
    const double qx = quat_wxyz[1];
    const double qy = quat_wxyz[2];
    const double qz = quat_wxyz[3];

    return { 2.0 * (-qz * qx + qw * qy),
             -2.0 * (qz * qy + qw * qx),
             1.0 - 2.0 * (qw * qw + qz * qz) };
}

double gaitPhase(const WalkPolicyConfig& config, double elapsed_s)
{
    if (config.gait_period_s <= 0.0)
    {
        return 0.0;
    }
    const double phase = std::fmod(elapsed_s, config.gait_period_s) / config.gait_period_s;
    return phase < 0.0 ? phase + 1.0 : phase;
}

std::array<double, kNumPolicyObs> assembleObservation(
    const WalkPolicyConfig& config, const std::array<double, 3>& base_angular_velocity,
    const std::array<double, 4>&                base_quat_wxyz,
    const std::array<double, kNumPolicyJoints>& joint_position,
    const std::array<double, kNumPolicyJoints>& joint_velocity,
    const std::array<double, kNumPolicyJoints>& previous_action,
    const std::array<double, 3>& command_mps, double phase)
{
    std::array<double, kNumPolicyObs> obs{};

    for (std::size_t i = 0; i < 3; ++i)
    {
        obs[i] = base_angular_velocity[i] * config.ang_vel_scale;
    }

    const auto gravity = gravityOrientation(base_quat_wxyz);
    for (std::size_t i = 0; i < 3; ++i)
    {
        obs[3 + i] = gravity[i];
    }

    /*
     * The vendor feeds a normalised joystick axis here and multiplies by both
     * cmd_scale and max_cmd. We carry a command in real units instead, so the
     * equivalent is velocity * cmd_scale -- the max_cmd factor is folded into
     * the clamp rather than the scale. Same number reaches the policy either
     * way; this form just keeps the command in m/s everywhere upstream.
     */
    for (std::size_t i = 0; i < 3; ++i)
    {
        const double limit   = config.max_cmd[i];
        const double clamped = std::clamp(command_mps[i], -limit, limit);
        obs[6 + i]           = clamped * config.cmd_scale[i];
    }

    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        obs[9 + i] = (joint_position[i] - config.default_angles[i]) * config.dof_pos_scale;
        obs[9 + kNumPolicyJoints + i]       = joint_velocity[i] * config.dof_vel_scale;
        obs[9 + (2 * kNumPolicyJoints) + i] = previous_action[i];
    }

    const double phase_rad              = 2.0 * M_PI * phase;
    obs[9 + (3 * kNumPolicyJoints)]     = std::sin(phase_rad);
    obs[9 + (3 * kNumPolicyJoints) + 1] = std::cos(phase_rad);

    return obs;
}

std::array<double, kNumPolicyJoints> actionToJointTargets(
    const WalkPolicyConfig& config, const std::array<double, kNumPolicyJoints>& action)
{
    std::array<double, kNumPolicyJoints> targets{};
    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        targets[i] = config.default_angles[i] + (action[i] * config.action_scale);
    }
    return targets;
}

}  // namespace g1_bringup
