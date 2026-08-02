/**
 * @file walk_policy.cpp
 * @brief Observation assembly, action mapping, and velocity latching for the sim walking policy.
 */
#include "g1_bringup/walk_policy.hpp"

#include <algorithm>
#include <cmath>

namespace g1_bringup
{

const std::array<const char*, kNumBodyMotors> kDdsMotorOrder = {
    "left_hip_pitch_joint",      "left_hip_roll_joint",        "left_hip_yaw_joint",
    "left_knee_joint",           "left_ankle_pitch_joint",     "left_ankle_roll_joint",
    "right_hip_pitch_joint",     "right_hip_roll_joint",       "right_hip_yaw_joint",
    "right_knee_joint",          "right_ankle_pitch_joint",    "right_ankle_roll_joint",
    "waist_yaw_joint",           "waist_roll_joint",           "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",   "left_shoulder_yaw_joint",
    "left_elbow_joint",          "left_wrist_roll_joint",      "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",      "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",  "right_elbow_joint",          "right_wrist_roll_joint",
    "right_wrist_pitch_joint",   "right_wrist_yaw_joint"
};

std::string checkJointOrder(const std::vector<std::string>& joint_names)
{
    if (joint_names.size() != kDdsMotorOrder.size())
    {
        return "expected " + std::to_string(kDdsMotorOrder.size()) + " joint names, got " +
               std::to_string(joint_names.size());
    }
    for (std::size_t i = 0; i < kDdsMotorOrder.size(); ++i)
    {
        if (joint_names[i] != kDdsMotorOrder[i])
        {
            return "joint_names[" + std::to_string(i) + "] is '" + joint_names[i] +
                   "', expected '" + kDdsMotorOrder[i] + "' (Unitree DDS motor order)";
        }
    }
    return {};
}

std::array<double, 3>
rotateWorldToBase(const std::array<double, 4>& quat, const std::array<double, 3>& v)
{
    // Inverse rotation via the standard q^-1 * v * q expansion for a unit quaternion, matching
    // the reference implementation's _quat_apply_inverse exactly.
    const double                w = quat[0];
    const std::array<double, 3> xyz{ quat[1], quat[2], quat[3] };

    const std::array<double, 3> cross_xyz_v{ xyz[1] * v[2] - xyz[2] * v[1],
                                             xyz[2] * v[0] - xyz[0] * v[2],
                                             xyz[0] * v[1] - xyz[1] * v[0] };
    const std::array<double, 3> t{ 2.0 * cross_xyz_v[0],
                                   2.0 * cross_xyz_v[1],
                                   2.0 * cross_xyz_v[2] };
    const std::array<double, 3> cross_xyz_t{ xyz[1] * t[2] - xyz[2] * t[1],
                                             xyz[2] * t[0] - xyz[0] * t[2],
                                             xyz[0] * t[1] - xyz[1] * t[0] };

    return { v[0] - w * t[0] + cross_xyz_t[0],
             v[1] - w * t[1] + cross_xyz_t[1],
             v[2] - w * t[2] + cross_xyz_t[2] };
}

std::array<float, kObsDim> assembleObservation(
    const PolicyInputs& inputs, const PolicyConfig& config,
    const std::array<float, kActionDim>& last_action, const std::array<double, 3>& command)
{
    std::array<float, kObsDim> obs{};

    const auto lin_vel_body = rotateWorldToBase(inputs.base_quat, inputs.base_lin_vel_world);
    const auto gravity_body = rotateWorldToBase(inputs.base_quat, { 0.0, 0.0, -1.0 });

    for (std::size_t i = 0; i < 3; ++i)
    {
        obs[kObsBaseLinVel + i] = static_cast<float>(lin_vel_body[i]);
        obs[kObsBaseAngVel + i] = static_cast<float>(inputs.base_ang_vel_body[i]);
        obs[kObsGravity + i]    = static_cast<float>(gravity_body[i]);
        obs[kObsCommand + i]    = static_cast<float>(command[i]);
    }
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        obs[kObsJointPos + i] =
            static_cast<float>(inputs.joint_pos[i] - config.default_joint_pos[i]);
        obs[kObsJointVel + i]   = static_cast<float>(inputs.joint_vel[i]);
        obs[kObsLastAction + i] = last_action[i];
    }
    return obs;
}

std::array<double, kNumBodyMotors>
actionToJointTargets(const std::array<float, kActionDim>& action, const PolicyConfig& config)
{
    std::array<double, kNumBodyMotors> targets{};
    for (std::size_t i = 0; i < kNumBodyMotors; ++i)
    {
        targets[i] =
            config.default_joint_pos[i] + static_cast<double>(action[i]) * config.action_scales[i];
    }
    return targets;
}

std::array<double, 3> clampVelocity(double vx, double vy, double vyaw, const PolicyConfig& config)
{
    return { std::clamp(vx, -config.max_velocity[0], config.max_velocity[0]),
             std::clamp(vy, -config.max_velocity[1], config.max_velocity[1]),
             std::clamp(vyaw, -config.max_velocity[2], config.max_velocity[2]) };
}

bool isBelowGaitThreshold(const std::array<double, 3>& command, const PolicyConfig& config)
{
    for (std::size_t i = 0; i < 3; ++i)
    {
        if (std::abs(command[i]) >= config.gait_initiation_threshold[i])
        {
            return false;
        }
    }
    return true;
}

VelocityCommand latchVelocity(
    const std::array<double, 3>& command, double duration_s,
    std::chrono::steady_clock::time_point now, const PolicyConfig& config)
{
    const double clamped_s = std::clamp(duration_s, 0.0, config.velocity_duration_max_s);
    return VelocityCommand{ command[0],
                            command[1],
                            command[2],
                            now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(clamped_s)) };
}

std::array<double, 3> activeCommand(
    const std::optional<VelocityCommand>& latched, std::chrono::steady_clock::time_point now)
{
    if (!latched || now >= latched->expiry)
    {
        return { 0.0, 0.0, 0.0 };
    }
    return { latched->vx, latched->vy, latched->vyaw };
}

}  // namespace g1_bringup
