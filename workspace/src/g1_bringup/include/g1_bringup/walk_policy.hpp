#ifndef G1_BRINGUP__WALK_POLICY_HPP_
#define G1_BRINGUP__WALK_POLICY_HPP_

/**
 * @file walk_policy.hpp
 * @brief Observation assembly, action mapping, and velocity latching for the sim walking policy.
 */

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "g1_bringup/blend_math.hpp"

namespace g1_bringup
{

/// Motors the policy owns: legs 0-11 plus waist 12-14. Arms (kFirstArmMotor..) stay with /arm_sdk.
inline constexpr int kNumLowerMotors = kFirstArmMotor;

/**
 * @brief Observation layout the policy was trained against, as element offsets.
 *
 * base_lin_vel(3) | base_ang_vel(3) | projected_gravity(3) | joint_pos(29)
 * | joint_vel(29) | last_action(29) | command(3) = 99. Both velocity blocks are
 * in the base frame, and joint_pos is measured RELATIVE to the default posture.
 */
inline constexpr std::size_t kObsBaseLinVel = 0;
inline constexpr std::size_t kObsBaseAngVel = 3;
inline constexpr std::size_t kObsGravity    = 6;
inline constexpr std::size_t kObsJointPos   = 9;
inline constexpr std::size_t kObsJointVel   = 38;
inline constexpr std::size_t kObsLastAction = 67;
inline constexpr std::size_t kObsCommand    = 96;
inline constexpr std::size_t kObsDim        = 99;
inline constexpr std::size_t kActionDim     = kNumBodyMotors;

/**
 * @brief The 29 joint names in Unitree DDS motor-index order.
 *
 * The authority is unitree_mujoco's own unitree_robots/g1/g1_joint_index_dds.md
 * (29DOF table), which LowCmd.motor_cmd[] and LowState.motor_state[] both follow.
 * The policy's own joint order happens to match this exactly, one-for-one, so no
 * remap table is needed -- but checkJointOrder() asserts it rather than trusting
 * it, because a silent permutation would land every gain and target on the wrong
 * joint. The reference package shipped precisely that bug by indexing a 29-name
 * list into a 43-joint model.
 */
extern const std::array<const char*, kNumBodyMotors> kDdsMotorOrder;

/**
 * @brief Verifies configured joint names against the DDS motor order.
 *
 * @param joint_names  Joint names as configured, expected in DDS motor-index order.
 * @return Empty string when the order matches; otherwise a message naming the first mismatch.
 */
std::string checkJointOrder(const std::vector<std::string>& joint_names);

/**
 * @brief A velocity command latched from a LocoClient SET_VELOCITY request.
 *
 * Carries its own expiry rather than relying on a separate timer: the vendor's
 * `duration` field IS the dead-man, so honouring it here means a silent or dead
 * bridge stops the robot without any parallel timeout mechanism.
 */
struct VelocityCommand
{
    double                                vx{ 0.0 };
    double                                vy{ 0.0 };
    double                                vyaw{ 0.0 };
    std::chrono::steady_clock::time_point expiry{};
};

/**
 * @brief Rotates a world-frame vector into the base frame using the base orientation.
 *
 * Applies the inverse of `quat` to `v`. Used for both the projected-gravity block
 * (rotating world -Z) and the base linear velocity, which arrives from
 * /sportmodestate in the world frame while the policy expects it base-relative.
 *
 * @param quat  Base orientation as (w, x, y, z), matching LowState.imu_state.quaternion.
 * @param v     Vector in the world frame.
 * @return `v` expressed in the base frame.
 */
std::array<double, 3>
rotateWorldToBase(const std::array<double, 4>& quat, const std::array<double, 3>& v);

/**
 * @brief Per-tick inputs to the policy, gathered from /lowstate and /sportmodestate.
 */
struct PolicyInputs
{
    std::array<double, 3>              base_lin_vel_world{};  ///< /sportmodestate velocity
    std::array<double, 4>              base_quat{ 1.0, 0.0, 0.0, 0.0 };  ///< imu_state.quaternion
    std::array<double, 3>              base_ang_vel_body{};              ///< imu_state.gyroscope
    std::array<double, kNumBodyMotors> joint_pos{};                      ///< motor_state[i].q
    std::array<double, kNumBodyMotors> joint_vel{};                      ///< motor_state[i].dq
};

/**
 * @brief Static, config-derived quantities the observation and action mapping need.
 */
struct PolicyConfig
{
    std::array<double, kNumBodyMotors>  default_joint_pos{};
    std::array<double, kNumBodyMotors>  action_scales{};
    std::array<double, kNumLowerMotors> lower_kp{};
    std::array<double, kNumLowerMotors> lower_kd{};
    std::array<double, 3>               max_velocity{ 1.0, 0.8, 2.0 };
    std::array<double, 3>               gait_initiation_threshold{ 0.4, 0.5, 1.5 };
    double                              velocity_duration_max_s{ 2.0 };
};

/**
 * @brief Assembles the 99-element observation vector in the policy's trained layout.
 *
 * Returned RAW, deliberately un-normalised: the exported graph begins with
 * Sub(mean) then Div(std), so normalising here would apply it twice. The
 * obs_mean/obs_std arrays in the reference package's model_config.json are
 * informational duplicates of those baked-in constants and must not be applied.
 *
 * @param inputs       Latest base and joint state.
 * @param config       Default posture (joint_pos is relative to it) and limits.
 * @param last_action  Previous policy output, fed back as part of the observation.
 * @param command      Velocity command (vx, vy, vyaw) already clamped.
 * @return The 99-element observation.
 */
std::array<float, kObsDim> assembleObservation(
    const PolicyInputs& inputs, const PolicyConfig& config,
    const std::array<float, kActionDim>& last_action, const std::array<double, 3>& command);

/**
 * @brief Maps raw policy actions to joint position targets.
 *
 * target = default_joint_pos + action * action_scales, per joint. All 29 are
 * computed; the caller uses only the lower-body 15 (see kNumLowerMotors).
 *
 * @param action  Raw policy output.
 * @param config  Default posture and per-joint action scales.
 * @return Position targets for all body motors.
 */
std::array<double, kNumBodyMotors>
actionToJointTargets(const std::array<float, kActionDim>& action, const PolicyConfig& config);

/**
 * @brief Clamps a requested velocity to the configured per-axis limits.
 *
 * @param vx      Requested forward velocity, m/s.
 * @param vy      Requested lateral velocity, m/s.
 * @param vyaw    Requested yaw rate, rad/s.
 * @param config  Supplies max_velocity.
 * @return The clamped (vx, vy, vyaw).
 */
std::array<double, 3> clampVelocity(double vx, double vy, double vyaw, const PolicyConfig& config);

/**
 * @brief Whether a command is below the policy's measured gait-initiation threshold.
 *
 * Advisory only -- the caller logs and passes the command through unchanged.
 * Scaling a below-threshold command UP to make the robot move would turn a small
 * command into a large motion, which CONTROL_MODES.md forbids outright.
 *
 * @param command  Clamped (vx, vy, vyaw).
 * @param config   Supplies gait_initiation_threshold.
 * @return True when every axis sits below its threshold, so the robot will not step.
 */
bool isBelowGaitThreshold(const std::array<double, 3>& command, const PolicyConfig& config);

/**
 * @brief Builds a latched command with its expiry derived from the request's duration.
 *
 * `duration_s` is clamped to [0, velocity_duration_max_s] so a rogue client cannot
 * latch the vendor's 864000 s "continuous" value indefinitely.
 *
 * @param command     Clamped (vx, vy, vyaw).
 * @param duration_s  Dead-man duration from the SET_VELOCITY request, in seconds.
 * @param now         Arrival time.
 * @param config      Supplies velocity_duration_max_s.
 * @return The latched command carrying its own expiry.
 */
VelocityCommand latchVelocity(
    const std::array<double, 3>& command, double duration_s,
    std::chrono::steady_clock::time_point now, const PolicyConfig& config);

/**
 * @brief The command in force now, or a zero command once the dead-man has expired.
 *
 * @param latched  Currently latched command, if any.
 * @param now      Current time.
 * @return (vx, vy, vyaw), all zero when nothing is latched or the latch has expired.
 */
std::array<double, 3> activeCommand(
    const std::optional<VelocityCommand>& latched, std::chrono::steady_clock::time_point now);

}  // namespace g1_bringup

#endif  // G1_BRINGUP__WALK_POLICY_HPP_
