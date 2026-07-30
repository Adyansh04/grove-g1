#ifndef G1_BRINGUP__WALK_POLICY_HPP_
#define G1_BRINGUP__WALK_POLICY_HPP_

/**
 * @file walk_policy.hpp
 * @brief Pure observation assembly and action decoding for the sim-only walking policy.
 */

#include <array>
#include <cstddef>

namespace g1_bringup
{

/// Leg joints the policy drives: motors 0-11. Waist and arms are never touched.
inline constexpr std::size_t kNumPolicyJoints = 12;

/// Observation width the pretrained checkpoint expects (`num_obs: 47` in the vendor config).
inline constexpr std::size_t kNumPolicyObs = 47;

/**
 * @brief Scales and reference posture the policy was trained with.
 *
 * Every value is transcribed from `unitree_rl_gym`'s own
 * `deploy/deploy_real/configs/g1.yaml` at the SHA pinned in the dev image, not
 * re-derived. A wrong scale or a wrong default angle yields a policy that runs
 * and then falls over, so this struct exists to keep the numbers in one place
 * and let a unit test pin them against hand-computed values.
 */
struct WalkPolicyConfig
{
    double                               ang_vel_scale{ 0.25 };
    double                               dof_pos_scale{ 1.0 };
    double                               dof_vel_scale{ 0.05 };
    double                               action_scale{ 0.25 };
    std::array<double, 3>                cmd_scale{ 2.0, 2.0, 0.25 };
    std::array<double, 3>                max_cmd{ 0.8, 0.5, 1.57 };
    std::array<double, kNumPolicyJoints> default_angles{ -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                                                         -0.1, 0.0, 0.0, 0.3, -0.2, 0.0 };
    /// Gait phase period in seconds; the policy's clock signal, not a tunable.
    double gait_period_s{ 0.8 };
};

/**
 * @brief Projects gravity into the base frame from the IMU quaternion (w, x, y, z).
 *
 * Transcribed from the vendor's `get_gravity_orientation()`. This is what tells
 * the policy which way is down, so a sign error here is a fall.
 */
std::array<double, 3> gravityOrientation(const std::array<double, 4>& quat_wxyz);

/**
 * @brief Assembles the 47-element observation vector in the exact order the checkpoint expects.
 *
 * Layout, from the vendor's `run()`: `[0:3]` scaled angular velocity, `[3:6]`
 * gravity orientation, `[6:9]` scaled velocity command, `[9:21]` joint position
 * offset from the default posture, `[21:33]` scaled joint velocity, `[33:45]`
 * the previous action, `[45]` sin(phase), `[46]` cos(phase).
 *
 * @param command_mps  Velocity command as (vx, vy, vyaw) in m/s and rad/s; clamped to max_cmd
 *   here, so callers may pass an unclamped intent.
 * @param phase  Gait phase in [0, 1); see gaitPhase().
 */
std::array<double, kNumPolicyObs> assembleObservation(
    const WalkPolicyConfig& config, const std::array<double, 3>& base_angular_velocity,
    const std::array<double, 4>&                base_quat_wxyz,
    const std::array<double, kNumPolicyJoints>& joint_position,
    const std::array<double, kNumPolicyJoints>& joint_velocity,
    const std::array<double, kNumPolicyJoints>& previous_action,
    const std::array<double, 3>& command_mps, double phase);

/**
 * @brief Gait phase in [0, 1) from elapsed time — the policy's own clock.
 *
 * The vendor derives this from a tick counter times `control_dt`; using elapsed
 * seconds directly is equivalent and does not drift if a tick is ever missed.
 */
double gaitPhase(const WalkPolicyConfig& config, double elapsed_s);

/**
 * @brief Decodes policy output into leg joint targets: `default_angles + action * action_scale`.
 */
std::array<double, kNumPolicyJoints> actionToJointTargets(
    const WalkPolicyConfig& config, const std::array<double, kNumPolicyJoints>& action);

}  // namespace g1_bringup

#endif  // G1_BRINGUP__WALK_POLICY_HPP_
