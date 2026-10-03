#ifndef G1_LOCOMOTION__APPROACH_PLANNER_HPP_
#define G1_LOCOMOTION__APPROACH_PLANNER_HPP_

/**
 * @file approach_planner.hpp
 * @brief Where a measured object has to end up for the arm to reach it, and how fast to walk
 *        there.
 *
 * Proportional with a floor on the linear axes, because the gait ignores speeds inside its
 * deadband; yaw has none. All three axes run at once in one closed loop. Arrival is judged on the
 * linear axes only: heading just keeps the robot square to the surface while it closes.
 */

#include <cstdint>
#include <optional>

namespace g1_locomotion
{

enum class ApproachState : std::uint8_t
{
    kArrived,   ///< Inside the reachable window on both linear axes.
    kClosing,   ///< Outside it; drive at the velocity in the command.
    kOvershot,  ///< Under the robot's own footprint, where no walk helps. Re-stage via Nav2.
    kInvalid,   ///< The limits themselves are unusable.
};

/**
 * @brief Where the object should end up.
 *
 * Distances are in the horizontal plane of the base frame; the base cannot influence height.
 */
struct ApproachLimits
{
    /// Where the arm reaches, from IK at the workbench's height. target_y_m mirrors for the left
    /// arm.
    double target_x_m = 0.270;
    double target_y_m = -0.220;

    /// How close each axis has to get; the robot coasts 25 to 37 mm after the command stops.
    double forward_tolerance_m = 0.050;
    double lateral_tolerance_m = 0.040;

    /// Nearer than this the object is under the robot and no walk recovers it.
    double min_forward_m = 0.050;

    /// Loose on purpose: it only keeps the robot roughly square to the surface.
    double heading_tolerance_rad = 0.350;

    /// How far beyond the window's far edge a stopped robot may find the object: stopped, the
    /// pelvis settles back over the feet, so the object reads further than it did on the move.
    double settle_slack_m = 0.040;

    /// How far outside the window sideways an object may settle and be nudged in, not driven.
    double nudge_band_m = 0.050;
};

/**
 * @brief The velocity range the gait actually honours.
 */
struct GaitLimits
{
    /// Below these the gait does not move at all.
    double min_speed_x_mps = 0.20;
    double min_speed_y_mps = 0.25;

    /// Well inside what the policy tracks, since this runs next to furniture.
    double max_speed_x_mps  = 0.40;
    double max_speed_y_mps  = 0.35;
    double max_yaw_rate_rps = 0.60;

    /// Proportional gains, before the floors and ceilings clamp them.
    double speed_per_m      = 1.0;
    double yaw_rate_per_rad = 1.0;

    /// A sideways nudge's length at the floor speed: too short to commit a stride.
    double nudge_y_s = 0.15;
};

/**
 * @brief A short sideways pulse, for an object that settled just outside the window.
 */
struct Nudge
{
    double vy_mps  = 0.0;
    double seconds = 0.0;
};

/**
 * @brief The decision, plus the numbers behind it so a caller can log or publish them.
 */
struct ApproachCommand
{
    ApproachState state = ApproachState::kInvalid;

    /// Remaining error in the base frame. Positive forward means the object is too far ahead;
    /// positive lateral means it is too far to the robot's left.
    double forward_error_m = 0.0;
    double lateral_error_m = 0.0;

    /// Base-frame velocity to publish. Zero on every state but kClosing.
    double vx_mps       = 0.0;
    double vy_mps       = 0.0;
    double yaw_rate_rps = 0.0;
};

/**
 * @brief Validates the reach window before the planner is asked to aim at it.
 *
 * @return true if these limits describe a window the planner can aim at.
 */
bool limitsAreUsable(const ApproachLimits& limits);

/**
 * @brief Validates the gait envelope before a velocity is commanded from it.
 *
 * @return true if these gait limits describe a velocity range the robot can be asked for.
 */
bool gaitLimitsAreUsable(const GaitLimits& gait);

/**
 * @brief Decide whether the robot has arrived and, if not, how fast to walk.
 * @param object_x_m,object_y_m The object's position in the current base frame, the frame the
 *        arm works in, so the window means reachability.
 * @param heading_error_rad Working heading minus the robot's, already wrapped to [-pi, pi].
 * @return The state and, when closing, the base-frame velocity to command.
 */
ApproachCommand planApproach(
    double object_x_m, double object_y_m, double heading_error_rad, const ApproachLimits& limits,
    const GaitLimits& gait);

/**
 * @brief Whether a stopped robot has the object in reach: the window, with the settle slack on
 *        its far side.
 */
bool settledInReach(double object_x_m, double object_y_m, const ApproachLimits& limits);

/**
 * @brief The sideways pulse that nudges in an object that settled in reach ahead but just
 *        outside the window sideways, at the lateral floor, toward the target.
 * @return Nothing when it is in reach already, out of reach ahead, or further out sideways than
 *         the nudge band, where a drive does better.
 */
std::optional<Nudge> planNudge(
    double object_x_m, double object_y_m, const ApproachLimits& limits, const GaitLimits& gait);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
