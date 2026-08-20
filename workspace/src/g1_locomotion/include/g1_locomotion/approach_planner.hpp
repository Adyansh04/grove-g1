#ifndef G1_LOCOMOTION__APPROACH_PLANNER_HPP_
#define G1_LOCOMOTION__APPROACH_PLANNER_HPP_

/**
 * @file approach_planner.hpp
 * @brief Where a measured object has to end up for the arm to reach it, and how fast to walk
 *        there.
 *
 * Separated from the node so the control law is assertable without a simulator, a gait, or a
 * graph.
 *
 * The gait takes a velocity and returns a proportional fraction of it, with one property that
 * shapes everything here: a deadband on both linear axes. Commanded 0.10 m/s the robot does not
 * move at all, measured 0.016 forward and 0.007 lateral, and from about 0.20 upward it
 * tracks at 70-80 % of command. So the law is **proportional with a floor**, not plain
 * proportional: a pure P term near the target asks for a speed the robot ignores, and the
 * approach stalls a few centimetres short of the window. Yaw has no deadband and tracks near
 * 1:1, so it gets no floor. The measurements are in the package README.
 *
 * All three axes are driven at once, which is why there is no move sequencing here at all: the
 * skill is one closed loop, not a plan of primitives.
 *
 * Heading is not part of arriving. Reachability is judged in the base frame, by where the object
 * sits relative to the robot, and which way the room faces is not part of that. The heading
 * term only keeps the robot square to the surface while it closes, and it is zeroed the moment
 * both linear axes are inside their tolerances.
 */

#include <cstdint>

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
    /// MEASURED with /compute_ik at the workbench cube's height: x 0.16 to 0.36 all solve,
    /// 0.38 does not. target_y_m mirrors for the left arm, exactly as the grasp offset does.
    double target_x_m = 0.270;
    double target_y_m = -0.220;

    /// How close each axis has to get. Tighter than the band the arm grants (+/-0.11 forward),
    /// because the loop can hold it: measured coast after the command stops is 0.025-0.037 m.
    double forward_tolerance_m = 0.050;
    double lateral_tolerance_m = 0.040;

    /// Nearer than this and the object is genuinely under the robot. Everything above it is
    /// recovered by reversing, so keep it well below the band's near end.
    double min_forward_m = 0.050;

    /// Loose on purpose: this only decides how square the robot stands to the surface while it
    /// closes, and holding it tightly buys nothing.
    double heading_tolerance_rad = 0.350;
};

/**
 * @brief What the gait will actually honour. Every number here is measured, not chosen.
 */
struct GaitLimits
{
    /// Floors, not minimum-useful speeds: below these the gait delivers nothing at all.
    double min_speed_x_mps = 0.20;
    double min_speed_y_mps = 0.25;

    /// Ceilings. Well inside what the policy tracks, because this skill works next to furniture.
    double max_speed_x_mps  = 0.40;
    double max_speed_y_mps  = 0.35;
    double max_yaw_rate_rps = 0.60;

    /// Proportional gains. 1.0 means "one metre of error asks for one metre per second", which
    /// the clamps then bring into the honoured range.
    double speed_per_m      = 1.0;
    double yaw_rate_per_rad = 1.0;
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

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
