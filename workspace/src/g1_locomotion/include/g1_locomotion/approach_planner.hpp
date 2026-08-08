#ifndef G1_LOCOMOTION__APPROACH_PLANNER_HPP_
#define G1_LOCOMOTION__APPROACH_PLANNER_HPP_

/**
 * @file approach_planner.hpp
 * @brief Decides the next move when walking the base into arm's reach of a measured object.
 *
 * Separated from the node for the reason GaitShaper is: this is where the reasoning lives, and
 * it is worth testing without a simulator, a gait, or a graph.
 *
 * The problem is that the gait's primitives are quantised, and quantised unevenly. Measured:
 *
 *   - forward is IRREDUCIBLE at about 0.29 m, whatever duration is commanded, and yaws +8 deg
 *   - lateral resolves to about 0.035 m, with under a degree of yaw and no forward coupling
 *   - yaw resolves to about 3.8 deg, and moves the robot 3 mm
 *
 * So lateral and heading are precision knobs and forward is a sledgehammer. The arm's window is
 * about 0.11 m wide, a third of one forward step.
 *
 * The answer is to take the last forward step OBLIQUELY: turn by theta, take the one step size
 * the gait has, and advance `quantum * cos(theta)`. Since theta is settable in 3.8 deg
 * increments, near 60 degrees that resolves the advance to under 2 cm. Whatever lateral error
 * the oblique step introduces is then strafed out, which is the cheap direction.
 *
 * The heading is NOT derived from where the object is. It is given by the caller and held. That
 * is the standard mobile-manipulation pattern -- a stand-off pose on the surface normal, already
 * facing the working direction -- and the previous version's habit of re-aiming at the object
 * from a metre out is what made it walk in on a curve.
 *
 * Measurements and the rest of the rationale: docs/notes/m9-base-approach.md.
 */

#include <cstdint>

namespace g1_locomotion
{

/// What the caller should do next.
enum class ApproachMove : std::uint8_t
{
    kDone,      ///< The object is inside the reachable window.
    kStep,      ///< Aim at `working_yaw + oblique_rad`, then take ONE forward pulse.
    kStrafe,    ///< One lateral pulse; `lateral_sign` says which way.
    kTurn,      ///< Yaw by `turn_rad` to restore the working heading.
    kOvershot,  ///< Closer than the arm can work with, and the gait cannot reverse.
    kInvalid,   ///< The limits themselves are unusable.
};

/// Where the object should end up, and what the gait can do about it. Distances are in the
/// horizontal plane of the base frame; the base cannot influence height.
struct ApproachLimits
{
    /// Where the object must sit in the base frame for the arm to reach it. Defaults are the
    /// position the grasp is PROVEN at: the manipulation scene welds the pelvis at the origin,
    /// puts the cube at (0.28, -0.20), and the pick succeeds there end to end.
    double target_x_m = 0.280;
    double target_y_m = -0.200;

    /// How close each axis has to get. Forward is the tight one because it is the axis the gait
    /// cannot resolve directly; lateral is generous relative to its own 0.035 m quantum.
    double forward_tolerance_m = 0.045;
    double lateral_tolerance_m = 0.050;
    /// The yaw response is bimodal -- one 0.15 s pulse measured 3 to 14 degrees -- so a
    /// tolerance near a single quantum cannot be held. Residual heading error shows up as
    /// lateral error anyway, which is the cheap axis to correct.
    double heading_tolerance_rad = 0.140;

    /// Forward of this and the object is under the robot's own shell. Terminal, not correctable:
    /// the gait has no reverse.
    double min_forward_m = 0.180;

    /// What one forward pulse actually carries the robot. Seeded from measurement and raised,
    /// never lowered, by the caller as pulses are observed. See maxObservedAdvance().
    double pulse_advance_m = 0.293;

    /// Ceiling on the oblique. At theta the step also moves `quantum * sin(theta)` sideways,
    /// which then has to be strafed out at 0.035 m a pulse, so a very large theta is expensive
    /// rather than wrong. 75 deg leaves an advance of about a quarter of a step.
    double max_oblique_rad = 1.309;
};

/// The decision, plus the numbers behind it so a caller can log or publish them.
struct ApproachCommand
{
    ApproachMove move = ApproachMove::kInvalid;
    /// kStep: how far off the working heading to aim before stepping. Signed.
    double oblique_rad = 0.0;
    /// kTurn: the heading correction to make. Signed.
    double turn_rad = 0.0;
    /// kStrafe: +1 to move left, -1 to move right.
    double lateral_sign = 0.0;
    /// Remaining error in the base frame, for feedback and logging.
    double forward_error_m = 0.0;
    double lateral_error_m = 0.0;
};

/// True if these limits describe a window the planner can aim at.
bool limitsAreUsable(const ApproachLimits& limits);

/**
 * @brief Decide the next move.
 * @param object_x_m,object_y_m  The object's position in the base frame.
 * @param heading_error_rad      `working_yaw - current_yaw`, wrapped. What a kTurn would undo.
 *
 * Forward is resolved before heading and lateral, because a forward step is the only move that
 * needs a deliberately wrong heading and the other two exist largely to clean up after it.
 */
ApproachCommand planApproach(
    double object_x_m, double object_y_m, double heading_error_rad, const ApproachLimits& limits);

/**
 * @brief Fold an observed pulse displacement into the quantum estimate.
 *
 * Deliberately a running MAXIMUM rather than an average. The two errors are not symmetric:
 * undershooting costs one more pulse, while overshooting puts the object inside the robot and
 * cannot be undone at all, because reverse is inside the gait's dead zone. Assuming the pulse is
 * as large as anything yet seen biases every oblique step toward undershoot.
 */
double maxObservedAdvance(double current_estimate_m, double observed_m);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
