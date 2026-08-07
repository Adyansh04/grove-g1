#ifndef G1_LOCOMOTION__APPROACH_PLANNER_HPP_
#define G1_LOCOMOTION__APPROACH_PLANNER_HPP_

/**
 * @file approach_planner.hpp
 * @brief Decides the next move when walking the base into arm's reach of a measured object.
 *
 * Separated from the node for the reason GaitShaper is: this is where the reasoning lives, and
 * it is worth testing without a simulator, a gait, or a graph.
 *
 * The problem it solves is not tracking a setpoint. This gait's smallest forward pulse measures
 * 0.3 to 0.6 m and the arm's reachable window is about 0.12 m wide, so a controller that walks
 * straight at the target cannot land inside it except by luck. The answer is to walk OBLIQUELY
 * once the remaining distance is under one pulse: a pulse taken at angle theta off the line
 * advances `quantum * cos(theta)`, so the advance shrinks continuously while the step itself
 * stays the only size the gait can produce.
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
    kTurn,      ///< Yaw by `turn_rad` before anything else.
    kStep,      ///< Heading is right; take one forward pulse.
    kOvershot,  ///< Closer than the arm can work with, and the gait cannot reverse.
    kInvalid,   ///< The limits themselves are unusable.
};

/// Where the object should end up, and what the gait can do about it. Every distance is in the
/// horizontal plane of the base frame; the base cannot influence height.
struct ApproachLimits
{
    /// Range and bearing the object should sit at when the arm can reach it. Defaults are the
    /// pelvis-frame position the manipulation scene's grasp is proven at, (0.28, -0.20) for the
    /// right hand, expressed in polar form.
    double target_range_m     = 0.344;
    double target_bearing_rad = -0.620;
    double range_tolerance_m  = 0.055;
    /// Wider than it looks it should be: one yaw pulse turns about 9 degrees, and a tolerance
    /// tighter than the turn quantum cannot be held, so the robot hunts instead of settling.
    double bearing_tolerance_rad = 0.200;

    /// Below this the object is under the robot's own shell and the arm has nowhere to go. The
    /// gait cannot reverse, so this is a terminal failure rather than something to correct.
    double min_range_m = 0.215;

    /// How far one forward pulse actually carries the robot. Seeded from measurement and then
    /// raised by the caller as pulses are observed -- see maxObservedAdvance().
    double pulse_advance_m = 0.35;

    /// Ceiling on how far off the line an oblique step may aim. At theta the step also moves
    /// the robot `quantum * sin(theta)` sideways, so this bounds the excursion; 75 degrees
    /// leaves an advance of about a quarter of a pulse, which is finer than the window.
    double max_oblique_rad = 1.309;
};

/// The decision, plus the numbers behind it so a caller can log or publish them.
struct ApproachCommand
{
    ApproachMove move = ApproachMove::kInvalid;
    /// Heading change to make before stepping. Signed; zero for kStep and kDone.
    double turn_rad = 0.0;
    /// Range still to close to the centre of the window. Negative means too close.
    double remaining_m = 0.0;
    /// The oblique angle folded into `turn_rad`, for logging. Zero when stepping straight in.
    double oblique_rad = 0.0;
};

/// True if these limits describe a window the planner can aim at.
bool limitsAreUsable(const ApproachLimits& limits);

/**
 * @brief Decide the next move from the object's measured position in the base frame.
 * @param range_m   Horizontal distance from the base origin to the object.
 * @param bearing_rad Bearing to the object, zero straight ahead, left positive.
 *
 * Bearing is corrected before range, because turning barely changes range while walking
 * off-bearing wastes a pulse the robot cannot take back.
 */
ApproachCommand planApproach(double range_m, double bearing_rad, const ApproachLimits& limits);

/**
 * @brief Fold an observed pulse displacement into the quantum estimate.
 *
 * Deliberately a running MAXIMUM rather than an average. The two errors are not symmetric:
 * undershooting costs one more pulse, while overshooting puts the object inside the robot and
 * cannot be undone at all, because reverse is inside the gait's dead zone. Assuming the pulse
 * is as large as anything yet seen biases every oblique step toward undershoot.
 */
double maxObservedAdvance(double current_estimate_m, double observed_m);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
