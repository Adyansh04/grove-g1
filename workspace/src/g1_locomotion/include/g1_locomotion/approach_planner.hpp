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
 * So the coarse approach is forward steps and the FINE approach is not a forward step at all.
 * Turning off the working heading by a moderate angle and strafing advances
 * `strafe_quantum * sin(offset)` per pulse: at 45 degrees that is 2.5 cm forward and 2.5 cm
 * sideways, both precise, and the sideways part can be pointed at whichever side the lateral
 * error needs.
 *
 * An earlier version closed the last gap with an OBLIQUE forward step instead -- turn by theta,
 * take the one step size the gait has, advance `quantum * cos(theta)`. It does not survive
 * contact: a small remainder needs theta near 75 degrees, which throws the robot half a metre
 * sideways and costs about forty pulses to turn into and out of, at 3.8 degrees a pulse. Live,
 * it oscillated across the target and spent its whole budget turning.
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
    kStep,      ///< On the working heading, take ONE forward pulse. Coarse: about 0.29 m.
    kCreep,     ///< Aim at `working_yaw + fine_offset_rad`, then strafe. Fine: about 0.025 m in.
    kStrafe,    ///< One lateral pulse on the working heading; `lateral_sign` says which way.
    kTurn,      ///< Yaw by `turn_rad` to restore the working heading.
    kOvershot,  ///< Closer than the arm can work with, and the gait cannot reverse.
    kInvalid,   ///< The limits themselves are unusable.
};

/// Where the object should end up, and what the gait can do about it. Distances are in the
/// horizontal plane of the base frame; the base cannot influence height.
struct ApproachLimits
{
    /// Where the object must sit in the base frame for the arm to reach it. MEASURED with
    /// /compute_ik at the workbench cube's height: x 0.16 to 0.36 all solve, 0.38 does not.
    double target_x_m = 0.280;
    double target_y_m = -0.220;

    /// How close each axis has to get. The forward window is the measured band with margin at
    /// its near end, NOT a guess about how precise the base can be -- the base is not precise,
    /// and the point of measuring the arm properly was to find out how much slack it grants.
    double forward_tolerance_m = 0.060;
    double lateral_tolerance_m = 0.040;
    /// The yaw response is bimodal -- one 0.15 s pulse measured 3 to 14 degrees -- so a
    /// tolerance near a single quantum cannot be held. Residual heading error shows up as
    /// lateral error anyway, which is the cheap axis to correct.
    double heading_tolerance_rad = 0.140;

    /// Nearer than this and the object is under the robot's own shell. Sits just below the
    /// measured reachable band, which starts at 0.16, so there is room between "past the window"
    /// and "unrecoverable" for a backwards creep to work in.
    double min_forward_m = 0.140;

    /// Take a forward step while more than this is left, and creep below it. Roughly one
    /// typical step (measured 0.29 to 0.50 depending on how warm the gait is), so a step is
    /// taken whenever one is likely to land nearer the window than it started.
    ///
    /// It does NOT have to be an upper bound on the step, which is what it used to be. That
    /// version refused to step with 0.58 m left, on the grounds that a 0.60 m step might sail
    /// past the window, and then tried to close the whole 0.58 m at 2.5 cm a creep. Overshooting
    /// is recoverable now, so being cautious here costs far more than it saves.
    double step_threshold_m = 0.32;

    /// How far off the working heading a creep aims. The trade is fixed by trigonometry: a
    /// strafe there advances `strafe * sin(offset)` and slides `strafe * cos(offset)`. 45 degrees
    /// splits it evenly, costs about twelve pulses to turn into, and keeps the slide useful
    /// rather than wasted -- it is pointed at whichever side the lateral error needs.
    double fine_offset_rad = 0.785;
};

/// The decision, plus the numbers behind it so a caller can log or publish them.
struct ApproachCommand
{
    ApproachMove move = ApproachMove::kInvalid;
    /// kCreep: heading offset to aim at, relative to the working heading. Signed, and signed
    /// OPPOSITE to the strafe, since turning left and strafing right is what moves forward.
    double fine_offset_rad = 0.0;
    /// kTurn: the heading correction to make. Signed.
    double turn_rad = 0.0;
    /// kStrafe and kCreep: +1 to strafe left, -1 to strafe right.
    double lateral_sign = 0.0;
    /// Remaining error in the base frame, for feedback and logging.
    double forward_error_m = 0.0;
    double lateral_error_m = 0.0;
};

/// True if these limits describe a window the planner can aim at.
bool limitsAreUsable(const ApproachLimits& limits);

/**
 * @brief Decide the next move.
 * @param object_x_m,object_y_m  The object's position in the WORKING-HEADING frame, not the
 *        current base frame. The caller rotates it; see the note below.
 * @param heading_error_rad      `working_yaw - current_yaw`, wrapped. What a kTurn would undo.
 *
 * The frame matters and got this wrong once. A creep deliberately leaves the robot turned 45 deg
 * off the working heading, so the object's position in the CURRENT base frame is measured in a
 * rotated frame and neither error component means what it says. Live, that showed up as a
 * forward error stuck at 0.50 m while the lateral error grew, with the robot creeping on numbers
 * that described a frame it was no longer trying to reach.
 *
 * Forward is resolved before heading and lateral, because a forward step is the only move that
 * needs a deliberately wrong heading and the other two exist largely to clean up after it.
 */
ApproachCommand planApproach(
    double object_x_m, double object_y_m, double heading_error_rad, const ApproachLimits& limits);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
