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
 * THERE IS NO TURNING IN THE APPROACH. The heading comes from the navigation goal, which is
 * already aimed at the surface, and the arm does not care which way the room faces -- only where
 * the object sits relative to the robot. So the error is nulled with the three primitives that
 * are stable and cheap: drive forward, reverse, strafe. Yaw is the slow, asymmetric, bimodal one
 * and it is used only if the heading has drifted badly.
 *
 * Two earlier versions did turn, and both failed on it. An OBLIQUE forward step needs theta near
 * 75 degrees for a small remainder, which throws the robot half a metre sideways; a 45 degree
 * CREEP-and-strafe costs 45 degrees of turning each way, and one live run spent 48 pulses that
 * way taking out 17 cm of lateral error that five strafes would have covered.
 *
 * Fine forward motion, which the gait cannot produce directly, is instead a forward drive that
 * stops at zero and a reverse that takes back whatever the coast added. Reverse resolves more
 * finely than forward does: -0.247 m/s against 0.35.
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
    kReverse,   ///< Straight back. For having come too far.
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
    double target_x_m = 0.270;
    double target_y_m = -0.220;

    /// How close each axis has to get. The forward window is the measured band with margin at
    /// its near end, NOT a guess about how precise the base can be -- the base is not precise,
    /// and the point of measuring the arm properly was to find out how much slack it grants.
    double forward_tolerance_m = 0.110;
    double lateral_tolerance_m = 0.040;
    /// Deliberately loose. The window is judged in the base frame, so heading is not part of
    /// reachability: it only steers the drives. A tight value here spends the whole pulse
    /// budget taking out a few degrees that cost nothing.
    double heading_tolerance_rad = 0.350;

    /// Nearer than this and the object is under the robot's own shell. Sits just below the
    /// measured reachable band, which starts at 0.16, so there is room between "past the window"
    /// and "unrecoverable" for a backwards creep to work in.
    double min_forward_m = 0.120;

    /// More than this left and the caller may stop its forward drive early, trusting the coast.
    /// Less, and it drives to zero and lets a reverse take back the overshoot.
    double step_threshold_m = 0.32;
};

/// The decision, plus the numbers behind it so a caller can log or publish them.
struct ApproachCommand
{
    ApproachMove move = ApproachMove::kInvalid;
    /// kStep: true when more than a full step remains, so the caller can stop the drive early
    /// and let the gait coast. False means creep the last few centimetres in, stopping at zero
    /// and letting a reverse clean up whatever the coast adds.
    bool coarse = false;
    /// kTurn: the heading correction to make. Signed.
    double turn_rad = 0.0;
    /// kStrafe: +1 to strafe left, -1 to strafe right.
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
