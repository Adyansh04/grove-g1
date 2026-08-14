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
 * So lateral is a precision knob and forward is a sledgehammer. The arm's window is about
 * 0.11 m wide, a third of one forward step.
 *
 * THE PLANNER NEVER ASKS FOR A TURN. Reachability is judged in the base frame -- where the
 * object sits relative to the robot -- and heading is simply not part of that, so it is not an
 * input here. The caller holds its working heading on its own during forward drives, because a
 * forward step yaws +8 deg and an uncorrected sequence walks an arc; that aim loop is the only
 * yaw in the skill, and it belongs to the caller, not the plan.
 *
 * Earlier versions planned turns as part of this decision, and each one failed differently: an
 * oblique step throws the robot sideways for a small remainder, a turn-and-strafe hybrid burns
 * far more pulses than strafing alone, and a terminal heading gate can deadlock the caller if it
 * ever stops handling turns. Heading stays out of this decision entirely.
 *
 * Fine forward motion, which the gait cannot produce directly, is instead a forward drive that
 * stops at zero and a reverse that takes back whatever the coast added. Reverse resolves more
 * finely than forward does: -0.247 m/s against 0.35.
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
    kStrafe,    ///< One lateral pulse; `lateral_sign` says which way.
    kOvershot,  ///< Under the robot's own shell, where no move here helps. Re-stage through Nav2.
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

    /// Nearer than this and the object is genuinely under the robot, where no move helps.
    /// Everything above it is recovered by reversing, so keep it WELL below the arm band's near
    /// end: the last centimetres of forward error are closed by deliberately overshooting and
    /// reversing back, and a floor set near the band turns that designed move into an abort.
    double min_forward_m = 0.050;

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
 * @param object_x_m,object_y_m  The object's position in the CURRENT base frame -- the frame
 *        the arm works in, which is what makes the window mean reachability.
 *
 * Forward is resolved before lateral, because a forward step is the move that yaws and drifts,
 * and the lateral error it leaves behind is cheap to strafe out afterwards. The other order
 * would strafe to a place the next drive walks away from.
 */
ApproachCommand planApproach(double object_x_m, double object_y_m, const ApproachLimits& limits);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__APPROACH_PLANNER_HPP_
