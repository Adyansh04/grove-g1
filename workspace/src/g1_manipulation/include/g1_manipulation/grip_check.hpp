#ifndef G1_MANIPULATION__GRIP_CHECK_HPP_
#define G1_MANIPULATION__GRIP_CHECK_HPP_

/**
 * @file grip_check.hpp
 * @brief Whether a closed hand is holding something, from what the fingers report.
 *
 * An empty hand reaches its posture unloaded; a holding one stalls short and pushes, which the
 * trajectory controller also reports as success. The Dex3 reports tau_est and the Inspire hand its
 * finger force as a torque, as the simulator does.
 */

#include <string>
#include <vector>

namespace g1_manipulation
{

/// One finger joint as the hand reports it: radians, radians, newton metres.
struct JointGrip
{
    std::string name;
    double      target{ 0.0 };
    double      position{ 0.0 };
    double      effort{ 0.0 };
    /// False for a joint the close does not move, such as a thumb's rotation: it can stall on its
    /// travel in an empty hand and says nothing about a grip.
    bool closes{ true };
    /// rad. How far short of its target this joint reads as blocked, for a joint whose close is
    /// shorter than the hand's others; 0 takes verifyGrip()'s @p min_error_rad.
    double min_error_rad{ 0.0 };
};

struct GripVerdict
{
    bool holding{ false };
    /// Holding with the thumb among the pressing fingers. Index and middle alone, side by side,
    /// push a free-standing object away.
    bool opposed{ false };
    /// How many fingers press and how hard, or why the hand reads empty. Goes into the result.
    std::string why;
};

/**
 * @brief Whether at least two of the hand's fingers, the thumb counting as one, are pressing on
 *        something.
 *
 * A joint is loaded when it is at least @p min_error_rad (or its own JointGrip::min_error_rad)
 * short of its target and pushing at least @p min_effort_nm: short alone is a slack drive, pushing
 * alone a finger holding at its target.
 * Two distinct fingers, not two joints: one finger jammed against the table reads like a grip.
 *
 * @param joints Every joint of one hand, in any order. Those that do not close are ignored, and a
 *               joint is a finger's by its name: thumb, index, middle, ring or pinky.
 */
[[nodiscard]] GripVerdict
verifyGrip(const std::vector<JointGrip>& joints, double min_error_rad, double min_effort_nm);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GRIP_CHECK_HPP_
