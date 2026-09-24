#ifndef G1_MANIPULATION__GRIP_CHECK_HPP_
#define G1_MANIPULATION__GRIP_CHECK_HPP_

/**
 * @file grip_check.hpp
 * @brief Whether a closed hand is holding something, from what the fingers report.
 *
 * An empty hand reaches its posture unloaded; a holding one stalls short and pushes, which the
 * trajectory controller also reports as success. The Dex3 reports tau_est as the simulator does.
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
 * @brief Whether at least two of the hand's three fingers are pressing on something.
 *
 * A joint is loaded when it is at least @p min_error_rad short of its target and pushing at least
 * @p min_effort_nm: short alone is a slack drive, pushing alone a finger holding at its target.
 * Two distinct fingers, not two joints: one finger jammed against the table reads like a grip.
 *
 * @param joints Every joint of one hand, in any order. thumb_0 is ignored: it rolls the thumb
 *               rather than closing it.
 */
[[nodiscard]] GripVerdict
verifyGrip(const std::vector<JointGrip>& joints, double min_error_rad, double min_effort_nm);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GRIP_CHECK_HPP_
