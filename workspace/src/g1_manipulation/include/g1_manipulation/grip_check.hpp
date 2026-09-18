#ifndef G1_MANIPULATION__GRIP_CHECK_HPP_
#define G1_MANIPULATION__GRIP_CHECK_HPP_

/**
 * @file grip_check.hpp
 * @brief Whether a closed hand is holding something, from what the fingers report.
 *
 * A hand that closed on nothing reaches its commanded posture with no torque; a hand holding an
 * object stalls short of it and pushes. The trajectory controller cannot tell the two apart (it
 * has no per-joint goal tolerances, so a blocked finger still reports success), so the skill asks
 * here. Kept free of ROS so it can be tested without a node, and the same arithmetic runs on the
 * robot: the Dex3 reports tau_est the same way the simulator does.
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
    /// Which fingers are pressing, or why the hand is judged empty. Goes into the action result.
    std::string why;
};

/**
 * @brief Whether at least two of the hand's three fingers are pressing on something.
 *
 * A joint counts as loaded when it stalls at least @p min_error_rad short of its target *and*
 * pushes with at least @p min_effort_nm. Both, not either: a slack drive is short without
 * pushing, and a finger resting against its own end stop pushes without being short.
 *
 * Two distinct fingers rather than one, and named rather than counted by joint: a grip is an
 * object squeezed between surfaces, and one finger jammed against the table stalls and pushes
 * exactly like one holding something. In practice a good grasp loads all three, the thumb
 * opposing index and middle across the object.
 *
 * @param joints Every joint of one hand, in any order. thumb_0 is ignored: it rolls the thumb
 *               rather than closing it.
 */
[[nodiscard]] GripVerdict
verifyGrip(const std::vector<JointGrip>& joints, double min_error_rad, double min_effort_nm);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GRIP_CHECK_HPP_
