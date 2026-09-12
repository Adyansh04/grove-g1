#ifndef G1_MANIPULATION__GRASP_FILTER_HPP_
#define G1_MANIPULATION__GRASP_FILTER_HPP_

/**
 * @file grasp_filter.hpp
 * @brief The arithmetic between a generated grasp and one this arm can be asked for.
 *
 * A generator answers in its own gripper frame and knows nothing about this robot: which link
 * the pose belongs to, which way the arm can approach, or that a table exists. These are the
 * two conversions that turn one into the other, kept free of MoveIt so they can be tested
 * without a robot.
 */

#include <array>
#include <geometry_msgs/msg/pose.hpp>

namespace g1_manipulation
{

/**
 * @brief Re-expresses a generated grasp as a goal for this robot's own grasp frame.
 *
 * @param generated Pose of the generator's gripper frame, in any frame.
 * @param xyz_rpy   Generator gripper frame to `<side>_hand_grasp_frame`, measured once against
 *                  the candidates in RViz. Right hand.
 * @param is_left   Mirrors the offset the way the URDF mirrors the two grasp frames: y, z and
 *                  the roll change sign.
 */
[[nodiscard]] geometry_msgs::msg::Pose applyGripperOffset(
    const geometry_msgs::msg::Pose& generated, const std::array<double, 6>& xyz_rpy, bool is_left);

/**
 * @brief Angle between a grasp's approach axis and straight down, in radians.
 *
 * The generator's convention is that +z of the grasp frame is the direction the hand travels as
 * it closes on the object. Zero is a grasp reaching straight down; past a right angle the hand
 * is coming up from underneath, which on a table means through it.
 */
[[nodiscard]] double approachTiltRad(const geometry_msgs::msg::Pose& generated);

/**
 * @brief The approach axis of a generated grasp, as a unit vector in the pose's own frame.
 */
[[nodiscard]] std::array<double, 3> approachAxis(const geometry_msgs::msg::Pose& generated);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GRASP_FILTER_HPP_
