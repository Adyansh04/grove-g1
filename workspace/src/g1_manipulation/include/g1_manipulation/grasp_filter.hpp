#ifndef G1_MANIPULATION__GRASP_FILTER_HPP_
#define G1_MANIPULATION__GRASP_FILTER_HPP_

/**
 * @file grasp_filter.hpp
 * @brief The arithmetic between a generated grasp and one this arm can be asked for.
 *
 * A generator answers in its own gripper frame and knows nothing about this robot. Kept free of
 * MoveIt so it can be tested without one.
 */

#include <array>
#include <geometry_msgs/msg/pose.hpp>

namespace g1_manipulation
{

/**
 * @brief Re-expresses a generated grasp as a goal for this robot's own grasp frame.
 *
 * @param generated Pose of the generator's gripper frame, in any frame.
 * @param xyz_rpy   Generator gripper frame to `<side>_hand_grasp_frame`, for the right hand,
 *                  measured against the candidates in RViz.
 * @param is_left   Mirrors it the way the URDF does: y, z and the roll change sign.
 */
[[nodiscard]] geometry_msgs::msg::Pose applyGripperOffset(
    const geometry_msgs::msg::Pose& generated, const std::array<double, 6>& xyz_rpy, bool is_left);

/**
 * @brief Angle between a grasp's approach axis and straight down, in radians.
 *
 * +z of the grasp frame is the direction the hand travels. Zero reaches straight down; past a
 * right angle it is coming up from underneath, which on a table means through it.
 */
[[nodiscard]] double approachTiltRad(const geometry_msgs::msg::Pose& generated);

/**
 * @brief The approach axis of a generated grasp, as a unit vector in the pose's own frame.
 */
[[nodiscard]] std::array<double, 3> approachAxis(const geometry_msgs::msg::Pose& generated);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GRASP_FILTER_HPP_
