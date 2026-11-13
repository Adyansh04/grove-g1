#ifndef G1_MANIPULATION__GRASP_FILTER_HPP_
#define G1_MANIPULATION__GRASP_FILTER_HPP_

/**
 * @file grasp_filter.hpp
 * @brief The arithmetic between a generated grasp and one this arm can be asked for.
 *
 * A generator answers in its own gripper frame and knows nothing about this robot.
 */

#include <array>
#include <geometry_msgs/msg/pose.hpp>

namespace g1_manipulation
{

/**
 * @brief Re-expresses a generated grasp as a goal for this robot's own grasp frame.
 *
 * @param generated Pose of the generator's gripper frame, in any frame.
 * @param xyz_rpy   Generator gripper frame to the grasp frame of the hand the generator serves,
 *                  metres then radians. Measured per hand: a generator serves one.
 */
[[nodiscard]] geometry_msgs::msg::Pose
applyGripperOffset(const geometry_msgs::msg::Pose& generated, const std::array<double, 6>& xyz_rpy);

/**
 * @brief The gripper offset for the hand a generator answers by mirroring.
 *
 * g1_perception's grasp adapter answers the hand its generator does not describe with grasps found
 * in the mirrored scene, each reflected back through the camera's y-z plane with the gripper's own
 * x axis reversed. For hands whose grasp frames mirror each other through their x-z planes, as the
 * G1's do, this is the offset from such a grasp to the other hand's grasp frame.
 *
 * @param xyz_rpy Offset of the hand the generator describes, as for applyGripperOffset().
 * @return The other hand's offset, metres then radians.
 */
[[nodiscard]] std::array<double, 6> mirroredGripperOffset(const std::array<double, 6>& xyz_rpy);

/**
 * @brief Angle between a generated grasp's approach axis and straight down, in radians.
 *
 * The approach is the gripper frame's +z. Zero reaches straight down; past a right angle the hand
 * comes up from underneath, which on a table means through it.
 *
 * @param generated Expressed in a z-up frame.
 */
[[nodiscard]] double approachTiltRad(const geometry_msgs::msg::Pose& generated);

/**
 * @brief The approach axis of a generated grasp, as a unit vector in the frame the pose is
 *        expressed in.
 */
[[nodiscard]] std::array<double, 3> approachAxis(const geometry_msgs::msg::Pose& generated);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__GRASP_FILTER_HPP_
