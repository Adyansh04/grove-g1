#ifndef G1_MANIPULATION__HAND_CONTACT_HPP_
#define G1_MANIPULATION__HAND_CONTACT_HPP_

/**
 * @file hand_contact.hpp
 * @brief Exempting a hand from collision checking while it is holding something.
 *
 * Shared by the planned and the learned skills, so the exempt links and the edit are defined once.
 */

#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace g1_manipulation
{

/**
 * @brief The links that unavoidably enter occupied space during a grasp.
 *
 * The hand group, the palm and all three wrist links; roll is the one that reaches.
 *
 * @param side "left" or "right".
 * @return Empty if the model has no hand group for that side, an SRDF mismatch.
 */
[[nodiscard]] std::vector<std::string>
handContactLinks(const moveit::core::RobotModel& model, const std::string& side);

/**
 * @brief Sets or clears the exemption in an allowed-collision matrix.
 *
 * Touchables the matrix lacks are appended as a full row and column; links it lacks are skipped.
 * Touchables are exempt from each other too: a lifted object drags through the surface's voxels.
 *
 * @param include_links false exempts the touchables from each other only, leaving the hand and
 *        wrist checked, which is what a carried object needs against the voxels it casts.
 */
void editHandContact(
    moveit_msgs::msg::AllowedCollisionMatrix& acm, const std::vector<std::string>& links,
    const std::vector<std::string>& touchables, bool allowed, bool include_links);

/**
 * @brief Reads the live matrix, edits it, and applies it back.
 *
 * ApplyPlanningScene replaces the whole matrix, so sending only these entries would drop the
 * SRDF's self-collision rules. Blocks without spinning, so never call it from an executor thread.
 *
 * @return false if either service did not answer or the apply failed, leaving the exemption
 *         neither applied nor restored. A failed restore leaves the scene blind to the octomap.
 */
[[nodiscard]] bool applyHandContact(
    const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr&   get_scene,
    const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr& apply_scene,
    const rclcpp::Logger& logger, const std::vector<std::string>& links,
    const std::vector<std::string>& touchables, bool allowed, bool include_links);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__HAND_CONTACT_HPP_
