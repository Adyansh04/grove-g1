#ifndef G1_MANIPULATION__HAND_CONTACT_HPP_
#define G1_MANIPULATION__HAND_CONTACT_HPP_

/**
 * @file hand_contact.hpp
 * @brief Exempting a hand from collision checking while it is holding something.
 *
 * Shared by every skill that closes a hand, planned or learned, so which links are exempt and
 * how the matrix is edited are each defined once.
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
 * All three wrist links, not just pitch and yaw: roll is the one that reaches during a grasp.
 *
 * @param side "left" or "right".
 * @return Empty if the model has no hand group for that side, which is an SRDF mismatch.
 */
[[nodiscard]] std::vector<std::string>
handContactLinks(const moveit::core::RobotModel& model, const std::string& side);

/**
 * @brief Sets or clears the exemption in an allowed-collision matrix.
 *
 * Names the matrix has not seen are appended as a full row and column first. The touchables are
 * exempted from each other too: lifting an object out of a surface drags it through its voxels.
 *
 * @param include_links false exempts the touchables from each other only, leaving the hand and
 *        wrist collision-checked, which is what carrying an object over a surface wants.
 */
void editHandContact(
    moveit_msgs::msg::AllowedCollisionMatrix& acm, const std::vector<std::string>& links,
    const std::vector<std::string>& touchables, bool allowed, bool include_links);

/**
 * @brief Reads the live matrix, edits it, and applies it back.
 *
 * Read-modify-write because ApplyPlanningScene replaces the whole matrix rather than merging:
 * sending only our entries would drop every self-collision rule the SRDF set up.
 *
 * Waited on WITHOUT spinning, so never call this from the thread owning the node's executor.
 *
 * @return false if either service did not answer, leaving the exemption neither applied nor
 *         restored. A failed restore leaves the scene blinded to the octomap.
 */
[[nodiscard]] bool applyHandContact(
    const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr&   get_scene,
    const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr& apply_scene,
    const rclcpp::Logger& logger, const std::vector<std::string>& links,
    const std::vector<std::string>& touchables, bool allowed, bool include_links);

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__HAND_CONTACT_HPP_
