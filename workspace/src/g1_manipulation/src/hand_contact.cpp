/**
 * @file hand_contact.cpp
 * @brief Allowed-collision-matrix edits for a hand that is holding something.
 */

#include "g1_manipulation/hand_contact.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

namespace g1_manipulation
{

namespace
{

// move_group answers both off its own scene, so this long a wait means wedged, not busy.
constexpr std::chrono::seconds kSceneServiceTimeout{ 5 };

std::size_t indexOf(const moveit_msgs::msg::AllowedCollisionMatrix& acm, const std::string& name)
{
    return static_cast<std::size_t>(
        std::find(acm.entry_names.begin(), acm.entry_names.end(), name) - acm.entry_names.begin());
}

void ensureEntry(moveit_msgs::msg::AllowedCollisionMatrix& acm, const std::string& name)
{
    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), name) != acm.entry_names.end())
    {
        return;
    }
    acm.entry_names.push_back(name);
    for (moveit_msgs::msg::AllowedCollisionEntry& row : acm.entry_values)
    {
        row.enabled.push_back(false);
    }
    moveit_msgs::msg::AllowedCollisionEntry row;
    row.enabled.assign(acm.entry_names.size(), false);
    acm.entry_values.push_back(row);
}

}  // namespace

std::vector<std::string>
handContactLinks(const moveit::core::RobotModel& model, const std::string& side)
{
    const moveit::core::JointModelGroup* hand = model.getJointModelGroup(side + "_hand");
    if (hand == nullptr)
    {
        return {};
    }
    std::vector<std::string> links = hand->getLinkModelNames();
    links.push_back(side + "_hand_palm_link");
    links.push_back(side + "_wrist_pitch_link");
    links.push_back(side + "_wrist_yaw_link");
    links.push_back(side + "_wrist_roll_link");
    return links;
}

void editHandContact(
    moveit_msgs::msg::AllowedCollisionMatrix& acm, const std::vector<std::string>& links,
    const std::vector<std::string>& touchables, bool allowed, bool include_links)
{
    for (const std::string& touchable : touchables)
    {
        ensureEntry(acm, touchable);
    }

    for (std::size_t i = 0; i < touchables.size(); ++i)
    {
        for (std::size_t j = i + 1; j < touchables.size(); ++j)
        {
            const std::size_t a            = indexOf(acm, touchables[i]);
            const std::size_t b            = indexOf(acm, touchables[j]);
            acm.entry_values[a].enabled[b] = allowed;
            acm.entry_values[b].enabled[a] = allowed;
        }
    }

    if (!include_links)
    {
        return;
    }
    for (const std::string& touchable : touchables)
    {
        const std::size_t other = indexOf(acm, touchable);
        for (const std::string& link : links)
        {
            const auto it = std::find(acm.entry_names.begin(), acm.entry_names.end(), link);
            if (it == acm.entry_names.end())
            {
                continue;
            }
            const auto index = static_cast<std::size_t>(it - acm.entry_names.begin());
            acm.entry_values[index].enabled[other] = allowed;
            acm.entry_values[other].enabled[index] = allowed;
        }
    }
}

bool applyHandContact(
    const rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr&   get_scene,
    const rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr& apply_scene,
    const rclcpp::Logger& logger, const std::vector<std::string>& links,
    const std::vector<std::string>& touchables, bool allowed, bool include_links)
{
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_scene->async_send_request(request);
    if (future.wait_for(kSceneServiceTimeout) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger, "/get_planning_scene did not answer");
        return false;
    }

    moveit_msgs::msg::AllowedCollisionMatrix acm = future.get()->scene.allowed_collision_matrix;
    editHandContact(acm, links, touchables, allowed, include_links);

    auto apply           = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply->scene.is_diff = true;
    apply->scene.allowed_collision_matrix = acm;
    auto applied                          = apply_scene->async_send_request(apply);
    if (applied.wait_for(kSceneServiceTimeout) != std::future_status::ready)
    {
        RCLCPP_ERROR(logger, "/apply_planning_scene did not answer");
        return false;
    }
    return applied.get()->success;
}

}  // namespace g1_manipulation
