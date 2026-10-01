#ifndef G1_ORCHESTRATION__MISSION_PREVIEW_HPP_
#define G1_ORCHESTRATION__MISSION_PREVIEW_HPP_

/**
 * @file mission_preview.hpp
 * @brief Where a mission would take the base, step by step, before anything moves: the
 *        PreviewMission service.
 *
 * Each step starts where the one before ends, from the robot's pose now. A walk to a pose or to a
 * target gets the path Nav2's planner finds: only the planner runs, nothing reaches the
 * controller. A straight walk and a turn in place end where their arguments say. A step whose end
 * is decided by what the robot sees as it goes (an approach, exploring) says so, and later steps
 * start from where it was headed.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <canopy_msgs/srv/get_approach_pose.hpp>
#include <memory>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nervros_interfaces/msg/step_preview.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

#include "g1_orchestration/mission_validator.hpp"
#include "g1_orchestration/port_types.hpp"

namespace g1_orchestration
{

class MissionPreview
{
public:
    struct Params
    {
        std::string planner_action;
        std::string approach_service;
        std::string map_frame;
        std::string base_frame;
        /// Budget for each planner and world-model call.
        double timeout_s = 5.0;
    };

    MissionPreview(rclcpp::Node& node, Params params, const rclcpp::CallbackGroup::SharedPtr& group);

    /**
     * @brief The preview of @p steps; blocks on the planner, so call it from a callback group
     *        other than the one given to the constructor.
     * @return The steps, or why there is no preview at all (no robot pose).
     */
    [[nodiscard]] std::vector<nervros_interfaces::msg::StepPreview>
    preview(const std::vector<MissionStep>& steps, std::string& why);

    /// Where a straight walk of @p metres (negative is backward) ends.
    [[nodiscard]] static Station walkEnd(const Station& from, double metres);
    /// Where a turn in place by @p degrees (positive is left) ends.
    [[nodiscard]] static Station turnEnd(const Station& from, double degrees);

private:
    [[nodiscard]] std::optional<Station> robotPose(std::string& why) const;
    [[nodiscard]] std::optional<Station> approachPose(const std::string& target, std::string& why);
    [[nodiscard]] std::optional<nav_msgs::msg::Path>
    plan(const Station& from, const Station& to, std::string& why);
    [[nodiscard]] geometry_msgs::msg::PoseStamped stamped(const Station& station) const;

    Params                                                                 params_;
    rclcpp::Clock::SharedPtr                                               clock_;
    tf2_ros::Buffer                                                        buffer_;
    tf2_ros::TransformListener                                             listener_;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr planner_;
    rclcpp::Client<canopy_msgs::srv::GetApproachPose>::SharedPtr           approach_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__MISSION_PREVIEW_HPP_
