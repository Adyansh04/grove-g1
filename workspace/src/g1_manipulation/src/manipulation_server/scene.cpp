/**
 * @file scene.cpp
 * @brief The planning scene: collision objects, the hand's allowed contacts, and the octomap.
 */

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "g1_manipulation/g1_manipulation_server_node.hpp"
#include "g1_manipulation/hand_contact.hpp"

namespace g1_manipulation
{

namespace
{

// MoveIt treats a zero-extent primitive as degenerate and silently drops the attach.
constexpr double kMinPrimitiveExtent = 0.005;

constexpr auto kClearOctomapTimeout = std::chrono::seconds(2);

}  // namespace

void G1ManipulationServer::setHandContact(
    const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
    bool include_links)
{
    if (!allowHandContact(arm, touchables, allowed, include_links))
    {
        RCLCPP_ERROR(
            get_logger(),
            "collision exemption %s failed; the planning scene may be left %s",
            allowed ? "apply" : "restore",
            allowed ? "unchanged" : "blinded to the octomap");
    }
}

bool G1ManipulationServer::allowHandContact(
    const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
    bool include_links)
{
    MoveGroup* hand = groupFor(arm.hand_group);
    if (hand == nullptr)
    {
        return false;
    }
    const std::string              side  = arm.is_left ? "left" : "right";
    const std::vector<std::string> links = handContactLinks(*hand->getRobotModel(), side);
    if (links.empty() || !applyHandContact(
                             get_scene_,
                             apply_scene_,
                             get_logger(),
                             links,
                             touchables,
                             allowed,
                             include_links))
    {
        return false;
    }
    RCLCPP_INFO(
        get_logger(),
        "%s contact between the %s hand and %zu object(s)",
        allowed ? "allowing" : "restoring",
        side.c_str(),
        touchables.size());
    return true;
}

moveit_msgs::msg::CollisionObject G1ManipulationServer::publishCollisionObject(
    const Detection& detection, const geometry_msgs::msg::Pose& in_planning_frame)
{
    moveit_msgs::msg::CollisionObject object;
    object.id              = idOf(detection);
    object.header.frame_id = planning_frame_;

    // A box whatever the real shape: the planner only needs a conservative volume.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type       = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { std::max(detection.bbox.size.x, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.y, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.z, kMinPrimitiveExtent) };

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(in_planning_frame);
    // ADD on an existing id replaces it.
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_.applyCollisionObjects({ object });
    return object;
}

void G1ManipulationServer::removeFromScene(const std::string& object_id)
{
    moveit_msgs::msg::AttachedCollisionObject detach;
    detach.object.id        = object_id;
    detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    // A no-op when nothing is attached under that id. Both calls wait for move_group, so the
    // world copy the detach creates exists by the time the second one removes it.
    planning_scene_.applyAttachedCollisionObject(detach);
    planning_scene_.applyCollisionObject(detach.object);
}

geometry_msgs::msg::Pose topGraspGoal(
    const geometry_msgs::msg::Pose& object_pose, double object_height_m, double depth_below_top_m,
    double min_grip_height_m, const std::vector<double>& rpy, bool is_left)
{
    geometry_msgs::msg::Pose goal;
    goal.position = object_pose.position;

    const double top    = object_pose.position.z + (0.5 * object_height_m);
    const double bottom = object_pose.position.z - (0.5 * object_height_m);
    goal.position.z     = std::max(top - depth_below_top_m, bottom + min_grip_height_m);

    // The hands close in opposite directions, so the roll that points the closing axis flips.
    tf2::Quaternion rotation;
    rotation.setRPY((is_left ? -1.0 : 1.0) * rpy[0], rpy[1], rpy[2]);
    goal.orientation = tf2::toMsg(rotation);
    return goal;
}

void G1ManipulationServer::clearOctomapKeeping(
    const ArmContext& arm, const std::vector<std::string>& touchables)
{
    if (!clear_octomap_->wait_for_service(std::chrono::milliseconds(200)))
    {
        RCLCPP_WARN(get_logger(), "no /clear_octomap; planning against whatever the map holds");
    }
    else
    {
        // Waited on, because the next plan runs against this map.
        auto cleared =
            clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        if (cleared.wait_for(kClearOctomapTimeout) != std::future_status::ready)
        {
            RCLCPP_WARN(
                get_logger(),
                "/clear_octomap did not answer; the map may still hold voxels");
        }
        // The clear takes the surfaces as well as the stale voxels; one sensor update restores
        // what the camera can see.
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(octomap_rebuild_wait_s_)));
    }
    setHandContact(arm, touchables, true);
}

}  // namespace g1_manipulation
