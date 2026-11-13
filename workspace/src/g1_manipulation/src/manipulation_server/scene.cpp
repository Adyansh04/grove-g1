/**
 * @file scene.cpp
 * @brief The planning scene: collision objects, the hand's allowed contacts, and the octomap.
 */

#include <geometric_shapes/shapes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <memory>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
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
// m. How much a pick's support slab is shortened at a time to clear the arm.
constexpr double kSupportShrinkStepM = 0.02;

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

    // A box whatever the real shape: the planner only needs a conservative volume. Padded
    // sideways, since perception measures short and the arm lags its plan; not up, or the carried
    // box would sink into the table the place stands it on.
    const double                    pad = 2.0 * scene_object_padding_m_;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type       = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = { std::max(detection.bbox.size.x + pad, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.y + pad, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.z, kMinPrimitiveExtent) };

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(in_planning_frame);
    // ADD on an existing id replaces it.
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_.applyCollisionObjects({ object });
    return object;
}

void G1ManipulationServer::publishSupport(
    const ArmContext& arm, const geometry_msgs::msg::Pose& object_pose, double object_height_m)
{
    const double ahead = pick_support_reach_m_[1];
    double       back  = pick_support_reach_m_[0];
    const auto   slab  = [&] {
        geometry_msgs::msg::Pose at;
        at.position.x = object_pose.position.x + (0.5 * (ahead - back));
        at.position.y = object_pose.position.y;
        at.position.z =
            object_pose.position.z - (0.5 * object_height_m) - (0.5 * pick_support_thickness_m_);
        at.orientation.w = 1.0;
        return at;
    };
    // Never round where the arm already is, or no plan could start: shortened toward the robot
    // until the arm and hand clear it, as a hand left low beside the table's edge needs.
    MoveGroup* group = groupFor(arm.arm_group);
    if (const moveit::core::RobotStatePtr now =
            group != nullptr ? group->getCurrentState(1.0) : nullptr;
        now != nullptr)
    {
        planning_scene::PlanningScene               scratch(now->getRobotModel());
        collision_detection::AllowedCollisionMatrix acm = scratch.getAllowedCollisionMatrix();
        acm.setDefaultEntry(kPickSupportId, true);
        for (const std::string& name : { arm.arm_group, arm.hand_group })
        {
            if (const auto* jmg = now->getJointModelGroup(name))
            {
                for (const std::string& link : jmg->getLinkModelNamesWithCollisionGeometry())
                {
                    acm.setEntry(kPickSupportId, link, false);
                }
            }
        }
        const double full = back;
        for (int step = 0; full - (step * kSupportShrinkStepM) > 0.0; ++step)
        {
            back                                = full - (step * kSupportShrinkStepM);
            const geometry_msgs::msg::Point& p  = slab().position;
            Eigen::Isometry3d                at = Eigen::Isometry3d::Identity();
            at.translation()                    = Eigen::Vector3d(p.x, p.y, p.z);
            scratch.getWorldNonConst()->removeObject(kPickSupportId);
            scratch.getWorldNonConst()->addToObject(
                kPickSupportId,
                std::make_shared<const shapes::Box>(
                    back + ahead,
                    2.0 * pick_support_half_width_m_,
                    pick_support_thickness_m_),
                at);
            collision_detection::CollisionRequest request;
            collision_detection::CollisionResult  result;
            scratch.getCollisionEnv()->checkRobotCollision(request, result, *now, acm);
            if (!result.collision)
            {
                break;
            }
            back = 0.0;
        }
    }
    moveit_msgs::msg::CollisionObject support;
    support.id              = kPickSupportId;
    support.header.frame_id = planning_frame_;
    shape_msgs::msg::SolidPrimitive box;
    box.type       = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { back + ahead, 2.0 * pick_support_half_width_m_, pick_support_thickness_m_ };
    support.primitives.push_back(box);
    support.primitive_poses.push_back(slab());
    support.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene_.applyCollisionObjects({ support });
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

void G1ManipulationServer::clearOctomap()
{
    if (!clear_octomap_->wait_for_service(std::chrono::milliseconds(200)))
    {
        RCLCPP_WARN(get_logger(), "no /clear_octomap; planning against whatever the map holds");
        return;
    }
    // Waited on, because the next plan runs against this map.
    auto cleared =
        clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    if (cleared.wait_for(kClearOctomapTimeout) != std::future_status::ready)
    {
        RCLCPP_WARN(get_logger(), "/clear_octomap did not answer; the map may still hold voxels");
    }
    // The clear takes the surfaces as well as the stale voxels; one sensor update restores
    // what the camera can see.
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(octomap_rebuild_wait_s_)));
}

void G1ManipulationServer::clearOctomapKeeping(
    const ArmContext& arm, const std::vector<std::string>& touchables)
{
    clearOctomap();
    setHandContact(arm, touchables, true);
}

}  // namespace g1_manipulation
