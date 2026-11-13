/**
 * @file grasping.cpp
 * @brief Choosing a grasp: the top-down goal, generated candidates, and the plan drawn in RViz.
 */

#include <geometric_shapes/shapes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <memory>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_matrix.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <string>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

#include "g1_manipulation/g1_manipulation_server_node.hpp"
#include "g1_manipulation/grasp_filter.hpp"

namespace g1_manipulation
{

namespace
{

constexpr double kCandidateArrowLengthM = 0.06;
// What the object rests on, in the scratch scene that checks a hand's clearance of it: a slab under
// the object's base, as wide as a hand reaches round it. Only the hand and the wrist are checked
// against it, since the rest of the arm hangs beside the body, not over the table.
constexpr const char* kSupportId     = "support";
constexpr double      kSupportSideM  = 0.6;
constexpr double      kSupportDepthM = 0.2;
// m. Padding on the other objects in that scene, and how near a detection's centre must lie to the
// target's to be the target itself, listed again under its bare name.
constexpr double kNeighbourPaddingM = 0.01;
constexpr double kSameObjectM       = 0.02;
// Shares of the deepest close checked: fingers sweep out along the approach before curling back.
constexpr std::array<double, 5> kCloseShares = { 0.0, 0.25, 0.5, 0.75, 1.0 };
// m. Steps a grasp that would strike is backed off along its approach, up to
// grasp_max_backoff_m.
constexpr double kBackoffStepM     = 0.005;
constexpr double kGraspAxisLengthM = 0.05;
constexpr double kLabelHeightM     = 0.02;
constexpr double kLabelLiftM       = 0.03;

geometry_msgs::msg::Point offsetBy(const geometry_msgs::msg::Point& from, const tf2::Vector3& by)
{
    geometry_msgs::msg::Point to;
    to.x = from.x + by.x();
    to.y = from.y + by.y();
    to.z = from.z + by.z();
    return to;
}

visualization_msgs::msg::Marker arrow(
    const std::string& frame, const std::string& ns, int id, const geometry_msgs::msg::Point& tail,
    const geometry_msgs::msg::Point& head, float r, float g, float b)
{
    visualization_msgs::msg::Marker marker;
    // Unstamped, so RViz draws it at the latest transform.
    marker.header.frame_id = frame;
    marker.ns              = ns;
    marker.id              = id;
    marker.type            = visualization_msgs::msg::Marker::ARROW;
    marker.action          = visualization_msgs::msg::Marker::ADD;
    marker.points          = { tail, head };
    marker.scale.x         = 0.004;
    marker.scale.y         = 0.010;
    marker.scale.z         = 0.012;
    marker.color.r         = r;
    marker.color.g         = g;
    marker.color.b         = b;
    marker.color.a         = 0.9F;
    return marker;
}

}  // namespace

geometry_msgs::msg::Pose G1ManipulationServer::graspFrameGoal(
    const geometry_msgs::msg::Pose& object_pose, double object_height_m, const ArmContext& arm) const
{
    return topGraspGoal(
        object_pose,
        object_height_m,
        grasp_depth_below_top_m_,
        min_grip_height_m_,
        grasp_rpy_,
        arm.is_left);
}

std::optional<geometry_msgs::msg::Pose> G1ManipulationServer::heldPlaceGoal(
    MoveGroup& group, const ArmContext& arm, const moveit_msgs::msg::AttachedCollisionObject& held,
    const geometry_msgs::msg::Point& object_at)
{
    if (held.object.header.frame_id != arm.palm_link || held.object.primitive_poses.empty())
    {
        return std::nullopt;
    }
    tf2::Transform palm;
    tf2::Transform origin;
    tf2::Transform box;
    tf2::fromMsg(group.getCurrentPose(arm.palm_link).pose, palm);
    tf2::fromMsg(held.object.pose, origin);
    tf2::fromMsg(held.object.primitive_poses.front(), box);
    const tf2::Vector3       now  = (palm * origin * box).getOrigin();
    geometry_msgs::msg::Pose goal = group.getCurrentPose(arm.grasp_frame).pose;
    goal.position.x += object_at.x - now.x();
    goal.position.y += object_at.y - now.y();
    goal.position.z += object_at.z - now.z();
    return goal;
}

std::optional<double> G1ManipulationServer::handBelowHeldM(
    MoveGroup& group, const ArmContext& arm, const moveit_msgs::msg::AttachedCollisionObject& held,
    double held_height_m)
{
    const moveit::core::RobotStatePtr state = group.getCurrentState(1.0);
    if (held.object.header.frame_id != arm.palm_link || held.object.primitive_poses.empty() ||
        state == nullptr)
    {
        return std::nullopt;
    }
    const moveit::core::JointModelGroup* hand = state->getJointModelGroup(arm.hand_group);
    if (hand == nullptr)
    {
        return std::nullopt;
    }
    tf2::Transform palm;
    tf2::Transform origin;
    tf2::Transform box;
    tf2::fromMsg(group.getCurrentPose(arm.palm_link).pose, palm);
    tf2::fromMsg(held.object.pose, origin);
    tf2::fromMsg(held.object.primitive_poses.front(), box);
    const double base_z = (palm * origin * box).getOrigin().z() - (0.5 * held_height_m);
    // Each link's collision box, corner by corner: a few millimetres lower than its mesh at most.
    // Through the release too: fingers curled under an object sweep lower as they open.
    std::map<std::string, double> open;
    hand->getVariableDefaultPositions("pinch_ready", open);
    std::map<std::string, double> grip;
    for (const auto& [joint, value] : open)
    {
        grip[joint] = state->getVariablePosition(joint);
    }
    double lowest_z = std::numeric_limits<double>::infinity();
    for (const double share : kCloseShares)
    {
        for (const auto& [joint, from] : grip)
        {
            state->setVariablePosition(joint, from + (share * (open[joint] - from)));
        }
        state->update();
        for (const moveit::core::LinkModel* link : hand->getLinkModels())
        {
            if (link->getShapes().empty())
            {
                continue;
            }
            const Eigen::Vector3d   half = 0.5 * link->getShapeExtentsAtOrigin();
            const Eigen::Isometry3d frame =
                state->getGlobalLinkTransform(link) *
                Eigen::Translation3d(link->getCenteredBoundingBoxOffset());
            for (int corner = 0; corner < 8; ++corner)
            {
                const Eigen::Vector3d at(
                    (corner & 1) != 0 ? half.x() : -half.x(),
                    (corner & 2) != 0 ? half.y() : -half.y(),
                    (corner & 4) != 0 ? half.z() : -half.z());
                lowest_z = std::min(lowest_z, (frame * at).z());
            }
        }
    }
    if (!std::isfinite(lowest_z))
    {
        return std::nullopt;
    }
    return base_z - lowest_z;
}

std::optional<g1_msgs::srv::GenerateGrasps::Response> G1ManipulationServer::requestGrasps(
    const std::string& object_id, const ArmContext& arm, std::string& why)
{
    if (!grasps_->wait_for_service(std::chrono::milliseconds(500)))
    {
        why = std::string("no grasp generator is serving ") + grasps_->get_service_name();
        return std::nullopt;
    }
    auto request       = std::make_shared<g1_msgs::srv::GenerateGrasps::Request>();
    request->object_id = object_id;
    request->hand      = arm.is_left ? "left" : "right";

    // Waited on, not spun: re-entering the executor from a goal thread deadlocks.
    auto future = grasps_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(grasp_timeout_s_)) !=
        std::future_status::ready)
    {
        why = std::format("the grasp generator did not answer within {:.1f} s", grasp_timeout_s_);
        return std::nullopt;
    }
    const auto response = future.get();
    if (!response->ok)
    {
        why = "the grasp generator refused: " + response->message;
        return std::nullopt;
    }
    if (response->grasps.empty())
    {
        why = "the grasp generator found no grasp on '" + object_id + "'";
        return std::nullopt;
    }
    return std::move(*response);
}

geometry_msgs::msg::Point placeOffset(
    const geometry_msgs::msg::Pose& surface, const geometry_msgs::msg::Vector3& size,
    const geometry_msgs::msg::Point& preferred, double margin_m, double container_m)
{
    geometry_msgs::msg::Point offset;
    if (std::max(size.x, size.y) <= container_m)
    {
        return offset;
    }
    const double yaw = tf2::getYaw(surface.orientation);
    const double dx  = preferred.x - surface.position.x;
    const double dy  = preferred.y - surface.position.y;
    // Into the surface's frame, then kept inside its footprint less the margin.
    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double half_x  = std::max(0.0, 0.5 * size.x - margin_m);
    const double half_y  = std::max(0.0, 0.5 * size.y - margin_m);
    offset.x             = std::clamp(local_x, -half_x, half_x);
    offset.y             = std::clamp(local_y, -half_y, half_y);
    return offset;
}

geometry_msgs::msg::Pose
atOffset(const geometry_msgs::msg::Pose& surface, const geometry_msgs::msg::Point& offset)
{
    const double             yaw = tf2::getYaw(surface.orientation);
    geometry_msgs::msg::Pose out = surface;
    out.position.x += std::cos(yaw) * offset.x - std::sin(yaw) * offset.y;
    out.position.y += std::sin(yaw) * offset.x + std::cos(yaw) * offset.y;
    return out;
}

std::vector<G1ManipulationServer::GraspPlan> G1ManipulationServer::chooseGrasps(
    const Detection& detection, const geometry_msgs::msg::Pose& object_pose, const ArmContext& arm,
    std::string& why, std::vector<ConsideredGrasp>* verdicts)
{
    // GraspGenX describes one hand; the other keeps the grasp from the object's box.
    const bool generated =
        grasp_source_ == "generated" &&
        std::ranges::find(generated_grasp_arms_, arm.is_left ? "left" : "right") !=
            generated_grasp_arms_.end();
    if (!generated)
    {
        GraspPlan plan;
        plan.grasp    = graspFrameGoal(object_pose, detection.bbox.size.z, arm);
        plan.pregrasp = plan.grasp;
        plan.pregrasp.position.z += approach_height_m_;
        plan.origin = "top-down";
        return { plan };
    }

    const auto response = requestGrasps(idOf(detection), arm, why);
    if (!response)
    {
        return {};
    }
    MoveGroup* group = groupFor(arm.arm_group);
    if (group == nullptr)
    {
        why = "no planning group called " + arm.arm_group;
        return {};
    }
    const moveit::core::RobotStatePtr state = group->getCurrentState();
    if (state == nullptr)
    {
        why = "no current state to solve a grasp against";
        return {};
    }
    const moveit::core::JointModelGroup* jmg      = state->getJointModelGroup(arm.arm_group);
    const moveit::core::JointModelGroup* hand_jmg = state->getJointModelGroup(arm.hand_group);
    std::map<std::string, double>        open;
    std::map<std::string, double>        closed;
    if (hand_jmg == nullptr || !hand_jmg->getVariableDefaultPositions("open", open) ||
        !hand_jmg->getVariableDefaultPositions("closed", closed))
    {
        why = "no '" + arm.hand_group + "' group with open and closed states";
        return {};
    }
    // No generator sees what the object stands on: each candidate's wrist and hand, through the
    // close down to the object's narrowest side, must clear its base by the margin.
    planning_scene::PlanningScene support(state->getRobotModel());
    Eigen::Isometry3d             slab = Eigen::Isometry3d::Identity();
    slab.translation()                 = Eigen::Vector3d(
        object_pose.position.x,
        object_pose.position.y,
        object_pose.position.z - (0.5 * detection.bbox.size.z) + grasp_surface_clearance_m_ -
            (0.5 * kSupportDepthM));
    support.getWorldNonConst()->addToObject(
        kSupportId,
        std::make_shared<const shapes::Box>(kSupportSideM, kSupportSideM, kSupportDepthM),
        slab);
    std::vector<std::string> guarded = hand_jmg->getLinkModelNamesWithCollisionGeometry();
    for (const std::string& link : jmg->getLinkModelNamesWithCollisionGeometry())
    {
        if (link.find("wrist") != std::string::npos)
        {
            guarded.push_back(link);
        }
    }
    collision_detection::AllowedCollisionMatrix acm   = support.getAllowedCollisionMatrix();
    const auto                                  guard = [&acm, &guarded](const std::string& body) {
        acm.setDefaultEntry(body, true);
        for (const std::string& link : guarded)
        {
            acm.setEntry(body, link, false);
        }
    };
    guard(kSupportId);
    // The other objects in view, so the hand neither lands on nor closes into a neighbour. Wider
    // ones are surfaces, which the slab stands for.
    vision_msgs::msg::Detection3DArray seen;
    {
        const std::lock_guard<std::mutex> lock(objects_mutex_);
        seen = objects_;
    }
    int neighbours = 0;
    for (const Detection& other : seen.detections)
    {
        if (other.results.empty() ||
            std::max(other.bbox.size.x, other.bbox.size.y) > place_container_max_m_)
        {
            continue;
        }
        const auto at = toPlanningFrame(other.results.front().pose.pose, seen.header.frame_id);
        if (!at || std::hypot(
                       at->position.x - object_pose.position.x,
                       at->position.y - object_pose.position.y,
                       at->position.z - object_pose.position.z) < kSameObjectM)
        {
            continue;
        }
        Eigen::Isometry3d box = Eigen::Isometry3d::Identity();
        box.translation()     = Eigen::Vector3d(at->position.x, at->position.y, at->position.z);
        box.linear()          = Eigen::Quaterniond(
                           at->orientation.w,
                           at->orientation.x,
                           at->orientation.y,
                           at->orientation.z)
                           .normalized()
                           .toRotationMatrix();
        const std::string name = "neighbour_" + std::to_string(neighbours++);
        support.getWorldNonConst()->addToObject(
            name,
            std::make_shared<const shapes::Box>(
                other.bbox.size.x + (2.0 * kNeighbourPaddingM),
                other.bbox.size.y + (2.0 * kNeighbourPaddingM),
                other.bbox.size.z + (2.0 * kNeighbourPaddingM)),
            box);
        guard(name);
    }
    const double deepest = closeFractions(
                               std::max(detection.bbox.size.x, detection.bbox.size.y),
                               std::min(detection.bbox.size.x, detection.bbox.size.y))
                               .second;
    // What blocked the candidates, so a pick with none left says what to move.
    std::map<std::string, int> struck;
    const auto                 clears = [&](moveit::core::RobotState& candidate) {
        for (const double share : kCloseShares)
        {
            for (const auto& [joint, from] : open)
            {
                if (const auto to = closed.find(joint); to != closed.end())
                {
                    candidate.setVariablePosition(
                        joint,
                        from + (share * deepest * (to->second - from)));
                }
            }
            candidate.update();
            collision_detection::CollisionRequest request;
            request.contacts = true;
            collision_detection::CollisionResult result;
            support.getCollisionEnv()->checkRobotCollision(request, result, candidate, acm);
            if (result.collision)
            {
                if (!result.contacts.empty())
                {
                    const auto& [first, second] = result.contacts.begin()->first;
                    ++struck[std::format("{} on {}", first, second)];
                }
                return false;
            }
        }
        return true;
    };
    // One scratch state for every candidate: a RobotState copy allocates per joint and link.
    moveit::core::RobotState attempt(*state);

    const auto record =
        [verdicts](const geometry_msgs::msg::Pose& pose, ConsideredGrasp::Verdict verdict) {
            if (verdicts != nullptr)
            {
                verdicts->push_back({ pose, verdict });
            }
        };
    int                    considered = 0;
    int                    reachable  = 0;
    int                    clear      = 0;
    std::vector<GraspPlan> plans;
    // Solvable at the grasp only: a 50 ms solve near the edge of reach misses pregrasps OMPL's
    // own goal sampling finds, so these follow the ones that solve at both ends.
    std::vector<GraspPlan> grasp_only;
    for (std::size_t i = 0; i < response->grasps.size() && considered < max_grasp_candidates_ &&
                            std::cmp_less(plans.size(), grasp_tries_);
         ++i)
    {
        // Sorted best first, so the first one under the bar ends the list.
        if (i < response->scores.size() && response->scores[i] < min_grasp_score_)
        {
            break;
        }
        ++considered;

        const auto in_planning_frame = toPlanningFrame(
            response->grasps[i],
            response->header.frame_id,
            rclcpp::Time(response->header.stamp));
        if (!in_planning_frame)
        {
            continue;
        }
        // The generator's +z is the approach; past the limit the hand reaches up through the
        // surface.
        // Coming back at the robot from beyond the object, the hand reaches round it: rarely
        // reachable, and the reach sweeps the object.
        if (approachTiltRad(*in_planning_frame) > max_approach_tilt_rad_ ||
            approachAxis(*in_planning_frame)[0] < min_approach_forward_)
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kTilted);
            continue;
        }

        // The generator describes the right hand; the left's grasps come back mirrored.
        const geometry_msgs::msg::Pose goal = applyGripperOffset(
            *in_planning_frame,
            arm.is_left ? graspgen_offset_mirrored_ : graspgen_offset_);
        // Back along the approach, not straight up: a side grasp has a ceiling above it.
        const std::array<double, 3> axis = approachAxis(*in_planning_frame);
        const auto backed_off            = [&axis](geometry_msgs::msg::Pose pose, double by_m) {
            pose.position.x -= axis[0] * by_m;
            pose.position.y -= axis[1] * by_m;
            pose.position.z -= axis[2] * by_m;
            return pose;
        };
        GraspPlan plan;
        plan.candidate = *in_planning_frame;
        // A grasp seated a little shallower still holds, where one driven into the table breaks a
        // finger: backed off along its own approach until the hand clears, within the limit.
        bool   solved = false;
        bool   safe   = false;
        double back_m = 0.0;
        for (int step = 0; step * kBackoffStepM <= grasp_max_backoff_m_ + 1e-9; ++step)
        {
            back_m     = step * kBackoffStepM;
            plan.grasp = backed_off(goal, back_m);
            // Reset per solve: a failed setFromIK leaves the state where it gave up.
            attempt = *state;
            if (!attempt.setFromIK(jmg, plan.grasp, arm.grasp_frame, ik_timeout_s_))
            {
                continue;
            }
            solved = true;
            if (clears(attempt))
            {
                safe = true;
                break;
            }
        }
        if (!solved)
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kUnreachable);
            continue;
        }
        ++reachable;
        if (!safe)
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kStrikes);
            continue;
        }
        ++clear;
        plan.pregrasp = backed_off(plan.grasp, approach_standoff_m_);
        plan.origin   = std::format(
            "generated candidate {} of {}, score {:.2f}{}",
            i,
            response->grasps.size(),
            i < response->scores.size() ? response->scores[i] : 0.0F,
            back_m > 0.0 ? std::format(", backed off {:.0f} mm", 1000.0 * back_m) : "");
        attempt = *state;
        if (attempt.setFromIK(jmg, plan.pregrasp, arm.grasp_frame, ik_timeout_s_))
        {
            plans.push_back(std::move(plan));
        }
        else
        {
            grasp_only.push_back(std::move(plan));
        }
    }
    for (GraspPlan& plan : grasp_only)
    {
        if (std::cmp_greater_equal(plans.size(), grasp_tries_))
        {
            break;
        }
        plans.push_back(std::move(plan));
    }
    // The first is the one drawn as chosen; the rest wait in case MoveIt finds no way to it.
    if (!plans.empty())
    {
        record(plans.front().candidate, ConsideredGrasp::Verdict::kChosen);
    }
    if (plans.empty())
    {
        why = std::format(
            "none of the {} candidates considered were usable ({} reachable, {} of those clear "
            "of what the object stands on and its neighbours) out of {} offered",
            considered,
            reachable,
            clear,
            response->grasps.size());
        for (const auto& [pair, count] : struck)
        {
            why += std::format("; {}: {}", pair, count);
        }
    }
    return plans;
}

void G1ManipulationServer::publishGraspPlan(
    const GraspPlan* plan, const std::vector<ConsideredGrasp>& verdicts)
{
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray markers;
    Marker                               clear;
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);

    // Each candidate ends at its gripper frame, along the direction the hand travels.
    int id = 0;
    for (const ConsideredGrasp& candidate : verdicts)
    {
        const std::array<double, 3> axis = approachAxis(candidate.pose);
        const tf2::Vector3          back(
            -axis[0] * kCandidateArrowLengthM,
            -axis[1] * kCandidateArrowLengthM,
            -axis[2] * kCandidateArrowLengthM);
        const bool chosen = candidate.verdict == ConsideredGrasp::Verdict::kChosen;
        const bool tilted = candidate.verdict == ConsideredGrasp::Verdict::kTilted ||
                            candidate.verdict == ConsideredGrasp::Verdict::kStrikes;
        markers.markers.push_back(arrow(
            planning_frame_,
            "candidates",
            id++,
            offsetBy(candidate.pose.position, back),
            candidate.pose.position,
            chosen ? 0.1F : 1.0F,
            chosen ? 0.9F : (tilted ? 0.15F : 0.6F),
            0.15F));
    }

    if (plan != nullptr)
    {
        // After the gripper offset: where the grasp frame is actually sent.
        tf2::Quaternion orientation;
        tf2::fromMsg(plan->grasp.orientation, orientation);
        const tf2::Matrix3x3 rotation(orientation);
        for (int column = 0; column < 3; ++column)
        {
            const tf2::Vector3 axis = rotation.getColumn(column) * kGraspAxisLengthM;
            markers.markers.push_back(arrow(
                planning_frame_,
                "grasp",
                column,
                plan->grasp.position,
                offsetBy(plan->grasp.position, axis),
                column == 0 ? 1.0F : 0.0F,
                column == 1 ? 1.0F : 0.0F,
                column == 2 ? 1.0F : 0.0F));
        }
        markers.markers.push_back(arrow(
            planning_frame_,
            "approach",
            0,
            plan->pregrasp.position,
            plan->grasp.position,
            0.1F,
            0.8F,
            1.0F));

        Marker& label         = markers.markers.emplace_back();
        label.header.frame_id = planning_frame_;
        label.ns              = "label";
        label.type            = Marker::TEXT_VIEW_FACING;
        label.action          = Marker::ADD;
        label.text            = plan->origin;
        label.scale.z         = kLabelHeightM;
        label.pose.position   = plan->pregrasp.position;
        label.pose.position.z += kLabelLiftM;
        label.pose.orientation.w = 1.0;
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
    }
    grasp_plan_pub_->publish(markers);
}

}  // namespace g1_manipulation
