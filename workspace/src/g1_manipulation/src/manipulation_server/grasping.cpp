/**
 * @file grasping.cpp
 * @brief Choosing a grasp: the top-down goal, generated candidates, and the plan drawn in RViz.
 */

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <chrono>
#include <format>
#include <memory>
#include <string>
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
constexpr double kGraspAxisLengthM      = 0.05;
constexpr double kLabelHeightM          = 0.02;
constexpr double kLabelLiftM            = 0.03;

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

std::optional<G1ManipulationServer::GraspPlan> G1ManipulationServer::chooseGrasp(
    const Detection& detection, const geometry_msgs::msg::Pose& object_pose, const ArmContext& arm,
    std::string& why, std::vector<ConsideredGrasp>* verdicts)
{
    if (grasp_source_ != "generated")
    {
        GraspPlan plan;
        plan.grasp    = graspFrameGoal(object_pose, detection.bbox.size.z, arm);
        plan.pregrasp = plan.grasp;
        plan.pregrasp.position.z += approach_height_m_;
        plan.origin = "top-down";
        return plan;
    }

    const auto response = requestGrasps(idOf(detection), arm, why);
    if (!response)
    {
        return std::nullopt;
    }
    MoveGroup* group = groupFor(arm.arm_group);
    if (group == nullptr)
    {
        why = "no planning group called " + arm.arm_group;
        return std::nullopt;
    }
    const moveit::core::RobotStatePtr state = group->getCurrentState();
    if (state == nullptr)
    {
        why = "no current state to solve a grasp against";
        return std::nullopt;
    }
    const moveit::core::JointModelGroup* jmg = state->getJointModelGroup(arm.arm_group);
    // One scratch state for every candidate: a RobotState copy allocates per joint and link.
    moveit::core::RobotState attempt(*state);

    const auto record =
        [verdicts](const geometry_msgs::msg::Pose& pose, ConsideredGrasp::Verdict verdict) {
            if (verdicts != nullptr)
            {
                verdicts->push_back({ pose, verdict });
            }
        };
    int considered = 0;
    int reachable  = 0;
    for (std::size_t i = 0; i < response->grasps.size() && considered < max_grasp_candidates_; ++i)
    {
        // Sorted best first, so the first one under the bar ends the list.
        if (i < response->scores.size() && response->scores[i] < min_grasp_score_)
        {
            break;
        }
        ++considered;

        const auto in_planning_frame =
            toPlanningFrame(response->grasps[i], response->header.frame_id);
        if (!in_planning_frame)
        {
            continue;
        }
        // The generator's +z is the approach; past the limit the hand reaches up through the
        // surface.
        if (approachTiltRad(*in_planning_frame) > max_approach_tilt_rad_)
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kTilted);
            continue;
        }

        const geometry_msgs::msg::Pose goal =
            applyGripperOffset(*in_planning_frame, graspgen_offset_);
        // Reset per solve: a failed setFromIK leaves the state where it gave up.
        attempt = *state;
        if (!attempt.setFromIK(jmg, goal, arm.grasp_frame, ik_timeout_s_))
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kUnreachable);
            continue;
        }
        ++reachable;
        record(*in_planning_frame, ConsideredGrasp::Verdict::kChosen);

        // Back along the approach, not straight up: a side grasp has a ceiling above it.
        const std::array<double, 3> axis = approachAxis(*in_planning_frame);
        GraspPlan                   plan;
        plan.grasp    = goal;
        plan.pregrasp = goal;
        plan.pregrasp.position.x -= axis[0] * approach_standoff_m_;
        plan.pregrasp.position.y -= axis[1] * approach_standoff_m_;
        plan.pregrasp.position.z -= axis[2] * approach_standoff_m_;
        plan.origin = std::format(
            "generated candidate {} of {}, score {:.2f}",
            i,
            response->grasps.size(),
            i < response->scores.size() ? response->scores[i] : 0.0F);
        return plan;
    }

    why = std::format(
        "none of the {} candidates considered were usable ({} reachable) out of {} offered",
        considered,
        reachable,
        response->grasps.size());
    return std::nullopt;
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
        const bool tilted = candidate.verdict == ConsideredGrasp::Verdict::kTilted;
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
