/**
 * @file g1_manipulation_server_node.cpp
 * @brief The pick, place and posture action servers, planned and executed through MoveIt.
 */

#include "g1_manipulation/g1_manipulation_server_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <memory>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace g1_manipulation
{

namespace
{

// A Cartesian fraction at or above this is a line walked to its end.
constexpr double kLineComplete = 0.99;

// Base travel during a place's reach below this is not worth a log line.
constexpr double kReportShiftM = 0.01;

rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

}  // namespace

bool resolveArm(const std::string& arm, ArmContext& out)
{
    if (arm != "left" && arm != "right")
    {
        return false;
    }
    out.arm_group   = arm + "_arm";
    out.hand_group  = arm + "_hand";
    out.palm_link   = arm + "_hand_palm_link";
    out.grasp_frame = arm + "_hand_grasp_frame";
    out.is_left     = arm == "left";
    return true;
}

G1ManipulationServer::G1ManipulationServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_manipulation_server", options)
  , tf_buffer_(get_clock())
  , tf_listener_(tf_buffer_)
{
    declareParameters();

    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) { onObjects(msg); });

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
            const std::lock_guard<std::mutex> lock(joint_states_mutex_);
            joint_states_ = *msg;
        });

    grasps_ =
        create_client<g1_msgs::srv::GenerateGrasps>(get_parameter("grasp_service").as_string());
    if (get_parameter("publish_markers").as_bool())
    {
        // Transient local: published once per pick, and RViz may connect after that.
        grasp_plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/grasp_plan",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }
    get_scene_     = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_scene_   = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
}

G1ManipulationServer::~G1ManipulationServer()
{
    while (goals_running_.load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void G1ManipulationServer::declareParameters()
{
    live_doubles_ = {
        { "approach_height_m", &approach_height_m_ },
        { "place_approach_height_m", &place_approach_height_m_ },
        { "grasp_depth_below_top_m", &grasp_depth_below_top_m_ },
        { "min_grip_height_m", &min_grip_height_m_ },
        { "settle_tolerance_m", &settle_tolerance_m_ },
        { "max_grasp_offset_m", &max_grasp_offset_m_ },
        { "grasp_refresh_max_shift_m", &grasp_refresh_max_shift_m_ },
        { "reaim_clearance_m", &reaim_clearance_m_ },
        { "settle_wait_s", &settle_wait_s_ },
        { "lift_height_m", &lift_height_m_ },
        { "hand_close_s", &hand_close_s_ },
        { "hand_span_open_m", &hand_span_open_m_ },
        { "hand_span_closed_m", &hand_span_closed_m_ },
        { "grip_preload_m", &grip_preload_m_ },
        { "grip_start_margin_m", &grip_start_margin_m_ },
        { "grip_max_width_m", &grip_max_width_m_ },
        { "grip_search_step_m", &grip_search_step_m_ },
        { "grip_search_beyond_m", &grip_search_beyond_m_ },
        { "grip_settled_speed_rad_s", &grip_settled_speed_rad_s_ },
        { "grip_hold_bias_rad", &grip_hold_bias_rad_ },
        { "grip_min_position_error_rad", &grip_min_position_error_rad_ },
        { "grip_min_effort_nm", &grip_min_effort_nm_ },
        { "place_tolerance_m", &place_tolerance_m_ },
        { "place_footprint_margin_m", &place_footprint_margin_m_ },
        { "place_confirm_timeout_s", &place_confirm_timeout_s_ },
        { "sighting_memory_s", &sighting_memory_s_ },
        { "octomap_rebuild_wait_s", &octomap_rebuild_wait_s_ },
        { "planning_time_s", &planning_time_s_ },
        { "grasp_timeout_s", &grasp_timeout_s_ },
        { "min_grasp_score", &min_grasp_score_ },
        { "approach_standoff_m", &approach_standoff_m_ },
        { "ik_timeout_s", &ik_timeout_s_ },
        { "cartesian_step_m", &cartesian_step_m_ },
    };
    live_ints_ = {
        { "settle_attempts", &settle_attempts_ },
        { "lift_attempts", &lift_attempts_ },
        { "planning_attempts", &planning_attempts_ },
        { "max_grasp_candidates", &max_grasp_candidates_ },
    };
    for (const auto& [name, value] : live_doubles_)
    {
        *value = declare_parameter<double>(name, *value);
    }
    for (const auto& [name, value] : live_ints_)
    {
        *value = static_cast<int>(declare_parameter<int>(name, *value));
    }
    grip_check_enabled_ = declare_parameter<bool>("grip_check_enabled", grip_check_enabled_);
    object_timeout_s_   = declare_parameter<double>("object_timeout_ms", 1000.0) / 1000.0;
    max_approach_tilt_rad_ =
        declare_parameter<double>("max_approach_tilt_deg", 75.0) * M_PI / 180.0;

    // Read once, by the constructor or initialize().
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only   = true;
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", velocity_scaling_, fixed);
    grasp_rpy_ = declare_parameter<std::vector<double>>("grasp_rpy", { -M_PI_2, 0.0, 0.0 }, fixed);
    grasp_source_ = declare_parameter<std::string>("grasp_source", "fixed_top_down", fixed);
    declare_parameter<std::string>("grasp_service", "/g1_grasp_engine/generate_grasps", fixed);
    declare_parameter<bool>("publish_markers", false, fixed);
    const std::vector<double> offset = declare_parameter<std::vector<double>>(
        "graspgen_to_grasp_frame_xyz_rpy",
        std::vector<double>(graspgen_offset_.size(), 0.0),
        fixed);
    // Refused rather than padded: a short list read as identity rpy is a silently wrong grasp.
    if (offset.size() != graspgen_offset_.size())
    {
        throw std::invalid_argument(
            "graspgen_to_grasp_frame_xyz_rpy needs 6 numbers, xyz then rpy; got " +
            std::to_string(offset.size()));
    }
    std::ranges::copy(offset, graspgen_offset_.begin());
    if (grasp_rpy_.size() != 3)
    {
        throw std::invalid_argument("grasp_rpy needs exactly 3 entries");
    }

    // Goal threads read the tunables unlocked, so a set is refused while one runs. Goal
    // acceptance shares this callback's group, so busy_ cannot change during the check.
    parameters_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& updated) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = !busy_.load();
            if (!result.successful)
            {
                result.reason = "a goal is running; set tunables between goals";
                return result;
            }
            for (const rclcpp::Parameter& parameter : updated)
            {
                const std::string& name = parameter.get_name();
                if (const auto found = live_doubles_.find(name); found != live_doubles_.end())
                {
                    *found->second = parameter.as_double();
                }
                else if (const auto count = live_ints_.find(name); count != live_ints_.end())
                {
                    *count->second = static_cast<int>(parameter.as_int());
                }
                else if (name == "grip_check_enabled")
                {
                    grip_check_enabled_ = parameter.as_bool();
                }
                else if (name == "object_timeout_ms")
                {
                    object_timeout_s_ = parameter.as_double() / 1000.0;
                }
                else if (name == "max_approach_tilt_deg")
                {
                    max_approach_tilt_rad_ = parameter.as_double() * M_PI / 180.0;
                }
            }
            return result;
        });
}

bool G1ManipulationServer::acquire()
{
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(get_logger(), "rejecting a goal: the arm is already running one");
        return false;
    }
    return true;
}

void G1ManipulationServer::initialize()
{
    // both_arms takes named postures only; pose goals would need the subgroup IK g1.srdf omits.
    for (const char* name : { "left_arm", "right_arm", "both_arms", "left_hand", "right_hand" })
    {
        MoveGroup& group = groups_.try_emplace(name, shared_from_this(), name).first->second;
        group.setMaxVelocityScalingFactor(velocity_scaling_);
        group.setMaxAccelerationScalingFactor(velocity_scaling_);
        group.setPlanningTime(planning_time_s_);
    }
    planning_frame_ = groups_.at("left_arm").getPlanningFrame();

    // Servers only once the groups exist, so an early goal is refused rather than timed out.
    pick_server_ = rclcpp_action::create_server<Pick>(
        this,
        "~/pick",
        [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Pick::Goal>&) {
            return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                               rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle<Pick>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<Pick>>& handle) {
            std::thread{ [this, handle] {
                runGuarded([&] { executePick(handle); }, handle);
            } }.detach();
        });

    place_server_ = rclcpp_action::create_server<Place>(
        this,
        "~/place",
        [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const Place::Goal>&) {
            return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                               rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle<Place>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<Place>>& handle) {
            std::thread{ [this, handle] {
                runGuarded([&] { executePlace(handle); }, handle);
            } }.detach();
        });

    posture_server_ = rclcpp_action::create_server<SetArmPosture>(
        this,
        "~/set_arm_posture",
        [this](const rclcpp_action::GoalUUID&, const std::shared_ptr<const SetArmPosture::Goal>&) {
            return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                               rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandle<SetArmPosture>>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<SetArmPosture>>& handle) {
            std::thread{ [this, handle] {
                runGuarded([&] { executeSetArmPosture(handle); }, handle);
            } }.detach();
        });

    RCLCPP_INFO(
        get_logger(),
        "Manipulation skills up, planning in '%s'. Executing needs the arm and hands already "
        "acquired; this node never takes control authority itself.",
        planning_frame_.c_str());
}

G1ManipulationServer::MoveGroup* G1ManipulationServer::groupFor(const std::string& name)
{
    const auto it = groups_.find(name);
    return it == groups_.end() ? nullptr : &it->second;
}

void G1ManipulationServer::executePick(const std::shared_ptr<GoalHandle<Pick>>& goal_handle)
{
    const auto goal     = goal_handle->get_goal();
    auto       result   = std::make_shared<Pick::Result>();
    auto       feedback = std::make_shared<Pick::Feedback>();

    ArmContext arm;
    if (!resolveArm(goal->arm, arm))
    {
        result->success = false;
        result->message = "arm must be 'left' or 'right', got '" + goal->arm + "'";
        goal_handle->abort(result);
        return;
    }
    MoveGroup* arm_group  = groupFor(arm.arm_group);
    MoveGroup* hand_group = groupFor(arm.hand_group);

    // Every exit leaves the hand open, nothing attached and the scene restored, so a retry starts
    // from a defined state.
    bool       descended = false;
    const auto clean_up  = [&] {
        moveHandTo(*hand_group, "pinch_ready");
        // Up before the exemptions go, or the arm is left standing in the map.
        if (descended)
        {
            backOff(arm, *arm_group);
        }
        removeFromScene(goal->object_id);
        setHandContact(arm, { "<octomap>", goal->object_id }, false);
    };
    const auto fail = [&](const std::string& phase, const std::string& why) {
        clean_up();
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
    };
    const auto cancelled = [&](const std::string& phase) {
        clean_up();
        result->success = false;
        result->message = phase + ": cancelled";
        goal_handle->canceled(result);
    };
    // Once the hand has closed on the object a cancel keeps it: opening the hand would drop it,
    // and the stop behind the cancel holds the arm where it is, so nothing moves.
    const auto cancelled_holding = [&](const std::string& phase) {
        // The object maps itself while held, as after a finished pick.
        setHandContact(arm, { "<octomap>", goal->object_id }, false);
        setHandContact(arm, { "<octomap>", goal->object_id }, true, /*include_links=*/false);
        result->success = false;
        result->message =
            phase + ": cancelled; the " + goal->arm + " hand keeps " + goal->object_id;
        goal_handle->canceled(result);
    };

    feedback->phase = Pick::Feedback::PHASE_LOCATING;
    goal_handle->publish_feedback(feedback);
    // A standing object has not moved since its last sighting, which covers detector misses.
    auto detection = lookUpObject(goal->object_id);
    if (!detection)
    {
        detection = lastSighting(goal->object_id);
    }
    if (!detection)
    {
        fail(Pick::Feedback::PHASE_LOCATING, "no fresh pose for '" + goal->object_id + "'");
        return;
    }
    const auto object_pose =
        toPlanningFrame(detection->results.front().pose.pose, detection->header.frame_id);
    if (!object_pose)
    {
        fail(Pick::Feedback::PHASE_LOCATING, "could not transform the object pose");
        return;
    }
    moveit_msgs::msg::CollisionObject object = publishCollisionObject(*detection, *object_pose);

    std::string                  why;
    std::vector<ConsideredGrasp> verdicts;
    const auto                   chosen =
        chooseGrasp(*detection, *object_pose, arm, why, grasp_plan_pub_ ? &verdicts : nullptr);
    if (grasp_plan_pub_)
    {
        publishGraspPlan(chosen ? &*chosen : nullptr, verdicts);
    }
    if (!chosen)
    {
        fail(Pick::Feedback::PHASE_LOCATING, "no usable grasp: " + why);
        return;
    }
    RCLCPP_INFO(
        get_logger(),
        "picking '%s' with the %s grasp",
        goal->object_id.c_str(),
        chosen->origin.c_str());
    const geometry_msgs::msg::Pose& pregrasp_goal = chosen->pregrasp;

    // Cancels are honoured between phases; a trajectory already executing cannot be unwound here.
    if (goal_handle->is_canceling())
    {
        cancelled(Pick::Feedback::PHASE_PREGRASP);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_PREGRASP;
    goal_handle->publish_feedback(feedback);
    // pinch_ready, not open: an open thumb hangs low enough to ground on the table first.
    if (!moveHandTo(*hand_group, "pinch_ready"))
    {
        fail(Pick::Feedback::PHASE_PREGRASP, "the hand would not open to pinch_ready");
        return;
    }
    if (!moveTo(*arm_group, pregrasp_goal, arm.grasp_frame, "pregrasp"))
    {
        // Where, because the cause is usually where the base stopped.
        fail(
            Pick::Feedback::PHASE_PREGRASP,
            std::format(
                "could not reach the pregrasp pose at ({:.3f} {:.3f} {:.3f}) in {}",
                pregrasp_goal.position.x,
                pregrasp_goal.position.y,
                pregrasp_goal.position.z,
                planning_frame_));
        return;
    }
    // The droop comes out up here in clear air, not by dragging the hand sideways at the object.
    if (!settleOnPose(*arm_group, pregrasp_goal, arm.grasp_frame, "pregrasp"))
    {
        fail(Pick::Feedback::PHASE_PREGRASP, "the hand would not settle on the pregrasp pose");
        return;
    }

    if (goal_handle->is_canceling())
    {
        cancelled(Pick::Feedback::PHASE_APPROACH);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_APPROACH;
    goal_handle->publish_feedback(feedback);

    // Re-measured at the pregrasp: on its legs the robot's pelvis moves while the arm reaches,
    // and the pelvis-frame target moves with it. Only the position follows the object's shift.
    geometry_msgs::msg::Pose descent_goal = chosen->grasp;
    if (const auto fresh = lookUpObject(goal->object_id))
    {
        if (const auto fresh_pose =
                toPlanningFrame(fresh->results.front().pose.pose, fresh->header.frame_id))
        {
            const double dx    = fresh_pose->position.x - object_pose->position.x;
            const double dy    = fresh_pose->position.y - object_pose->position.y;
            const double dz    = fresh_pose->position.z - object_pose->position.z;
            const double shift = std::hypot(dx, dy, dz);
            // Larger than body sway is a different object or a bad frame.
            if (shift > grasp_refresh_max_shift_m_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "approach: '%s' moved %.0f mm since it was located, past the %.0f mm this "
                    "correction trusts; descending on the original pose",
                    goal->object_id.c_str(),
                    shift * 1000.0,
                    grasp_refresh_max_shift_m_ * 1000.0);
            }
            else
            {
                descent_goal.position.x += dx;
                descent_goal.position.y += dy;
                descent_goal.position.z += dz;
                // The attached body has to sit where the object is now.
                geometry_msgs::msg::Point& at = object.primitive_poses.front().position;
                at.x += dx;
                at.y += dy;
                at.z += dz;
                RCLCPP_INFO(
                    get_logger(),
                    "approach: re-aimed %.0f mm for body sway since the object was located",
                    shift * 1000.0);
            }
        }
    }

    // Staged while the object is still in the scene, so the planned path avoids it.
    if (!moveTo(*arm_group, stagingPose(pregrasp_goal, descent_goal), arm.grasp_frame, "stage"))
    {
        RCLCPP_WARN(get_logger(), "approach: could not stage above the object; descending anyway");
    }
    // Then remove-descend-close-attach, as MoveIt's own pick does. Contact is allowed only for
    // the last few centimetres: exempt all skill long, a plan could route through the table.
    removeFromScene(goal->object_id);
    clearOctomapKeeping(arm, { "<octomap>" });
    descended = true;
    if (!descendOnto(*arm_group, pregrasp_goal, descent_goal, arm.grasp_frame, max_grasp_offset_m_))
    {
        fail(Pick::Feedback::PHASE_APPROACH, "could not reach the grasp pose");
        return;
    }

    if (goal_handle->is_canceling())
    {
        cancelled(Pick::Feedback::PHASE_GRASP);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_GRASP;
    goal_handle->publish_feedback(feedback);
    // Both horizontal sides: depth sees only the near surface, so one of them reads short.
    const double widest_m    = std::max(detection->bbox.size.x, detection->bbox.size.y);
    const double narrowest_m = std::min(detection->bbox.size.x, detection->bbox.size.y);
    if (!closeHandOn(*hand_group, arm, widest_m, narrowest_m))
    {
        fail(Pick::Feedback::PHASE_GRASP, "the hand did not close");
        return;
    }
    std::string grip;
    if (!isHolding(arm, grip, /*opposed=*/true))
    {
        fail(Pick::Feedback::PHASE_GRASP, "the hand closed on nothing: " + grip);
        return;
    }

    // Built here rather than attachObject(id, link), which only promotes an object still in the
    // world. The hand, palm and wrist may touch it, as allowHandContact() exempts them.
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name        = arm.palm_link;
    attached.object           = object;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached.touch_links =
        hand_group->getRobotModel()->getJointModelGroup(arm.hand_group)->getLinkModelNames();
    attached.touch_links.push_back(arm.palm_link);
    const std::string side = arm.is_left ? "left" : "right";
    for (const char* wrist : { "_wrist_pitch_link", "_wrist_yaw_link", "_wrist_roll_link" })
    {
        attached.touch_links.push_back(side + wrist);
    }
    planning_scene_.applyAttachedCollisionObject(attached);
    // The object still sits in the surface's voxels, so the lift starts exempt from them.
    setHandContact(arm, { "<octomap>", goal->object_id }, true);

    if (goal_handle->is_canceling())
    {
        cancelled_holding(Pick::Feedback::PHASE_LIFT);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_LIFT;
    goal_handle->publish_feedback(feedback);
    // Straight up from where the hand closed. A free planner with the table exempt would drag
    // the object across it, so each retry starts a new line from wherever the last one stopped.
    // A stop cancels the arm's trajectory too and cuts a line short, so the cancel is checked
    // before each attempt, not taken for a failed lift.
    geometry_msgs::msg::Pose lifted = descent_goal;
    lifted.position.z += lift_height_m_;
    double lift_fraction = 0.0;
    bool   lifted_some   = false;
    for (int attempt = 0; attempt < lift_attempts_; ++attempt)
    {
        if (goal_handle->is_canceling())
        {
            cancelled_holding(Pick::Feedback::PHASE_LIFT);
            return;
        }
        lift_fraction = moveStraight(*arm_group, lifted, arm.grasp_frame, "lift", 0.0);
        lifted_some   = lifted_some || lift_fraction > 0.0;
        if (lift_fraction <= 0.0 || lift_fraction >= kLineComplete)
        {
            break;
        }
    }
    if (goal_handle->is_canceling())
    {
        cancelled_holding(Pick::Feedback::PHASE_LIFT);
        return;
    }
    if (!lifted_some && !moveTo(*arm_group, lifted, arm.grasp_frame, "lift"))
    {
        if (goal_handle->is_canceling())
        {
            cancelled_holding(Pick::Feedback::PHASE_LIFT);
            return;
        }
        fail(Pick::Feedback::PHASE_LIFT, "could not lift clear of the surface");
        return;
    }
    // The gap to the target is mostly sag under load; only a short line is an obstacle.
    if (const auto residual = residualTo(*arm_group, lifted, arm.grasp_frame, "lift"))
    {
        RCLCPP_INFO(
            get_logger(),
            "lift: walked %.0f%% of the line, ended %.0f mm below the %.0f mm target",
            lift_fraction * 100.0,
            std::abs(residual->z) * 1000.0,
            lift_height_m_ * 1000.0);
    }
    // Reported only: a cradled object loads the fingers too little to tell held from dropped.
    if (std::string after_lift; !isHolding(arm, after_lift))
    {
        RCLCPP_INFO(get_logger(), "lift: %s", after_lift.c_str());
    }

    // The hand goes back to respecting the map, but the object maps itself while held, so it
    // stays exempt from its own voxels until the place.
    setHandContact(arm, { "<octomap>", goal->object_id }, false);
    setHandContact(arm, { "<octomap>", goal->object_id }, true, /*include_links=*/false);
    result->success = true;
    result->message = "picked " + goal->object_id + " with the " + goal->arm + " hand, " + grip;
    goal_handle->succeed(result);
}

void G1ManipulationServer::executePlace(const std::shared_ptr<GoalHandle<Place>>& goal_handle)
{
    const auto goal     = goal_handle->get_goal();
    auto       result   = std::make_shared<Place::Result>();
    auto       feedback = std::make_shared<Place::Feedback>();
    const auto refuse   = [&](const std::string& why) {
        result->success = false;
        result->message = why;
        goal_handle->abort(result);
    };

    ArmContext arm;
    if (!resolveArm(goal->arm, arm))
    {
        refuse("arm must be 'left' or 'right', got '" + goal->arm + "'");
        return;
    }
    MoveGroup* arm_group  = groupFor(arm.arm_group);
    MoveGroup* hand_group = groupFor(arm.hand_group);

    // What this arm holds, from the planning scene; the attach in executePick put it there.
    std::string                          held_id;
    double                               held_height = 0.0;
    const moveit::core::JointModelGroup* arm_links =
        arm_group->getRobotModel()->getJointModelGroup(arm.arm_group);
    for (const auto& [id, attached] : planning_scene_.getAttachedObjects())
    {
        if (arm_links->hasLinkModel(attached.link_name) && !attached.object.primitives.empty() &&
            attached.object.primitives.front().dimensions.size() == 3)
        {
            held_id     = id;
            held_height = attached.object.primitives.front().dimensions[2];
            break;
        }
    }
    if (held_id.empty())
    {
        refuse(
            std::string(Place::Feedback::PHASE_PREPLACE) + ": the " + goal->arm +
            " hand is holding nothing");
        return;
    }

    // An attached body is its own collision entity, so exempting the hand does nothing for it.
    const std::vector<std::string> touchables{ "<octomap>", held_id };
    bool                           lowered  = false;
    const auto                     clean_up = [&] {
        if (lowered)
        {
            backOff(arm, *arm_group);
        }
        setHandContact(arm, touchables, false);
    };
    const auto fail = [&](const std::string& phase, const std::string& why) {
        clean_up();
        result->success = false;
        result->message = phase + ": " + why;
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        goal_handle->abort(result);
    };
    const auto cancelled = [&](const std::string& phase) {
        clean_up();
        result->success = false;
        result->message = phase + ": cancelled";
        goal_handle->canceled(result);
    };

    // A named surface beats a coordinate: a tree writes its coordinates in map, and map-to-odom
    // drift alone exceeds the arm's lateral window.
    std::optional<geometry_msgs::msg::Pose> target;
    std::optional<Detection>                surface;
    if (!goal->surface_object_id.empty())
    {
        // The hand carrying the object often hides the surface, so a recent sighting stands in.
        surface = waitForObject(goal->surface_object_id, place_confirm_timeout_s_);
        if (!surface)
        {
            surface = lastSighting(goal->surface_object_id);
        }
        if (!surface)
        {
            fail(
                Place::Feedback::PHASE_PREPLACE,
                "nothing called '" + goal->surface_object_id + "' on /objects");
            return;
        }
        target = toPlanningFrame(surface->results.front().pose.pose, surface->header.frame_id);
    }
    else
    {
        target = toPlanningFrame(goal->pose.pose, goal->pose.header.frame_id);
    }
    if (!target)
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not transform the target pose");
        return;
    }
    // A surface reports its centre; the object goes on top of it.
    if (surface)
    {
        target->position.z += 0.5 * (surface->bbox.size.z + held_height);
    }
    geometry_msgs::msg::Pose place_goal = graspFrameGoal(*target, held_height, arm);
    geometry_msgs::msg::Pose preplace   = place_goal;
    preplace.position.z += place_approach_height_m_;

    if (goal_handle->is_canceling())
    {
        cancelled(Place::Feedback::PHASE_PREPLACE);
        return;
    }
    feedback->phase = Place::Feedback::PHASE_PREPLACE;
    goal_handle->publish_feedback(feedback);
    // The map has filled in around the carry pose, and the object and the fingers round it sit
    // in the voxels it casts, so the hand and the object stay exempt.
    clearOctomapKeeping(arm, touchables);
    if (!moveTo(*arm_group, preplace, arm.grasp_frame, "preplace"))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not reach the pose above the target");
        return;
    }
    if (!settleOnPose(*arm_group, preplace, arm.grasp_frame, "preplace"))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "the hand would not settle above the target");
        return;
    }

    // Re-resolved after the reach, since a loaded arm shifts the COM and the gait steps under it.
    // `expected` stays in the /objects frame, where the base's own travel cancels out.
    std::optional<geometry_msgs::msg::Point> expected;
    if (surface)
    {
        if (const auto fresh = lookUpObject(goal->surface_object_id, /*report=*/false))
        {
            const double lift = 0.5 * (fresh->bbox.size.z + held_height);
            expected          = fresh->results.front().pose.pose.position;
            expected->z += lift;
            if (auto moved =
                    toPlanningFrame(fresh->results.front().pose.pose, fresh->header.frame_id))
            {
                moved->position.z += lift;
                const geometry_msgs::msg::Pose regrasp = graspFrameGoal(*moved, held_height, arm);
                const double                   shift   = std::hypot(
                    regrasp.position.x - place_goal.position.x,
                    regrasp.position.y - place_goal.position.y);
                if (shift > kReportShiftM)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "the base moved %.3f m during the reach; re-aiming",
                        shift);
                }
                target     = moved;
                place_goal = regrasp;
                // The retreat returns here, so it moves with the re-aim.
                preplace = place_goal;
                preplace.position.z += place_approach_height_m_;
            }
        }
    }

    if (goal_handle->is_canceling())
    {
        cancelled(Place::Feedback::PHASE_LOWER);
        return;
    }
    feedback->phase = Place::Feedback::PHASE_LOWER;
    goal_handle->publish_feedback(feedback);
    clearOctomapKeeping(arm, touchables);
    lowered = true;
    // No refusal on a miss: a release a little off still lands the object, and the landing check
    // judges it.
    if (!descendOnto(
            *arm_group,
            preplace,
            place_goal,
            arm.grasp_frame,
            std::numeric_limits<double>::infinity()))
    {
        fail(Place::Feedback::PHASE_LOWER, "could not lower onto the target");
        return;
    }

    if (goal_handle->is_canceling())
    {
        cancelled(Place::Feedback::PHASE_RELEASE);
        return;
    }
    feedback->phase = Place::Feedback::PHASE_RELEASE;
    goal_handle->publish_feedback(feedback);
    if (!moveHandTo(*hand_group, "pinch_ready"))
    {
        fail(Place::Feedback::PHASE_RELEASE, "the hand did not let go");
        return;
    }
    removeFromScene(held_id);

    if (goal_handle->is_canceling())
    {
        cancelled(Place::Feedback::PHASE_RETREAT);
        return;
    }
    feedback->phase = Place::Feedback::PHASE_RETREAT;
    goal_handle->publish_feedback(feedback);
    // Straight up with the hand still exempt: it is wrapped round what it let go of. The map is
    // fresh because the forearm stood in it through the lower and the release.
    clearOctomapKeeping(arm, touchables);
    if (moveStraight(*arm_group, preplace, arm.grasp_frame, "retreat", 0.0) <= 0.0 &&
        !moveTo(*arm_group, preplace, arm.grasp_frame, "retreat"))
    {
        fail(Place::Feedback::PHASE_RETREAT, "could not retreat clear of the object");
        return;
    }
    lowered = false;
    setHandContact(arm, touchables, false);

    // A successful plan says nothing about where the object landed. The hand hid it until the
    // release, so the detector gets a moment to find it again.
    const auto landed = waitForObject(held_id, place_confirm_timeout_s_);
    if (!landed)
    {
        fail(
            Place::Feedback::PHASE_RETREAT,
            held_id + " is not on /objects after the release, so where it landed cannot be "
                      "confirmed");
        return;
    }
    const geometry_msgs::msg::Point&         aim   = expected ? *expected : target->position;
    std::optional<geometry_msgs::msg::Point> where = landed->results.front().pose.pose.position;
    if (!expected)
    {
        const auto in_planning =
            toPlanningFrame(landed->results.front().pose.pose, landed->header.frame_id);
        where = in_planning ? std::optional(in_planning->position) : std::nullopt;
    }
    if (!where)
    {
        fail(
            Place::Feedback::PHASE_RETREAT,
            held_id + " was found but its pose will not transform, so where it landed cannot be "
                      "confirmed");
        return;
    }
    const double off       = std::hypot(where->x - aim.x, where->y - aim.y, where->z - aim.z);
    bool         on_target = off <= place_tolerance_m_;
    if (!on_target && surface)
    {
        // On a named surface: inside its footprint, and between its floor and just above its top.
        // A radius misjudges containers, whose walls clip the mask and pull it toward the rim.
        const geometry_msgs::msg::Vector3& extent = surface->bbox.size;
        const double                       dx     = std::abs(where->x - aim.x);
        const double                       dy     = std::abs(where->y - aim.y);
        const double                       rise   = where->z - aim.z;
        on_target = dx <= (0.5 * extent.x) + place_footprint_margin_m_ &&
                    dy <= (0.5 * extent.y) + place_footprint_margin_m_ &&
                    std::hypot(dx, dy) <=
                        (0.5 * std::max(extent.x, extent.y)) + place_footprint_margin_m_ &&
                    rise >= -(extent.z + place_tolerance_m_) && rise <= place_tolerance_m_;
    }
    if (!on_target)
    {
        fail(
            Place::Feedback::PHASE_RETREAT,
            std::format(
                "{} ended up {:.3f} m from where it was placed, at ({:.3f} {:.3f} {:.3f}) "
                "against ({:.3f} {:.3f} {:.3f})",
                held_id,
                off,
                where->x,
                where->y,
                where->z,
                aim.x,
                aim.y,
                aim.z));
        return;
    }
    RCLCPP_INFO(get_logger(), "%s came to rest %.3f m from the target", held_id.c_str(), off);

    result->success = true;
    result->message = "placed with the " + goal->arm + " hand";
    goal_handle->succeed(result);
}

void G1ManipulationServer::executeSetArmPosture(
    const std::shared_ptr<GoalHandle<SetArmPosture>>& goal_handle)
{
    const auto goal   = goal_handle->get_goal();
    auto       result = std::make_shared<SetArmPosture::Result>();

    MoveGroup* group = groupFor(goal->group);
    if (group == nullptr)
    {
        result->success = false;
        result->message = "'" + goal->group + "' is not a group this node drives";
        goal_handle->abort(result);
        return;
    }
    if (!moveToNamed(*group, goal->named_target))
    {
        result->success = false;
        result->message = "could not reach '" + goal->named_target + "'";
        goal_handle->abort(result);
        return;
    }
    result->success = true;
    result->message = goal->group + " is at " + goal->named_target;
    goal_handle->succeed(result);
}

}  // namespace g1_manipulation
