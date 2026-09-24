/**
 * @file g1_manipulation_server_node.cpp
 * @brief The pick, place and posture action servers, planned and executed through MoveIt.
 */

#include "g1_manipulation/g1_manipulation_server_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <utility>
#include <vector>

#include "g1_manipulation/grasp_filter.hpp"
#include "g1_manipulation/grip_check.hpp"
#include "g1_manipulation/hand_contact.hpp"

namespace g1_manipulation
{

namespace
{

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// MoveIt treats a zero-extent primitive as degenerate and silently drops the attach.
constexpr double kMinPrimitiveExtent = 0.005;
// Shortest hand trajectory, so a small grip-search step is still a ramp.
constexpr double kMinHandMoveS = 0.15;
// A Cartesian fraction at or above this is a line walked to its end.
constexpr double kLineComplete = 0.99;
// Base travel during a place's reach below this is not worth a log line.
constexpr double kReportShiftM        = 0.01;
constexpr auto   kClearOctomapTimeout = std::chrono::seconds(2);
constexpr auto   kObjectPollPeriod    = std::chrono::milliseconds(200);
constexpr auto   kHandSamplePeriod    = std::chrono::milliseconds(100);

constexpr double kCandidateArrowLengthM = 0.06;
constexpr double kGraspAxisLengthM      = 0.05;
constexpr double kLabelHeightM          = 0.02;
constexpr double kLabelLiftM            = 0.03;

rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

const std::string& idOf(const vision_msgs::msg::Detection3D& detection)
{
    return detection.results.front().hypothesis.class_id;
}

/// Whether a detection's pose and box can be planned against.
bool isFinite(const vision_msgs::msg::Detection3D& detection)
{
    const geometry_msgs::msg::Pose&    pose = detection.results.front().pose.pose;
    const geometry_msgs::msg::Vector3& size = detection.bbox.size;
    return std::ranges::all_of(
        std::initializer_list<double>{ pose.position.x,
                                       pose.position.y,
                                       pose.position.z,
                                       pose.orientation.x,
                                       pose.orientation.y,
                                       pose.orientation.z,
                                       pose.orientation.w,
                                       size.x,
                                       size.y,
                                       size.z },
        [](double value) { return std::isfinite(value); });
}

/// `open` to `closed` interpolated at @p fraction, per joint; empty unless both poses name it.
std::map<std::string, double> handTargetsAt(MoveGroupInterface& hand, double fraction)
{
    const std::map<std::string, double> open_pose = hand.getNamedTargetValues("open");
    std::map<std::string, double>       targets;
    for (const auto& [joint, closed] : hand.getNamedTargetValues("closed"))
    {
        const auto open = open_pose.find(joint);
        if (open == open_pose.end())
        {
            return {};
        }
        targets.emplace(joint, open->second + (fraction * (closed - open->second)));
    }
    return targets;
}

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

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
        auto group = std::make_shared<MoveGroup>(shared_from_this(), name);
        group->setMaxVelocityScalingFactor(velocity_scaling_);
        group->setMaxAccelerationScalingFactor(velocity_scaling_);
        group->setPlanningTime(planning_time_s_);
        groups_.emplace(name, group);
    }
    planning_frame_ = groups_.at("left_arm")->getPlanningFrame();

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

std::optional<geometry_msgs::msg::Pose> G1ManipulationServer::toPlanningFrame(
    const geometry_msgs::msg::Pose& pose, const std::string& frame_id)
{
    if (frame_id.empty() || frame_id == planning_frame_)
    {
        return pose;
    }

    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = frame_id;
    // Latest transform, not the pose's stamp: a standing object's odom pose stays valid, and the
    // question is where it is from where the robot stands now.
    in.header.stamp = rclcpp::Time(0);
    in.pose         = pose;

    try
    {
        return tf_buffer_->transform(in, planning_frame_, std::chrono::milliseconds(500)).pose;
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "cannot transform '%s' into the planning frame '%s': %s",
            frame_id.c_str(),
            planning_frame_.c_str(),
            e.what());
        return std::nullopt;
    }
}

void G1ManipulationServer::onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg)
{
    const std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_ = *msg;
    for (const Detection& detection : msg->detections)
    {
        if (!detection.results.empty())
        {
            Detection& seen = sightings_[idOf(detection)];
            seen            = detection;
            seen.header     = msg->header;
        }
    }
    // Track ids are numbered afresh as tracks come and go, so aged-out ones are dropped.
    const rclcpp::Time newest(msg->header.stamp);
    std::erase_if(sightings_, [&](const auto& entry) {
        return (newest - rclcpp::Time(entry.second.header.stamp)).seconds() > sighting_memory_s_;
    });
}

std::optional<G1ManipulationServer::Detection>
G1ManipulationServer::lookUpObject(const std::string& object_id, bool report)
{
    std::optional<Detection> found;
    std_msgs::msg::Header    header;
    std::size_t              known = 0;
    {
        const std::lock_guard<std::mutex> lock(objects_mutex_);
        header = objects_.header;
        known  = objects_.detections.size();
        for (const Detection& detection : objects_.detections)
        {
            if (!detection.results.empty() && idOf(detection) == object_id)
            {
                found = detection;
                break;
            }
        }
    }

    // Judged here rather than at the source: only the skill about to commit the arm knows how old
    // is too old, which is why the pose source forwards the capture stamp.
    const double age = (now() - rclcpp::Time(header.stamp)).seconds();
    if (known == 0 || age > object_timeout_s_)
    {
        if (report)
        {
            RCLCPP_ERROR(
                get_logger(),
                "No usable object poses: %zu known, newest %.2f s old (limit %.2f). Is "
                "g1_object_pose_source active?",
                known,
                age,
                object_timeout_s_);
        }
        return std::nullopt;
    }
    if (!found)
    {
        if (report)
        {
            RCLCPP_ERROR(get_logger(), "No object called '%s' is being reported", object_id.c_str());
        }
        return std::nullopt;
    }
    if (!isFinite(*found))
    {
        if (report)
        {
            RCLCPP_ERROR(get_logger(), "'%s' has a non-finite pose or size", object_id.c_str());
        }
        return std::nullopt;
    }
    found->header = header;
    return found;
}

std::optional<G1ManipulationServer::Detection>
G1ManipulationServer::waitForObject(const std::string& object_id, double timeout_s)
{
    const rclcpp::Time deadline = now() + rclcpp::Duration::from_seconds(timeout_s);
    while (now() < deadline)
    {
        if (auto found = lookUpObject(object_id, /*report=*/false))
        {
            return found;
        }
        rclcpp::sleep_for(kObjectPollPeriod);
    }
    return lookUpObject(object_id);
}

std::optional<G1ManipulationServer::Detection>
G1ManipulationServer::lastSighting(const std::string& object_id)
{
    Detection seen;
    {
        const std::lock_guard<std::mutex> lock(objects_mutex_);
        const auto                        found = sightings_.find(object_id);
        if (found == sightings_.end())
        {
            return std::nullopt;
        }
        seen = found->second;
    }
    const double age = (now() - rclcpp::Time(seen.header.stamp)).seconds();
    if (age > sighting_memory_s_ || !isFinite(seen))
    {
        return std::nullopt;
    }
    RCLCPP_INFO(
        get_logger(),
        "'%s' is not in the newest detections; using where it was seen %.1f s ago",
        object_id.c_str(),
        age);
    return seen;
}

std::optional<G1ManipulationServer::JointSample>
G1ManipulationServer::readJoints(const std::vector<std::string>& joints)
{
    const std::lock_guard<std::mutex> lock(joint_states_mutex_);
    const std::vector<std::string>&   names = joint_states_.name;
    if (names.size() != joint_states_.position.size())
    {
        return std::nullopt;
    }
    const bool  with_effort = joint_states_.effort.size() == names.size();
    JointSample sample;
    sample.position.reserve(joints.size());
    if (with_effort)
    {
        sample.effort.reserve(joints.size());
    }
    for (const std::string& joint : joints)
    {
        const auto at = std::ranges::find(names, joint);
        if (at == names.end())
        {
            return std::nullopt;
        }
        const auto index = static_cast<std::size_t>(std::distance(names.begin(), at));
        sample.position.push_back(joint_states_.position[index]);
        if (with_effort)
        {
            sample.effort.push_back(joint_states_.effort[index]);
        }
    }
    return sample;
}

bool G1ManipulationServer::isHolding(const ArmContext& arm, std::string& why, bool opposed)
{
    if (!grip_check_enabled_)
    {
        why = "grip check disabled";
        return true;
    }
    MoveGroup* hand = groupFor(arm.hand_group);
    if (hand == nullptr)
    {
        why = "no hand group called " + arm.hand_group;
        return false;
    }
    // Against the last close, not `closed`: a close onto a measured width stops short on purpose.
    // After holdCurrentGrip the fingers sit short of these by construction, so effort decides.
    const std::map<std::string, double> targets = handTargetsAt(*hand, last_close_fraction_);
    std::vector<std::string>            joints;
    joints.reserve(targets.size());
    for (const auto& entry : targets)
    {
        joints.push_back(entry.first);
    }
    const std::optional<JointSample> measured = readJoints(joints);
    if (targets.empty() || !measured)
    {
        why = "the hand's joints are not all in /joint_states";
        return false;
    }
    if (measured->effort.empty())
    {
        why = "joint states carry no effort, so a grip cannot be told from an empty hand";
        return false;
    }

    std::vector<JointGrip> fingers;
    fingers.reserve(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        fingers.push_back(
            { joints[i], targets.at(joints[i]), measured->position[i], measured->effort[i] });
    }
    const GripVerdict verdict =
        verifyGrip(fingers, grip_min_position_error_rad_, grip_min_effort_nm_);
    why = verdict.why;
    return opposed ? verdict.opposed : verdict.holding;
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

void G1ManipulationServer::setStartStateInBounds(MoveGroup& group)
{
    const auto state = group.getCurrentState();
    if (!state)
    {
        group.setStartStateToCurrentState();
        return;
    }
    moveit::core::RobotState bounded(*state);
    bounded.enforceBounds(bounded.getRobotModel()->getJointModelGroup(group.getName()));
    group.setStartState(bounded);
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

std::optional<geometry_msgs::msg::Point> G1ManipulationServer::residualTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    geometry_msgs::msg::TransformStamped here;
    try
    {
        here = tf_buffer_->lookupTransform(
            group.getPlanningFrame(),
            link,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5));
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_WARN(get_logger(), "%s: cannot measure %s: %s", what.c_str(), link.c_str(), e.what());
        return std::nullopt;
    }
    geometry_msgs::msg::Point residual;
    residual.x = pose.position.x - here.transform.translation.x;
    residual.y = pose.position.y - here.transform.translation.y;
    residual.z = pose.position.z - here.transform.translation.z;
    return residual;
}

double G1ManipulationServer::moveStraight(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what, double min_fraction)
{
    setStartStateInBounds(group);
    const std::string previous_tip = group.getEndEffectorLink();
    group.setEndEffectorLink(link);
    moveit_msgs::msg::RobotTrajectory           path;
    const std::vector<geometry_msgs::msg::Pose> waypoints{ pose };
    const double                                fraction =
        group.computeCartesianPath(waypoints, cartesian_step_m_, path, /*avoid_collisions=*/true);
    group.setEndEffectorLink(previous_tip);
    if (fraction <= 0.0 || fraction < min_fraction)
    {
        return fraction;
    }
    MoveGroup::Plan plan;
    plan.trajectory = path;
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_WARN(get_logger(), "%s: the straight line would not execute", what.c_str());
        return 0.0;
    }
    // execute() returns before the arm has sagged into its steady-state error.
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_wait_s_));
    return fraction;
}

geometry_msgs::msg::Pose G1ManipulationServer::stagingPose(
    const geometry_msgs::msg::Pose& pregrasp, const geometry_msgs::msg::Pose& grasp) const
{
    const double axis_length = std::hypot(
        pregrasp.position.x - grasp.position.x,
        pregrasp.position.y - grasp.position.y,
        pregrasp.position.z - grasp.position.z);
    if (axis_length <= 0.0)
    {
        return grasp;
    }
    const double             scale   = reaim_clearance_m_ / axis_length;
    geometry_msgs::msg::Pose staging = grasp;
    staging.position.x += (pregrasp.position.x - grasp.position.x) * scale;
    staging.position.y += (pregrasp.position.y - grasp.position.y) * scale;
    staging.position.z += (pregrasp.position.z - grasp.position.z) * scale;
    return staging;
}

void G1ManipulationServer::backOff(const ArmContext& arm, MoveGroup& group)
{
    geometry_msgs::msg::Pose clear = group.getCurrentPose(arm.grasp_frame).pose;
    clear.position.z += reaim_clearance_m_;
    moveStraight(group, clear, arm.grasp_frame, "back off", 0.0);
}

bool G1ManipulationServer::descendOnto(
    MoveGroup& group, const geometry_msgs::msg::Pose& pregrasp,
    const geometry_msgs::msg::Pose& grasp, const std::string& link, double max_offset_m)
{
    if (stagingPose(pregrasp, grasp).position == grasp.position)
    {
        RCLCPP_ERROR(get_logger(), "approach: the pregrasp and the grasp are the same pose");
        return false;
    }

    geometry_msgs::msg::Pose commanded = grasp;
    for (int attempt = 0; attempt <= settle_attempts_; ++attempt)
    {
        const geometry_msgs::msg::Pose staging = stagingPose(pregrasp, commanded);
        if (attempt > 0)
        {
            // Straight back up the axis first: a planned move from grasp height can sweep the
            // object aside.
            geometry_msgs::msg::Pose raised = commanded;
            raised.position                 = group.getCurrentPose(link).pose.position;
            raised.position.x += staging.position.x - commanded.position.x;
            raised.position.y += staging.position.y - commanded.position.y;
            raised.position.z += staging.position.z - commanded.position.z;
            moveStraight(group, raised, link, "back up", 0.0);
        }
        // Planned, because a retry needs a fresh arm configuration to stop the line truncating
        // in the same place. The first attempt skips it when the caller already staged.
        const auto to_staging = residualTo(group, staging, link, "re-stage");
        const bool staged =
            attempt == 0 && to_staging &&
            std::hypot(to_staging->x, to_staging->y, to_staging->z) <= settle_tolerance_m_;
        if (!staged && !moveTo(group, staging, link, "re-stage"))
        {
            RCLCPP_WARN(get_logger(), "approach: could not re-stage; descending from here");
        }

        const double walked   = moveStraight(group, commanded, link, "approach", 0.0);
        const auto   residual = residualTo(group, grasp, link, "approach");
        if (!residual)
        {
            return false;
        }
        const double error = std::hypot(residual->x, residual->y, residual->z);
        if (!std::isfinite(error))
        {
            RCLCPP_ERROR(get_logger(), "approach: the hand's position is not finite");
            return false;
        }
        if (error <= settle_tolerance_m_)
        {
            RCLCPP_INFO(
                get_logger(),
                "approach: on the grasp pose to %.0f mm after %d descent(s)",
                error * 1000.0,
                attempt + 1);
            return true;
        }
        if (attempt == settle_attempts_)
        {
            // Past max_offset_m the object cannot be between the fingers.
            if (error > max_offset_m)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "approach: %.0f mm off after %d descents, past the %.0f mm the hand can still "
                    "close over; not closing on air",
                    error * 1000.0,
                    attempt + 1,
                    max_offset_m * 1000.0);
                return false;
            }
            RCLCPP_WARN(
                get_logger(),
                "approach: still %.0f mm off after %d descents; closing from there",
                error * 1000.0,
                attempt + 1);
            return true;
        }
        // Overshoot only what the arm failed to hold. A line that ran out early has its unwalked
        // rest in the residual, and adding that aims the hand into the surface.
        if (walked >= 1.0)
        {
            RCLCPP_INFO(
                get_logger(),
                "approach: %.0f mm off, re-aiming by (%+.0f %+.0f %+.0f) mm",
                error * 1000.0,
                residual->x * 1000.0,
                residual->y * 1000.0,
                residual->z * 1000.0);
            commanded.position.x += residual->x;
            commanded.position.y += residual->y;
            commanded.position.z += residual->z;
        }
        else
        {
            RCLCPP_WARN(
                get_logger(),
                "approach: the straight line ran out %.0f%% in, %.0f mm short; starting it again",
                walked * 100.0,
                error * 1000.0);
        }
    }
    return true;
}

bool G1ManipulationServer::settleOnPose(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    // Re-commanding the residual beats raising gains, which the balance controller shares.
    geometry_msgs::msg::Pose commanded = pose;
    // One more measurement than nudges, so the last nudge is checked too.
    for (int attempt = 0; attempt <= settle_attempts_; ++attempt)
    {
        const auto residual = residualTo(group, pose, link, what);
        if (!residual)
        {
            return false;
        }
        const double error = std::hypot(residual->x, residual->y, residual->z);
        if (error <= settle_tolerance_m_)
        {
            RCLCPP_INFO(
                get_logger(),
                "%s: settled %.0f mm from the grasp pose after %d nudge(s)",
                what.c_str(),
                error * 1000.0,
                attempt);
            return true;
        }
        if (attempt == settle_attempts_)
        {
            break;
        }
        // The next target droops by about the same amount, so it lands on the one asked for.
        commanded.position.x += residual->x;
        commanded.position.y += residual->y;
        commanded.position.z += residual->z;
        RCLCPP_INFO(
            get_logger(),
            "%s: %.0f mm short, nudging (%+.0f %+.0f %+.0f) mm",
            what.c_str(),
            error * 1000.0,
            residual->x * 1000.0,
            residual->y * 1000.0,
            residual->z * 1000.0);
        // A straight line only: re-planning a small correction discards the arm's configuration.
        if (moveStraight(group, commanded, link, what + " settle", 0.0) <= 0.0)
        {
            RCLCPP_WARN(
                get_logger(),
                "%s: no straight line to the correction; carrying on",
                what.c_str());
            return true;
        }
    }
    RCLCPP_WARN(
        get_logger(),
        "%s: gave up nudging after %d attempts; carrying on from where it is",
        what.c_str(),
        settle_attempts_);
    return true;
}

bool G1ManipulationServer::moveTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    setStartStateInBounds(group);
    // The goal is for `link`, which is not the group's own tip.
    group.setPoseTarget(pose, link);

    MoveGroup::Plan plan;
    const auto      planned = planWithinBudget(group, plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: planning failed (%d)", what.c_str(), planned.val);
        return false;
    }
    const auto executed = group.execute(plan);
    if (executed != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s: execution failed (%d). The usual cause is the arm not being acquired; run "
            "g1_bringup's activate_arm first.",
            what.c_str(),
            executed.val);
        return false;
    }
    return true;
}

bool G1ManipulationServer::commandHand(
    MoveGroup& hand, const std::map<std::string, double>& targets, const std::string& what)
{
    const std::vector<std::string>&  joints = hand.getActiveJoints();
    const std::optional<JointSample> from   = readJoints(joints);
    if (!from)
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s: the %s joints are not all in /joint_states",
            what.c_str(),
            hand.getName().c_str());
        return false;
    }
    const std::map<std::string, double> open   = handTargetsAt(hand, 0.0);
    const std::map<std::string, double> closed = handTargetsAt(hand, 1.0);

    trajectory_msgs::msg::JointTrajectoryPoint start;
    trajectory_msgs::msg::JointTrajectoryPoint end;
    moveit_msgs::msg::RobotTrajectory          path;
    // Largest move as a share of that finger's full open-to-closed travel.
    double travel = 0.0;
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const auto target = targets.find(joints[i]);
        if (target == targets.end() || !std::isfinite(target->second))
        {
            RCLCPP_ERROR(
                get_logger(),
                "%s: no usable target for %s",
                what.c_str(),
                joints[i].c_str());
            return false;
        }
        path.joint_trajectory.joint_names.push_back(joints[i]);
        start.positions.push_back(from->position[i]);
        end.positions.push_back(target->second);
        const auto o = open.find(joints[i]);
        const auto c = closed.find(joints[i]);
        if (o != open.end() && c != closed.end() && c->second != o->second)
        {
            travel = std::max(
                travel,
                std::abs(target->second - from->position[i]) / std::abs(c->second - o->second));
        }
    }
    // Starts where the fingers are: execution refuses a trajectory that starts elsewhere.
    start.velocities.assign(start.positions.size(), 0.0);
    end.velocities.assign(end.positions.size(), 0.0);
    start.time_from_start = rclcpp::Duration::from_seconds(0.0);
    end.time_from_start   = rclcpp::Duration::from_seconds(
        std::clamp(travel * hand_close_s_, kMinHandMoveS, std::max(kMinHandMoveS, hand_close_s_)));
    path.joint_trajectory.points = { start, end };

    MoveGroup::Plan plan;
    plan.trajectory = std::move(path);
    if (hand.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "%s: the hand would not execute", what.c_str());
        return false;
    }
    return true;
}

bool G1ManipulationServer::moveHandTo(MoveGroup& hand, const std::string& named_target)
{
    const std::map<std::string, double> targets = hand.getNamedTargetValues(named_target);
    if (targets.empty())
    {
        RCLCPP_ERROR(
            get_logger(),
            "'%s' is not a named pose of group '%s'",
            named_target.c_str(),
            hand.getName().c_str());
        return false;
    }
    return commandHand(hand, targets, named_target);
}

bool G1ManipulationServer::moveHandToFraction(
    MoveGroup& hand, double fraction, const std::string& what)
{
    if (!std::isfinite(fraction))
    {
        RCLCPP_ERROR(get_logger(), "%s: refusing a non-finite hand fraction", what.c_str());
        return false;
    }
    const double                        f       = std::clamp(fraction, 0.0, 1.0);
    const std::map<std::string, double> targets = handTargetsAt(hand, f);
    if (targets.empty())
    {
        RCLCPP_ERROR(get_logger(), "%s needs both 'open' and 'closed'", hand.getName().c_str());
        return false;
    }
    if (!commandHand(hand, targets, what))
    {
        return false;
    }
    last_close_fraction_ = f;
    return true;
}

void G1ManipulationServer::holdCurrentGrip(MoveGroup& hand)
{
    // From /joint_states, not MoveGroup's cached state, which lags the close and would reopen it.
    const std::vector<std::string>&     joints  = hand.getActiveJoints();
    const std::optional<JointSample>    stalled = readJoints(joints);
    const std::map<std::string, double> open    = handTargetsAt(hand, 0.0);
    const std::map<std::string, double> closed  = handTargetsAt(hand, 1.0);

    std::map<std::string, double> targets;
    for (std::size_t i = 0; stalled && i < joints.size(); ++i)
    {
        const auto o = open.find(joints[i]);
        const auto c = closed.find(joints[i]);
        if (o == open.end() || c == closed.end())
        {
            break;
        }
        // Signed per joint: the thumb closes the other way on each hand.
        const double toward = c->second > o->second ? 1.0 : -1.0;
        const double biased = stalled->position[i] + (toward * grip_hold_bias_rad_);
        targets[joints[i]] =
            toward > 0.0 ? std::min(biased, c->second) : std::max(biased, c->second);
    }
    if (targets.size() != joints.size() || !commandHand(hand, targets, "hold"))
    {
        RCLCPP_WARN(
            get_logger(),
            "could not re-command the grip; the fingers keep the close's targets");
    }
}

bool G1ManipulationServer::waitForHandToSettle(MoveGroup& hand)
{
    const std::vector<std::string>& joints = hand.getActiveJoints();
    const double                    still_rad =
        grip_settled_speed_rad_s_ * std::chrono::duration<double>(kHandSamplePeriod).count();
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(hand_close_s_));
    std::optional<JointSample> before = readJoints(joints);
    while (before && std::chrono::steady_clock::now() < deadline)
    {
        std::this_thread::sleep_for(kHandSamplePeriod);
        std::optional<JointSample> after = readJoints(joints);
        if (!after)
        {
            return false;
        }
        double moved = 0.0;
        for (std::size_t i = 0; i < joints.size(); ++i)
        {
            moved = std::max(moved, std::abs(after->position[i] - before->position[i]));
        }
        if (moved < still_rad)
        {
            return true;
        }
        before = std::move(after);
    }
    return false;
}

bool G1ManipulationServer::closeHandOn(
    MoveGroup& hand, const ArmContext& arm, double widest_m, double narrowest_m)
{
    const double range       = hand_span_open_m_ - hand_span_closed_m_;
    const auto   fraction_at = [&](double span) {
        return range > 0.0 ? std::clamp((hand_span_open_m_ - span) / range, 0.0, 1.0) : 1.0;
    };
    // Start outside the widest side, since perception tends to measure an object short, and never
    // wider than the hand takes.
    const double start =
        fraction_at(std::min(widest_m, grip_max_width_m_) + grip_start_margin_m_ - grip_preload_m_);
    // Stop a little past the narrowest side: with no contact by then the object is not between
    // the fingers, and closing further only sweeps it away.
    const double stop_span = std::max(hand_span_closed_m_, narrowest_m - grip_search_beyond_m_);
    const double stop      = std::max(start, fraction_at(stop_span));
    RCLCPP_INFO(
        get_logger(),
        "closing for an object %.0f to %.0f mm across, from %.0f%% of the way to closed, then "
        "feeling for contact",
        1000.0 * narrowest_m,
        1000.0 * widest_m,
        100.0 * start);

    // Nothing can feel contact without the grip check, so close straight to the object's size.
    if (!grip_check_enabled_)
    {
        return moveHandToFraction(hand, stop, "grasp");
    }
    const double step  = range > 0.0 ? grip_search_step_m_ / range : 0.0;
    const int    steps = step > 0.0 ? static_cast<int>(std::ceil((stop - start) / step)) : 0;
    std::string  why;
    for (int i = 0; i <= steps; ++i)
    {
        const double f = std::min(stop, start + (i * step));
        if (!moveHandToFraction(hand, f, "grasp"))
        {
            return false;
        }
        // A finger still travelling reads exactly like a blocked one.
        if (!waitForHandToSettle(hand))
        {
            RCLCPP_WARN(get_logger(), "the fingers had not stopped after %.1f s", hand_close_s_);
            continue;
        }
        // The thumb has to be among them: index and middle side by side oppose nothing.
        if (isHolding(arm, why, /*opposed=*/true))
        {
            RCLCPP_INFO(
                get_logger(),
                "contact at %.0f%% of the way to closed: %s",
                100.0 * f,
                why.c_str());
            // No squeeze past contact: on a round object it ejects rather than holds.
            holdCurrentGrip(hand);
            return true;
        }
    }
    RCLCPP_WARN(
        get_logger(),
        "no thumb-and-finger contact by %.0f mm, the object's size plus the search slop: %s",
        1000.0 * stop_span,
        why.c_str());
    return true;
}

bool G1ManipulationServer::moveToNamed(MoveGroup& group, const std::string& named_target)
{
    setStartStateInBounds(group);
    if (!group.setNamedTarget(named_target))
    {
        RCLCPP_ERROR(
            get_logger(),
            "'%s' is not a named pose of group '%s'",
            named_target.c_str(),
            group.getName().c_str());
        return false;
    }

    // Plan then execute, not move(): move() re-validates against every octomap update, and the
    // arm's own fresh voxels invalidate it constantly.
    MoveGroup::Plan plan;
    const auto      planned = planWithinBudget(group, plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "'%s': planning failed (%d)", named_target.c_str(), planned.val);
        return false;
    }
    return group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

moveit::core::MoveItErrorCode
G1ManipulationServer::planWithinBudget(MoveGroup& group, MoveGroup::Plan& plan)
{
    using Clock         = std::chrono::steady_clock;
    const auto deadline = Clock::now() + std::chrono::duration<double>(planning_time_s_);
    moveit::core::MoveItErrorCode result = moveit::core::MoveItErrorCode::FAILURE;
    for (int attempt = 1; attempt <= planning_attempts_; ++attempt)
    {
        const double remaining_s = std::chrono::duration<double>(deadline - Clock::now()).count();
        if (remaining_s <= 0.0)
        {
            break;
        }
        group.setPlanningTime(remaining_s);
        result = group.plan(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            break;
        }
        RCLCPP_WARN(
            get_logger(),
            "%s: plan %d of %d failed with %.1f s of budget left",
            group.getName().c_str(),
            attempt,
            planning_attempts_,
            std::chrono::duration<double>(deadline - Clock::now()).count());
    }
    group.setPlanningTime(planning_time_s_);
    return result;
}

G1ManipulationServer::MoveGroup* G1ManipulationServer::groupFor(const std::string& name)
{
    const auto it = groups_.find(name);
    return it == groups_.end() ? nullptr : it->second.get();
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
        cancelled(Pick::Feedback::PHASE_LIFT);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_LIFT;
    goal_handle->publish_feedback(feedback);
    // Straight up from where the hand closed. A free planner with the table exempt would drag
    // the object across it, so each retry starts a new line from wherever the last one stopped.
    geometry_msgs::msg::Pose lifted = descent_goal;
    lifted.position.z += lift_height_m_;
    double lift_fraction = 0.0;
    bool   lifted_some   = false;
    for (int attempt = 0; attempt < lift_attempts_; ++attempt)
    {
        lift_fraction = moveStraight(*arm_group, lifted, arm.grasp_frame, "lift", 0.0);
        lifted_some   = lifted_some || lift_fraction > 0.0;
        if (lift_fraction <= 0.0 || lift_fraction >= kLineComplete)
        {
            break;
        }
    }
    if (!lifted_some && !moveTo(*arm_group, lifted, arm.grasp_frame, "lift"))
    {
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
