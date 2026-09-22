/**
 * @file g1_manipulation_server_node.cpp
 * @brief The pick, place and posture action servers, planned and executed through MoveIt.
 */

#include "g1_manipulation/g1_manipulation_server_node.hpp"

#include <moveit/robot_state/attached_body.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <map>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
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

// A collision box never shrinks below this in any axis. MoveIt treats a zero-extent primitive
// as degenerate and the attach silently does nothing, which then reads as a planner that
// ignored the object.
constexpr double kMinPrimitiveExtent = 0.005;

rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

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
    // Unstamped, so RViz draws it at the latest transform rather than one it may not have yet.
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

GraspApproach graspApproachFrom(const std::string& name)
{
    // Anything unrecognised is Top, which is what every scene here was tuned against. A typo
    // should not silently change which face the hand comes in on.
    return name == "front" ? GraspApproach::kFront : GraspApproach::kTop;
}

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

G1ManipulationServer::~G1ManipulationServer()
{
    while (goals_running_.load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

/// The /objects array frame, read under the lock. Reaching around lookUpObject's snapshot to
/// touch objects_.header from a detached thread is a data race on a std::string.
std::string G1ManipulationServer::objectsFrame()
{
    const std::lock_guard<std::mutex> lock(objects_mutex_);
    return objects_.header.frame_id;
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

G1ManipulationServer::G1ManipulationServer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_manipulation_server", options)
{
    object_timeout_s_          = declare_parameter<double>("object_timeout_ms", 1000.0) / 1000.0;
    approach_height_m_         = declare_parameter<double>("approach_height_m", 0.22);
    place_approach_height_m_   = declare_parameter<double>("place_approach_height_m", 0.15);
    grasp_depth_below_top_m_   = declare_parameter<double>("grasp_depth_below_top_m", 0.020);
    min_grip_height_m_         = declare_parameter<double>("min_grip_height_m", 0.080);
    settle_tolerance_m_        = declare_parameter<double>("settle_tolerance_m", 0.010);
    max_grasp_offset_m_        = declare_parameter<double>("max_grasp_offset_m", 0.020);
    grasp_refresh_max_shift_m_ = declare_parameter<double>("grasp_refresh_max_shift_m", 0.050);
    settle_attempts_           = static_cast<int>(declare_parameter<int>("settle_attempts", 2));
    reaim_clearance_m_         = declare_parameter<double>("reaim_clearance_m", 0.08);
    settle_wait_s_             = declare_parameter<double>("settle_wait_s", 0.8);
    lift_height_m_             = declare_parameter<double>("lift_height_m", 0.15);
    lift_attempts_             = static_cast<int>(declare_parameter<int>("lift_attempts", 3));
    // What counts as a grip: how far short of the commanded posture a finger must stall, and how
    // hard it must push while short. Parameters because the real hand's tau_est carries noise
    // these thresholds have to clear.
    hand_close_s_                = declare_parameter<double>("hand_close_s", 1.5);
    grip_check_enabled_          = declare_parameter<bool>("grip_check_enabled", true);
    grip_min_position_error_rad_ = declare_parameter<double>("grip_min_position_error_rad", 0.08);
    grip_min_effort_nm_          = declare_parameter<double>("grip_min_effort_nm", 0.10);
    // How far a released object may be from where it was aimed before the place is a failure.
    place_tolerance_m_       = declare_parameter<double>("place_tolerance_m", 0.08);
    place_confirm_timeout_s_ = declare_parameter<double>("place_confirm_timeout_s", 4.0);
    octomap_rebuild_wait_s_  = declare_parameter<double>("octomap_rebuild_wait_s", 0.8);
    // Well under the joint limits' own 0.8 rad/s cap. Arm motion disturbs a standing humanoid
    // measurably, and slowing the whole path is preferred over clamping joints, which would
    // bend the path itself.
    velocity_scaling_  = declare_parameter<double>("velocity_scaling", 0.3);
    planning_time_s_   = declare_parameter<double>("planning_time_s", 5.0);
    planning_attempts_ = static_cast<int>(declare_parameter<int>("planning_attempts", 5));

    // How the right hand is held at the grasp; the left mirrors its roll. Where it grips is the
    // {side}_hand_grasp_frame link in the URDF, not here. The fingers close toward the palm's
    // +y, so the roll is what turns the closing axis down for a grasp off a table.
    grasp_rpy_ = declare_parameter<std::vector<double>>("grasp_rpy", { -M_PI_2, 0.0, 0.0 });
    front_grasp_rpy_ =
        declare_parameter<std::vector<double>>("front_grasp_rpy", { -M_PI_2, 0.0, M_PI_4 });
    grasp_approach_ = graspApproachFrom(declare_parameter<std::string>("grasp_approach", "top"));
    front_approach_standoff_m_ = declare_parameter<double>("front_approach_standoff_m", 0.18);
    front_grip_height_m_       = declare_parameter<double>("front_grip_height_m", 0.045);

    grasp_source_         = declare_parameter<std::string>("grasp_source", "fixed_top_down");
    grasp_timeout_s_      = declare_parameter<double>("grasp_timeout_s", 20.0);
    min_grasp_score_      = declare_parameter<double>("min_grasp_score", 0.5);
    max_grasp_candidates_ = static_cast<int>(declare_parameter<int>("max_grasp_candidates", 20));
    max_approach_tilt_rad_ =
        declare_parameter<double>("max_approach_tilt_deg", 75.0) * M_PI / 180.0;
    approach_standoff_m_ = declare_parameter<double>("approach_standoff_m", 0.12);
    ik_timeout_s_        = declare_parameter<double>("ik_timeout_s", 0.05);
    // How finely the straight-line approach is walked, and how much of it must be clear.
    cartesian_step_m_ = declare_parameter<double>("cartesian_step_m", cartesian_step_m_);
    cartesian_min_fraction_ =
        declare_parameter<double>("cartesian_min_fraction", cartesian_min_fraction_);

    const std::vector<double> offset = declare_parameter<std::vector<double>>(
        "graspgen_to_grasp_frame_xyz_rpy",
        std::vector<double>(graspgen_offset_.size(), 0.0));
    // Refused rather than padded: a short list read as identity rpy is a wrong grasp pose with
    // nothing pointing at why.
    if (offset.size() != graspgen_offset_.size())
    {
        throw std::invalid_argument(
            "graspgen_to_grasp_frame_xyz_rpy needs 6 numbers, xyz then rpy; got " +
            std::to_string(offset.size()));
    }
    std::copy(offset.begin(), offset.end(), graspgen_offset_.begin());

    // The grasp numbers are all measurements of a particular hand against a particular object,
    // so they get retuned between goals rather than between launches -- on hardware by hand, in
    // the suites by the test that has to make a grasp miss on purpose. Nothing structural is
    // here: only values a running goal already finished reading.
    parameters_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& updated) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : updated)
            {
                const std::string& name = parameter.get_name();
                if (name == "grasp_depth_below_top_m")
                {
                    grasp_depth_below_top_m_ = parameter.as_double();
                }
                else if (name == "min_grip_height_m")
                {
                    min_grip_height_m_ = parameter.as_double();
                }
                else if (name == "settle_tolerance_m")
                {
                    settle_tolerance_m_ = parameter.as_double();
                }
                else if (name == "settle_attempts")
                {
                    settle_attempts_ = static_cast<int>(parameter.as_int());
                }
                else if (name == "grasp_approach")
                {
                    // Settable while the node runs, so the two approaches can be compared in one
                    // session against the same scene rather than across two rebuilds.
                    grasp_approach_ = graspApproachFrom(parameter.as_string());
                }
                else if (name == "front_grip_height_m")
                {
                    front_grip_height_m_ = parameter.as_double();
                }
                else if (name == "front_approach_standoff_m")
                {
                    front_approach_standoff_m_ = parameter.as_double();
                }
                else if (name == "reaim_clearance_m")
                {
                    reaim_clearance_m_ = parameter.as_double();
                }
                else if (name == "settle_wait_s")
                {
                    settle_wait_s_ = parameter.as_double();
                }
                else if (name == "grip_check_enabled")
                {
                    grip_check_enabled_ = parameter.as_bool();
                }
                else if (name == "grip_min_position_error_rad")
                {
                    grip_min_position_error_rad_ = parameter.as_double();
                }
                else if (name == "grip_min_effort_nm")
                {
                    grip_min_effort_nm_ = parameter.as_double();
                }
            }
            return result;
        });

    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) { onObjects(msg); });

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(joint_states_mutex_);
            joint_states_ = *msg;
        });

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grasps_ = create_client<g1_msgs::srv::GenerateGrasps>(
        declare_parameter<std::string>("grasp_service", "/g1_grasp_engine/generate_grasps"));
    if (declare_parameter<bool>("publish_markers", false))
    {
        // Transient local: it is published once per pick, and RViz may connect after that.
        grasp_plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/grasp_plan",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }
    get_scene_     = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_scene_   = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    clear_octomap_ = create_client<std_srvs::srv::Empty>("/clear_octomap");
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
    if (links.empty())
    {
        return false;
    }

    if (!applyHandContact(
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
    // Deliberately the latest available rather than a stamp: the source's own stamp can be a
    // few sample periods old, and tf2 would then either extrapolate or refuse. The robot
    // stands still to manipulate, so latest is the right reading.
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

void G1ManipulationServer::initialize()
{
    if (front_grasp_rpy_.size() != 3)
    {
        throw std::runtime_error("front_grasp_rpy needs exactly 3 entries");
    }
    if (grasp_rpy_.size() != 3)
    {
        throw std::runtime_error("grasp_rpy needs exactly 3 entries");
    }

    // both_arms is named postures only: the mission tree tucks both arms with it, and pose goals
    // would need the subgroup IK map g1.srdf deliberately omits.
    for (const char* name : { "left_arm", "right_arm", "both_arms", "left_hand", "right_hand" })
    {
        auto group = std::make_shared<MoveGroup>(shared_from_this(), name);
        group->setMaxVelocityScalingFactor(velocity_scaling_);
        group->setMaxAccelerationScalingFactor(velocity_scaling_);
        group->setPlanningTime(planning_time_s_);
        groups_.emplace(name, group);
    }
    planning_frame_ = groups_.at("left_arm")->getPlanningFrame();

    // Servers come up only once the groups are usable. Advertising first would accept a goal
    // this node cannot yet act on, and the caller would see a timeout rather than a refusal.
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

void G1ManipulationServer::onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_ = *msg;
}

bool G1ManipulationServer::isHolding(const ArmContext& arm, std::string& why)
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
    // The posture the close was planned to, so the targets and the command cannot drift apart.
    const std::map<std::string, double> targets = hand->getNamedTargetValues("closed");

    sensor_msgs::msg::JointState measured;
    {
        std::lock_guard<std::mutex> lock(joint_states_mutex_);
        measured = joint_states_;
    }
    if (measured.name.size() != measured.position.size() ||
        measured.name.size() != measured.effort.size())
    {
        why = "joint states carry no effort, so a grip cannot be told from an empty hand";
        return false;
    }

    std::vector<JointGrip> fingers;
    fingers.reserve(targets.size());
    for (std::size_t i = 0; i < measured.name.size(); ++i)
    {
        const auto target = targets.find(measured.name[i]);
        if (target != targets.end())
        {
            fingers.push_back(
                { measured.name[i], target->second, measured.position[i], measured.effort[i] });
        }
    }
    if (fingers.size() != targets.size())
    {
        why = std::format(
            "only {} of the hand's {} joints are being reported",
            fingers.size(),
            targets.size());
        return false;
    }

    const GripVerdict verdict =
        verifyGrip(fingers, grip_min_position_error_rad_, grip_min_effort_nm_);
    why = verdict.why;
    return verdict.holding;
}

std::optional<vision_msgs::msg::Detection3D>
G1ManipulationServer::lookUpObject(const std::string& object_id)
{
    vision_msgs::msg::Detection3DArray snapshot;
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        snapshot = objects_;
    }

    // Age is judged here rather than at the source, because only the thing about to commit an
    // arm to a grasp knows how old a pose is too old. g1_object_pose_source deliberately
    // forwards the sample's own stamp so this check means something.
    const double age = (now() - rclcpp::Time(snapshot.header.stamp)).seconds();
    if (snapshot.detections.empty() || age > object_timeout_s_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "No usable object poses: %zu known, newest %.2f s old (limit %.2f). Is "
            "g1_object_pose_source active?",
            snapshot.detections.size(),
            age,
            object_timeout_s_);
        return std::nullopt;
    }

    for (const vision_msgs::msg::Detection3D& detection : snapshot.detections)
    {
        if (!detection.results.empty() && detection.results.front().hypothesis.class_id == object_id)
        {
            return detection;
        }
    }
    RCLCPP_ERROR(get_logger(), "No object called '%s' is being reported", object_id.c_str());
    return std::nullopt;
}

moveit_msgs::msg::CollisionObject G1ManipulationServer::publishCollisionObject(
    const vision_msgs::msg::Detection3D& detection,
    const geometry_msgs::msg::Pose&      in_planning_frame)
{
    moveit_msgs::msg::CollisionObject object;
    object.id              = detection.results.front().hypothesis.class_id;
    object.header.frame_id = planning_frame_;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    // A box for every object, whatever its real shape. The planner only needs a conservative
    // volume to route around, and the bounding box the pose source reports is exactly that.
    primitive.dimensions = { std::max(detection.bbox.size.x, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.y, kMinPrimitiveExtent),
                             std::max(detection.bbox.size.z, kMinPrimitiveExtent) };

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(in_planning_frame);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // ADD on an id that already exists replaces it, so a re-plan against a moved object does
    // not need a remove first.
    planning_scene_.applyCollisionObjects({ object });
    // Returned so the attach can rebuild the same geometry: by then the world copy has been
    // removed, and an attached body has to carry its own shape.
    return object;
}

geometry_msgs::msg::Pose G1ManipulationServer::graspFrameGoal(
    const geometry_msgs::msg::Pose& object_pose, double object_height_m, const ArmContext& arm,
    GraspApproach approach) const
{
    // Horizontally the grasp frame goes straight to the object: that frame is defined as the
    // point the hand closes on, so putting it at the object IS the grasp.
    geometry_msgs::msg::Pose goal;
    goal.position = object_pose.position;

    const double top    = object_pose.position.z + 0.5 * object_height_m;
    const double bottom = object_pose.position.z - 0.5 * object_height_m;

    // Where up the object the grip lands, which is not the same question for the two approaches.
    //
    // Coming down on the top face, it aims just under that face so the fingers close around the
    // object rather than on top of it, but never nearer the base than the hand itself reaches:
    // measured in MuJoCo, the Dex3 hangs 63 mm below its grasp frame even with the thumb
    // retracted, so a grip point below that rests the thumb on the surface instead of the object.
    //
    // Coming in on the front face there is no top face to stay under, and the useful height is
    // simply a stated distance up from the base. Held on its side the object is gripped across
    // its width with its long axis across the fingers rather than along them.
    const auto& rpy = approach == GraspApproach::kFront ? front_grasp_rpy_ : grasp_rpy_;
    goal.position.z = approach == GraspApproach::kFront ?
                          bottom + front_grip_height_m_ :
                          std::max(top - grasp_depth_below_top_m_, bottom + min_grip_height_m_);

    // The orientation mirrors either way: the two hands close in opposite directions, so the
    // roll that points the closing axis flips sign.
    tf2::Quaternion rotation;
    rotation.setRPY((arm.is_left ? -1.0 : 1.0) * rpy[0], rpy[1], rpy[2]);
    goal.orientation = tf2::toMsg(rotation);
    return goal;
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
        why = "the grasp generator did not answer within " + std::to_string(grasp_timeout_s_) + "s";
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
    // Moved: the response carries every candidate pose and score, and this is its last reader.
    return std::move(*response);
}

std::optional<G1ManipulationServer::GraspPlan> G1ManipulationServer::chooseGrasp(
    const vision_msgs::msg::Detection3D& detection, const geometry_msgs::msg::Pose& object_pose,
    const ArmContext& arm, std::string& why, std::vector<ConsideredGrasp>* verdicts)
{
    const auto record =
        [verdicts](const geometry_msgs::msg::Pose& pose, ConsideredGrasp::Verdict verdict) {
            if (verdicts != nullptr)
            {
                verdicts->push_back({ pose, verdict });
            }
        };
    if (grasp_source_ != "generated")
    {
        GraspPlan plan;
        plan.grasp    = graspFrameGoal(object_pose, detection.bbox.size.z, arm, grasp_approach_);
        plan.pregrasp = plan.grasp;
        if (grasp_approach_ == GraspApproach::kFront)
        {
            // Staged ABOVE the grip, not back from it, even though the fingers close
            // horizontally. Coming in along the object's own axis is what a front grasp means
            // geometrically, and it pushes the object away before the fingers reach it: the
            // block travelled 60 to 96 mm and ended off the bench in all three runs.
            //
            // Nothing about the wrist pose needs the approach to share its closing axis. The
            // fingers straddle the object front and back, so the hand can descend past it and
            // close at mid-height, which is the whole point of a front grip: front_grip_height_m
            // puts the grasp on the object's centre of mass, where min_grip_height_m cannot.
            plan.pregrasp.position.z += front_approach_standoff_m_;
            plan.origin = "front";
        }
        else
        {
            plan.pregrasp.position.z += approach_height_m_;
            plan.origin = "top-down";
        }
        return plan;
    }

    const std::string object_id =
        detection.results.empty() ? std::string() : detection.results.front().hypothesis.class_id;
    const auto response = requestGrasps(object_id, arm, why);
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

    int considered = 0;
    int reachable  = 0;
    for (std::size_t i = 0; i < response->grasps.size(); ++i)
    {
        if (considered >= max_grasp_candidates_)
        {
            break;
        }
        if (i < response->scores.size() && response->scores[i] < min_grasp_score_)
        {
            // Sorted best first, so the first one under the bar ends the list.
            break;
        }
        ++considered;

        const auto in_planning_frame =
            toPlanningFrame(response->grasps[i], response->header.frame_id);
        if (!in_planning_frame)
        {
            continue;
        }
        // On the generator's own pose: its +z is the approach, and past the limit the hand is
        // reaching up through whatever the object rests on.
        if (approachTiltRad(*in_planning_frame) > max_approach_tilt_rad_)
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kTilted);
            continue;
        }

        const geometry_msgs::msg::Pose goal =
            applyGripperOffset(*in_planning_frame, graspgen_offset_, arm.is_left);
        // Reset per solve: a failed setFromIK leaves the state where it gave up.
        attempt = *state;
        if (!attempt.setFromIK(jmg, goal, arm.grasp_frame, ik_timeout_s_))
        {
            record(*in_planning_frame, ConsideredGrasp::Verdict::kUnreachable);
            continue;
        }
        ++reachable;
        record(*in_planning_frame, ConsideredGrasp::Verdict::kChosen);

        const std::array<double, 3> axis = approachAxis(*in_planning_frame);
        GraspPlan                   plan;
        plan.grasp    = goal;
        plan.pregrasp = goal;
        // Back along the approach, not straight up: a side grasp has a ceiling above it.
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

    why = "none of the " + std::to_string(considered) + " candidates considered were usable (" +
          std::to_string(reachable) + " reachable) out of " +
          std::to_string(response->grasps.size()) + " offered";
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

    // Each candidate ends where its gripper frame is, along the direction the hand travels in.
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
        // The goal after the gripper offset: this is where the grasp frame is actually sent.
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
    // The planner aborts if a start joint sits outside its URDF limit by any margin, and no
    // adapter clamps one back. Measured after a grasp: right_shoulder_yaw at -2.61874 against a
    // -2.618 limit, 0.74 mrad of tracking error on a joint IK put exactly on the stop.
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

bool G1ManipulationServer::moveAlongApproach(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    // A straight line, not a plan: the last centimetres above a table are a corridor a sampling
    // planner burns its budget failing to thread. MoveIt's own pick pipeline does the same.
    const double fraction = moveStraight(group, pose, link, what, cartesian_min_fraction_);
    if (fraction < cartesian_min_fraction_)
    {
        RCLCPP_WARN(
            get_logger(),
            "%s: a straight line covered %.0f%% of the way, under the %.0f%% needed; planning "
            "around instead",
            what.c_str(),
            fraction * 100.0,
            cartesian_min_fraction_ * 100.0);
        return moveTo(group, pose, link, what);
    }
    if (fraction < 1.0)
    {
        // Accepted short, but in metres and at WARN: how far the hand stops short is what
        // decides whether it closed on the object, and a percentage hides that.
        RCLCPP_WARN(
            get_logger(),
            "%s: stopping %.1f cm short of the grasp; the hand closes from there",
            what.c_str(),
            (1.0 - fraction) * approach_standoff_m_ * 100.0);
    }
    return true;
}

void G1ManipulationServer::clearOctomap()
{
    if (!clear_octomap_->wait_for_service(std::chrono::milliseconds(200)))
    {
        RCLCPP_WARN(get_logger(), "no /clear_octomap; descending against whatever the map holds");
        return;
    }
    // Waited on, not fired and forgotten. The reply carries nothing, but it is the only signal
    // that the map is actually empty, and the caller plans against that map on the very next
    // line: without the wait the descent races the clear and sometimes walks 0 % of its line
    // against voxels that were still there.
    auto cleared =
        clear_octomap_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    if (cleared.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_WARN(get_logger(), "/clear_octomap did not answer; the map may still hold voxels");
    }
    // Then wait for the sweep to put the world back. Nothing in this scene is a collision object:
    // measured, the planning scene holds zero of them and 3.6 MB of octomap, so the clear takes
    // the table and the bench with it, not just the arm's own stale voxels. Planning in that
    // window is planning blind, and the arm drives through whatever was there. One update at the
    // updater's 5 Hz is enough to get the visible surfaces back, and the stale voxels do not
    // return because nothing is looking at where the arm used to be.
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(octomap_rebuild_wait_s_)));
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
    // Reported rather than executed when it falls short, so a caller with somewhere else to go
    // still has the arm where it left it.
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
    // execute() returns when the controller says the trajectory is finished, which is not when
    // the arm has stopped: it is still sagging into its steady-state error for about a second
    // after. Measuring or planning from there reads a pose the arm has not reached, which came
    // out as corrections chasing a residual that was not real and Cartesian paths that gave up
    // three points in.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(settle_wait_s_ * 1000.0)));
    return fraction;
}

geometry_msgs::msg::Pose G1ManipulationServer::stagingPose(
    const geometry_msgs::msg::Pose& pregrasp, const geometry_msgs::msg::Pose& grasp) const
{
    const double axis_length = std::sqrt(
        std::pow(pregrasp.position.x - grasp.position.x, 2) +
        std::pow(pregrasp.position.y - grasp.position.y, 2) +
        std::pow(pregrasp.position.z - grasp.position.z, 2));
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

bool G1ManipulationServer::descendOnto(
    MoveGroup& group, const geometry_msgs::msg::Pose& pregrasp,
    const geometry_msgs::msg::Pose& grasp, const std::string& link)
{
    // Every descent starts from the same staging point, reaim_clearance_m back up the approach
    // axis, and only the last stretch from there is a straight line.
    //
    // Both halves of that matter. Getting to staging may be planned, because staging is above
    // the top of anything this hand can grip and a planner routing around up there cannot sweep
    // the object off the table -- which is exactly what a planned path all the way to the grasp
    // did to a 60 mm cylinder. And the straight line is short, so it tends to walk: the full
    // approach is 12 to 22 cm and runs out part way often enough that retrying it just repeats
    // itself.
    if (stagingPose(pregrasp, grasp).position == grasp.position)
    {
        RCLCPP_ERROR(get_logger(), "approach: the pregrasp and the grasp are the same pose");
        return false;
    }

    geometry_msgs::msg::Pose commanded = grasp;
    for (int attempt = 0; attempt <= settle_attempts_; ++attempt)
    {
        const geometry_msgs::msg::Pose staging = stagingPose(pregrasp, commanded);
        // Planned, like the first trip executePick makes. Tried as a straight line on the
        // argument that it cannot route through the object, which by this point has been removed
        // from the scene: that took the tabletop pick from 10 of 10 to 6 of 10, the truncation
        // coming straight back on the retries. The correctness argument lost to the measurement.
        //
        // Best effort. If it cannot get there the descent below starts from higher up, which is
        // where it started before there was a staging point at all.
        if (!moveTo(group, staging, link, "re-stage"))
        {
            RCLCPP_WARN(get_logger(), "approach: could not re-stage; descending from here");
        }
        const double walked   = moveStraight(group, commanded, link, "approach", 0.0);
        const auto   residual = residualTo(group, grasp, link, "approach");
        if (!residual)
        {
            return false;
        }
        const double error = std::sqrt(
            residual->x * residual->x + residual->y * residual->y + residual->z * residual->z);
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
            // Close from here only if the object could still be between the fingers. The hand
            // takes nothing wider than 75 mm and the blocks are 60, so a few millimetres is
            // slack and anything more is air: measured on a failed pick, the descent ended with
            // the block 115 mm away in the palm's own frame and the fingers swept shut through
            // nothing, which costs an attempt and tells the tree the grasp was tried.
            if (error > max_grasp_offset_m_)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "approach: %.0f mm off after %d descents, past the %.0f mm the hand can still "
                    "close over; not closing on air",
                    error * 1000.0,
                    attempt + 1,
                    max_grasp_offset_m_ * 1000.0);
                return false;
            }
            RCLCPP_WARN(
                get_logger(),
                "approach: still %.0f mm off after %d descents; closing from there",
                error * 1000.0,
                attempt + 1);
            return true;
        }
        // Overshoot only what the arm failed to hold. A line that ran out early left the rest of
        // itself unwalked, and adding THAT to the target aims the hand through the table: the
        // next descent then stops in the same place and the loop chases its own tail.
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
                "approach: the straight line ran out %.0f%% in, %.0f mm short; going back up to "
                "start it again rather than aiming lower",
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
    // A position-only arm does not stop where it was told. Against gravity the G1's shoulder
    // settles about 0.09 rad short at kp 40, which is 40 mm at the hand -- more than the whole
    // grip. Re-command the residual instead of raising gains, which the balance controller
    // shares. The same error exists on hardware, so this stays in the real pipeline.
    geometry_msgs::msg::Pose commanded = pose;
    // One more measurement than nudges: the last nudge has to be checked too, or a hand that
    // arrived is reported as one that gave up.
    for (int attempt = 0; attempt <= settle_attempts_; ++attempt)
    {
        const auto residual = residualTo(group, pose, link, what);
        if (!residual)
        {
            return false;
        }
        const double dx    = residual->x;
        const double dy    = residual->y;
        const double dz    = residual->z;
        const double error = std::sqrt(dx * dx + dy * dy + dz * dz);
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
        // Overshoot by the residual: the next target droops by roughly the same amount, so the
        // settled pose lands on the one that was asked for.
        commanded.position.x += dx;
        commanded.position.y += dy;
        commanded.position.z += dz;
        RCLCPP_INFO(
            get_logger(),
            "%s: %.0f mm short, nudging (%+.0f %+.0f %+.0f) mm",
            what.c_str(),
            error * 1000.0,
            dx * 1000.0,
            dy * 1000.0,
            dz * 1000.0);
        // A straight line only. Re-planning a two-centimetre correction throws away the
        // configuration the arm is already in, and the planner needs a budget it does not have
        // left by this point; a nudge that cannot be walked in a line is a nudge not worth
        // making.
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
        "%s: gave up nudging after %d attempts; closing from where it is",
        what.c_str(),
        settle_attempts_);
    return true;
}

bool G1ManipulationServer::moveTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    setStartStateInBounds(group);
    // Named explicitly rather than relying on the group's default tip: the goal is for the
    // grasp frame, which hangs off the palm and is not what the group ends at.
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
            "%s: execution failed (%d). The usual cause is the arm not being acquired -- run "
            "g1_bringup's activate_arm first.",
            what.c_str(),
            executed.val);
        return false;
    }
    return true;
}

bool G1ManipulationServer::moveHandTo(MoveGroup& hand, const std::string& named_target)
{
    // Commanded, not planned. A hand closing on an object is intentional contact, and asking a
    // motion planner to approve it needs a collision-free start state the hand does not have:
    // down at the grasp it is inside the octomap by construction, and planning fails in 25 ms
    // with "start state in collision" however the fingers are actually placed. MoveIt's own pick
    // pipeline does the same thing, carrying the gripper posture as a trajectory to execute
    // rather than a goal to plan for.
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

    // Two points, and the first one has to be where the fingers are now: the execution manager
    // rejects a trajectory that does not start within allowed_start_tolerance of the current
    // state, which a bare target point never does.
    const std::vector<std::string> joints  = hand.getActiveJoints();
    const std::vector<double>      current = hand.getCurrentJointValues();
    if (joints.size() != current.size())
    {
        RCLCPP_ERROR(
            get_logger(),
            "%s reports %zu joints but %zu positions",
            hand.getName().c_str(),
            joints.size(),
            current.size());
        return false;
    }

    trajectory_msgs::msg::JointTrajectoryPoint from;
    trajectory_msgs::msg::JointTrajectoryPoint to;
    moveit_msgs::msg::RobotTrajectory          path;
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const auto target = targets.find(joints[i]);
        if (target == targets.end())
        {
            RCLCPP_ERROR(
                get_logger(),
                "'%s' does not name %s",
                named_target.c_str(),
                joints[i].c_str());
            return false;
        }
        path.joint_trajectory.joint_names.push_back(joints[i]);
        from.positions.push_back(current[i]);
        to.positions.push_back(target->second);
    }
    from.velocities.assign(from.positions.size(), 0.0);
    to.velocities.assign(to.positions.size(), 0.0);
    from.time_from_start         = rclcpp::Duration::from_seconds(0.0);
    to.time_from_start           = rclcpp::Duration::from_seconds(hand_close_s_);
    path.joint_trajectory.points = { from, to };

    MoveGroup::Plan plan;
    plan.trajectory = path;
    if (hand.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "'%s': the hand would not execute", named_target.c_str());
        return false;
    }
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

    // Plan then execute, not move(): move() re-checks the remaining path against every
    // planning-scene update, and against a live octomap the arm's own freshly integrated voxels
    // invalidate it constantly. Everything here is still fully collision-checked at plan time.
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

    // Every failure below leaves the hand open and nothing attached, so a retry starts from a
    // defined state rather than part-way into a grasp. That is what makes the behavior tree's
    // retry meaningful rather than a replay.
    const auto fail = [&](const std::string& phase, const std::string& why) {
        moveHandTo(*hand_group, "pinch_ready");
        // Back out before the exemptions go, not after. A grasp that fails leaves the hand at
        // table level with the octomap exempted for it; restoring the ACM there leaves the arm
        // standing in collision, and every plan the retry asks for then fails on its start state
        // in milliseconds rather than for anything a second attempt could fix.
        geometry_msgs::msg::Pose clear = arm_group->getCurrentPose(arm.grasp_frame).pose;
        clear.position.z += reaim_clearance_m_;
        moveStraight(*arm_group, clear, arm.grasp_frame, "abort retreat", 0.0);
        arm_group->detachObject(goal->object_id);
        setHandContact(arm, { "<octomap>", goal->object_id }, false);
        // And its collision object, which the approach would have removed. Left behind, it sits
        // at a pose nothing refreshes and the next plan starts in collision with it.
        planning_scene_.removeCollisionObjects({ goal->object_id });
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
    };
    // Same cleanup as a failure, but reported as a cancel so the tree can tell the two apart.
    const auto cancelled = [&](const std::string& phase) {
        moveHandTo(*hand_group, "pinch_ready");
        arm_group->detachObject(goal->object_id);
        setHandContact(arm, { "<octomap>", goal->object_id }, false);
        planning_scene_.removeCollisionObjects({ goal->object_id });
        result->success = false;
        result->message = phase + ": cancelled";
        goal_handle->canceled(result);
    };

    feedback->phase = Pick::Feedback::PHASE_LOCATING;
    goal_handle->publish_feedback(feedback);
    const auto detection = lookUpObject(goal->object_id);
    if (!detection)
    {
        fail(Pick::Feedback::PHASE_LOCATING, "no fresh pose for '" + goal->object_id + "'");
        return;
    }
    // /objects is in odom and the planner works in pelvis; the two differ by wherever the
    // robot is standing, so every measured pose goes through TF before it is planned against.
    const std::string object_frame =
        detection->header.frame_id.empty() ? objectsFrame() : detection->header.frame_id;
    const auto object_pose = toPlanningFrame(detection->results.front().pose.pose, object_frame);
    if (!object_pose)
    {
        fail(Pick::Feedback::PHASE_LOCATING, "could not transform the object pose");
        return;
    }
    const moveit_msgs::msg::CollisionObject object =
        publishCollisionObject(*detection, *object_pose);

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
        // No fallback to the fixed grasp: quietly not using the generator it was told to use is
        // the failure the pose source's refusal exists to prevent.
        fail(Pick::Feedback::PHASE_LOCATING, "no usable grasp: " + why);
        return;
    }
    RCLCPP_INFO(
        get_logger(),
        "picking '%s' with the %s grasp",
        goal->object_id.c_str(),
        chosen->origin.c_str());
    const geometry_msgs::msg::Pose grasp_goal    = chosen->grasp;
    const geometry_msgs::msg::Pose pregrasp_goal = chosen->pregrasp;

    // A cancel is accepted by the server, so it has to be honoured somewhere: between phases is
    // the only safe place, because a trajectory already executing cannot be unwound here.
    if (goal_handle->is_canceling())
    {
        cancelled(Pick::Feedback::PHASE_PREGRASP);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_PREGRASP;
    goal_handle->publish_feedback(feedback);
    // pinch_ready, not open: the fingers have to be out of the way before the approach, but an
    // open thumb hangs 127 mm below the palm and grounds out on the table before the hand
    // arrives. Reported separately from the arm move because the causes differ, a hand that will
    // not open being an unpowered Dex3 where an arm that will not reach is geometry.
    if (!moveHandTo(*hand_group, "pinch_ready"))
    {
        fail(Pick::Feedback::PHASE_PREGRASP, "the hand would not open to pinch_ready");
        return;
    }
    if (!moveTo(*arm_group, pregrasp_goal, arm.grasp_frame, "pregrasp"))
    {
        fail(Pick::Feedback::PHASE_PREGRASP, "could not reach the pregrasp pose");
        return;
    }
    // Take the arm's droop out up here, in clear air. Correcting it down at the grasp instead
    // means dragging the hand sideways through the object it is about to pick up.
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

    // The grasp pose was measured before the arm moved and is expressed in the pelvis. Pinned,
    // it is still true when the descent starts. Standing on its legs it is not: the balance
    // controller moves the pelvis under the robot while the arm reaches the pregrasp, and a
    // frozen pelvis-frame target then points at where the object used to be. Measured on the
    // navigation route, the descent landed 6 mm from its commanded pose and closed on nothing,
    // having swept the block off the desk on the way down.
    //
    // Re-measuring in clear air at the pregrasp costs one detection and corrects the drift. Only
    // the grasp moves, by the object's own shift: the chosen grasp's orientation and strategy
    // are still the right ones, and the approach axis should tilt to aim from where the hand is
    // at where the object now is.
    geometry_msgs::msg::Pose descent_goal = grasp_goal;
    if (const auto fresh = lookUpObject(goal->object_id))
    {
        const std::string fresh_frame =
            fresh->header.frame_id.empty() ? objectsFrame() : fresh->header.frame_id;
        if (const auto fresh_pose = toPlanningFrame(fresh->results.front().pose.pose, fresh_frame))
        {
            const double dx    = fresh_pose->position.x - object_pose->position.x;
            const double dy    = fresh_pose->position.y - object_pose->position.y;
            const double dz    = fresh_pose->position.z - object_pose->position.z;
            const double shift = std::sqrt(dx * dx + dy * dy + dz * dz);
            // A shift this large is a different object or a bad frame, not body sway, and
            // following it would fling the hand across the table.
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
                RCLCPP_INFO(
                    get_logger(),
                    "approach: re-aimed %.0f mm for body sway since the object was located",
                    shift * 1000.0);
            }
        }
    }

    // Contact allowed only now, for the last few centimetres: an exemption held all skill long
    // lets a plan route straight through the table. The object is removed rather than exempted,
    // following MoveIt's remove-close-attach.
    // Stage BEFORE the object leaves the planning scene. Staging is planned, and a planner only
    // avoids what the scene contains: removed first, the path is free to route straight through
    // the object it is reaching for. Measured on the deliberate-miss test, which checks that a
    // grasp aimed wrongly is reported without disturbing anything: the block was shifted 181 mm.
    // descendOnto stages again on each attempt, by then a short move from close by.
    if (!moveTo(*arm_group, stagingPose(pregrasp_goal, descent_goal), arm.grasp_frame, "stage"))
    {
        RCLCPP_WARN(get_logger(), "approach: could not stage above the object; descending anyway");
    }

    planning_scene_.removeCollisionObjects({ goal->object_id });
    // Drop the map first, THEN exempt the hand from it. Clearing removes the octomap as a world
    // object and takes its allowed-collision entries with it, so an exemption set beforehand is
    // gone by the time the rebuilt map exists. Measured: the descent's own Cartesian line died
    // 3 % in on `<octomap> <-> right_hand_thumb_2_link`, which is the thumb passing the object it
    // is reaching for, the one contact this exemption is entirely about.
    clearOctomap();
    setHandContact(arm, { "<octomap>" }, true);

    if (!descendOnto(*arm_group, pregrasp_goal, descent_goal, arm.grasp_frame))
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
    if (!moveHandTo(*hand_group, "closed"))
    {
        fail(Pick::Feedback::PHASE_GRASP, "the hand did not close");
        return;
    }

    // The controller reports a finger blocked by the object as success, so ask the fingers
    // themselves before telling the planner it is held.
    std::string grip;
    if (!isHolding(arm, grip))
    {
        fail(Pick::Feedback::PHASE_GRASP, "the hand closed on nothing: " + grip);
        return;
    }
    // Built explicitly, not attachObject(id, link), which promotes an object still in the world;
    // this one was removed. touch_links stops the attach itself reading as a collision.
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name        = arm.palm_link;
    attached.object           = object;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // The hand's links plus the palm and the wrist: a cube gripped just under its top face
    // reaches past the fingers to touch the wrist, and without it every later plan starts with
    // the attached object already in collision. Same set allowHandContact() exempts.
    attached.touch_links =
        hand_group->getRobotModel()->getJointModelGroup(arm.hand_group)->getLinkModelNames();
    attached.touch_links.push_back(arm.palm_link);
    const std::string touch_side = arm.is_left ? "left" : "right";
    attached.touch_links.push_back(touch_side + "_wrist_pitch_link");
    attached.touch_links.push_back(touch_side + "_wrist_yaw_link");
    attached.touch_links.push_back(touch_side + "_wrist_roll_link");
    planning_scene_.applyAttachedCollisionObject(attached);

    // Extended to the object now that it is attached and about to be lifted out of the surface.
    // The ShapeMask stops new clouds re-adding it but does not erase the voxels already there,
    // so without this the lift starts in collision against the table it is still sitting on.
    setHandContact(arm, { "<octomap>", goal->object_id }, true);

    if (goal_handle->is_canceling())
    {
        cancelled(Pick::Feedback::PHASE_LIFT);
        return;
    }
    feedback->phase = Pick::Feedback::PHASE_LIFT;
    goal_handle->publish_feedback(feedback);
    geometry_msgs::msg::Pose lifted = grasp_goal;
    lifted.position.z += lift_height_m_;
    // Straight up, not planned. The octomap holding the table is exempted for the hand at this
    // point, so a free planner is free to route the lift sideways THROUGH the table, which drags
    // the object along the surface and off it: measured, 43 mm, which was enough to leave the
    // block overhanging the near edge.
    //
    // Repeated only while the LINE itself is short. What truncates an attempt is usually below
    // the hand, so a second line started from the new height gets further, and a carry begun from
    // a short lift drags the held object across the table.
    //
    // Judged on the Cartesian fraction, not on where the arm ends up. Measured: the arm finishes
    // 39 mm below the target whether the target is 200 mm or 150 mm, because this arm is
    // position-only and sags about 0.09 rad at the shoulder under load. That sag is not a short
    // line and cannot be retried away; reading it as one spent every attempt, every run.
    double lift_fraction = 0.0;
    for (int attempt = 0; attempt < lift_attempts_; ++attempt)
    {
        lift_fraction = moveStraight(*arm_group, lifted, arm.grasp_frame, "lift", 0.0);
        if (lift_fraction <= 0.0 || lift_fraction >= 0.99)
        {
            break;
        }
    }
    if (lift_fraction <= 0.0 && !moveTo(*arm_group, lifted, arm.grasp_frame, "lift"))
    {
        fail(Pick::Feedback::PHASE_LIFT, "could not lift clear of the surface");
        return;
    }
    // Both numbers, because they mean different things and only one is actionable: a short line
    // is an obstacle the carry will meet again, while the gap to the target is mostly the arm's
    // own sag and is the same 39 mm whatever the target.
    if (const auto residual = residualTo(*arm_group, lifted, arm.grasp_frame, "lift");
        residual.has_value())
    {
        RCLCPP_INFO(
            get_logger(),
            "lift: walked %.0f%% of the line, ended %.0f mm below the %.0f mm target",
            lift_fraction * 100.0,
            std::abs(residual->z) * 1000.0,
            lift_height_m_ * 1000.0);
    }
    // Again after the lift: an object can be raked out of the hand on the way up, and the
    // planning scene would carry on believing it is held.
    if (!isHolding(arm, grip))
    {
        fail(Pick::Feedback::PHASE_LIFT, "the object was dropped during the lift: " + grip);
        return;
    }

    setHandContact(arm, { "<octomap>", goal->object_id }, false);
    // The hand goes back to respecting the map, but the object it is still holding does not,
    // because it maps itself: held in the air it is the clearest thing the sensor sees, nothing
    // excludes an attached body from the octomap, and it then collides with its own voxels. Place
    // already knew this; the pick handed the problem to whatever ran next, which is the posture
    // change before a walk. Measured: two navigation missions of three failed "could not reach
    // carry", and a plan from a post-lift state was refused on START_STATE_IN_COLLISION with
    // <octomap> against the carried block and three finger links.
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

    // Read from the scene, which is the authority on what the hand has, and filtered by this arm:
    // getAttachedObjects() covers the whole scene, and the detach below filters the same way.
    std::string held_id;
    double      held_height = 0.0;
    MoveGroup*  held_group  = groupFor(arm.arm_group);
    for (const auto& [id, attached] : planning_scene_.getAttachedObjects())
    {
        const bool mine = held_group != nullptr && held_group->getRobotModel()
                                                       ->getJointModelGroup(arm.arm_group)
                                                       ->hasLinkModel(attached.link_name);
        if (mine && !attached.object.primitives.empty() &&
            attached.object.primitives.front().dimensions.size() == 3)
        {
            held_id     = id;
            held_height = attached.object.primitives.front().dimensions[2];
            break;
        }
    }

    // The octomap AND the carried object: an attached body is its own collision entity, so
    // exempting the palm it hangs off does nothing for it.
    const std::vector<std::string> touchables =
        held_id.empty() ? std::vector<std::string>{ "<octomap>" } :
                          std::vector<std::string>{ "<octomap>", held_id };

    const auto fail = [&](const std::string& phase, const std::string& why) {
        setHandContact(arm, touchables, false);
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
    };
    const auto cancelled = [&](const std::string& phase) {
        setHandContact(arm, touchables, false);
        result->success = false;
        result->message = phase + ": cancelled";
        goal_handle->canceled(result);
    };

    // The carried object, and only it: include_links stays false so the ARM is still checked
    // against everything mapped. The object has to be exempt because it maps ITSELF -- held up in
    // the air it is the clearest thing the LiDAR can see, nothing excludes an attached body from
    // the octomap, and it then collides with its own voxels. Measured: without this every
    // preplace plan was refused in 4.5 ms with its whole budget untouched.
    //
    // This was removed once, on the theory that it was what let a carried block sweep the table.
    // It was not: that was the arm missing its path by 33 mm on the median and 394 at worst, and
    // it went away when the arm was stiffened, not when this was taken out.
    setHandContact(arm, touchables, true, /*include_links=*/false);

    // A surface from /objects beats the caller's coordinate: a tree writes its drop point in map,
    // and map->odom drift alone exceeds the arm's 0.04 m lateral window.
    std::optional<geometry_msgs::msg::Pose>      target;
    std::optional<vision_msgs::msg::Detection3D> surface;
    if (!goal->surface_object_id.empty())
    {
        // Given the same moment the landing check gets. The surface is scored against a phrase
        // like anything else, and the arm carrying the object across the table is standing in
        // front of it: measured on the bench here, 0.60 to 0.67 against a 0.50 threshold, so a
        // partial occlusion drops it under, and a phrase with no track has no bare-phrase alias
        // for this to resolve. It comes back on its own within a frame or two.
        const rclcpp::Time surface_deadline =
            now() + rclcpp::Duration::from_seconds(place_confirm_timeout_s_);
        while (!(surface = lookUpObject(goal->surface_object_id)) && now() < surface_deadline)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        if (!surface)
        {
            fail(
                Place::Feedback::PHASE_PREPLACE,
                "nothing called '" + goal->surface_object_id + "' on /objects");
            return;
        }
        const std::string frame =
            surface->header.frame_id.empty() ? objectsFrame() : surface->header.frame_id;
        target = toPlanningFrame(surface->results.front().pose.pose, frame);
    }
    else
    {
        // An empty frame means the goal is already in the planning frame. Anything else is
        // transformed rather than assumed: a target in odom treated as pelvis lands metres away,
        // and by the time that is visible the arm is already moving.
        target = toPlanningFrame(goal->pose.pose, goal->pose.header.frame_id);
    }
    if (!target)
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not transform the target pose");
        return;
    }

    // A detected surface reports its own centre, so the held object goes on TOP of it: half the
    // surface's height to reach its face, half the object's to stand it there. A caller-supplied
    // pose is where the object itself goes and needs neither.
    if (surface)
    {
        target->position.z += 0.5 * (surface->bbox.size.z + held_height);
    }

    // A set-down is always from above, whichever face the object was picked up by.
    geometry_msgs::msg::Pose place_goal =
        graspFrameGoal(*target, held_height, arm, GraspApproach::kTop);
    geometry_msgs::msg::Pose preplace = place_goal;
    preplace.position.z += place_approach_height_m_;

    // A cancel is accepted by the server, so it has to be honoured somewhere: between phases is
    // the only safe place, because a trajectory already executing cannot be unwound here.
    if (goal_handle->is_canceling())
    {
        cancelled(Place::Feedback::PHASE_PREPLACE);
        return;
    }
    feedback->phase = Place::Feedback::PHASE_PREPLACE;
    goal_handle->publish_feedback(feedback);
    // The map has rebuilt around the carry pose by now, and the arm's own links are not exempt
    // here, so the plan starts inside a ghost of the arm and fails on its start state in
    // milliseconds rather than for want of a route. Same clear the pick does before it approaches.
    clearOctomap();
    if (!moveTo(*arm_group, preplace, arm.grasp_frame, "preplace"))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not reach the pose above the target");
        return;
    }
    // The same droop the pick corrects for, and it matters as much here: releasing 40 mm off
    // target drops the object rather than setting it down.
    if (!settleOnPose(*arm_group, preplace, arm.grasp_frame, "preplace"))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "the hand would not settle above the target");
        return;
    }
    // Carried by friction, so it can be gone before it is ever released. Said plainly here
    // rather than reported later as a place that landed a metre away.
    std::string carried;
    if (!isHolding(arm, carried))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "the object was dropped on the way: " + carried);
        return;
    }

    // Re-resolved because the reach moves the base: a loaded arm shifts the COM and the gait
    // steps to keep up, measured at 0.165 m. `expected` holds it in the /objects frame, where the
    // base's own travel does not read as placement error.
    std::optional<geometry_msgs::msg::Point> expected;
    if (surface)
    {
        if (const auto fresh = lookUpObject(goal->surface_object_id))
        {
            const std::string frame =
                fresh->header.frame_id.empty() ? objectsFrame() : fresh->header.frame_id;
            expected = fresh->results.front().pose.pose.position;
            expected->z += 0.5 * (fresh->bbox.size.z + held_height);
            if (auto moved = toPlanningFrame(fresh->results.front().pose.pose, frame))
            {
                moved->position.z += 0.5 * (fresh->bbox.size.z + held_height);
                target             = moved;
                const auto regrasp = graspFrameGoal(*moved, held_height, arm, GraspApproach::kTop);
                const double shift = std::hypot(
                    regrasp.position.x - place_goal.position.x,
                    regrasp.position.y - place_goal.position.y);
                if (shift > 0.01)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "the base moved %.3f m during the reach; re-aiming",
                        shift);
                }
                place_goal = regrasp;
                // The retreat returns here, so it moves with the re-aim. Left stale, the lift is
                // diagonal by however far the base walked.
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
    // Now the hand may touch the surface: the descent ends in contact by definition. Cleared
    // first, for the reason the pick's descent gives: the clear drops the octomap's allowed
    // collisions along with the octomap.
    clearOctomap();
    setHandContact(arm, touchables, true);
    if (!descendOnto(*arm_group, preplace, place_goal, arm.grasp_frame))
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
    // Detached only after the hand is open, or the planner routes the arm through a volume the
    // object still occupies. Null when the state monitor has nothing yet, which unguarded is a
    // crash on a detached thread rather than a failed goal.
    const moveit::core::RobotStatePtr state = arm_group->getCurrentState();
    if (!state)
    {
        fail(Place::Feedback::PHASE_RELEASE, "no current state, cannot detach what was held");
        return;
    }
    std::vector<const moveit::core::AttachedBody*> attached;
    state->getAttachedBodies(attached, state->getJointModelGroup(arm.arm_group));
    for (const moveit::core::AttachedBody* body : attached)
    {
        arm_group->detachObject(body->getName());
    }

    if (goal_handle->is_canceling())
    {
        cancelled(Place::Feedback::PHASE_RETREAT);
        return;
    }
    feedback->phase = Place::Feedback::PHASE_RETREAT;
    goal_handle->publish_feedback(feedback);
    // Straight up, and still exempt from the object it just let go of. An open hand at the place
    // pose is wrapped around that object, so restoring its collision first makes the start state
    // invalid and nothing can be planned or walked from it at all. A vertical line cannot route
    // through what is below it, which is the only thing restoring it early was protecting
    // against, so the restore waits until the hand is clear.
    //
    // The map is cleared first for the same reason the preplace clears it: the exemption covers
    // the hand, not the forearm, and by now the map has filled in around an arm that has been
    // holding still over the surface through the lower and the release.
    clearOctomap();
    if (moveStraight(*arm_group, preplace, arm.grasp_frame, "retreat", 0.0) <= 0.0 &&
        !moveTo(*arm_group, preplace, arm.grasp_frame, "retreat"))
    {
        fail(Place::Feedback::PHASE_RETREAT, "could not retreat clear of the object");
        return;
    }

    setHandContact(arm, touchables, false);

    // A successful plan says nothing about where the object landed: one release short dropped the
    // block on the floor with every leaf reporting success. Checked against `expected` where a
    // surface gave one, so both sides come from /objects and the walking base cancels.
    if (!held_id.empty())
    {
        // Given a moment, because the object was occluded by the hand until it let go and the
        // detector needs a frame or two to pick it up again.
        std::optional<vision_msgs::msg::Detection3D> landed;
        const rclcpp::Time                           confirm_deadline =
            now() + rclcpp::Duration::from_seconds(place_confirm_timeout_s_);
        while (!(landed = lookUpObject(held_id)) && now() < confirm_deadline)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }
        // Not finding it is a failure, not a pass. The two ways this check used to be skipped,
        // an object missing from /objects and a pose that will not transform, are the exact
        // states a dropped object leaves behind: measured, a block on the floor is out of the
        // camera's view, so every leaf reported success on a block that never reached the box.
        if (!landed)
        {
            result->success = false;
            result->message = std::format(
                "{}: {} is not on /objects after the release, so where it landed cannot be "
                "confirmed",
                Place::Feedback::PHASE_RETREAT,
                held_id);
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
            return;
        }
        {
            const geometry_msgs::msg::Pose& pose = landed->results.front().pose.pose;
            // Binds without a temporary; `target` was already checked engaged above.
            const geometry_msgs::msg::Point& aim = expected ? *expected : target->position;

            std::optional<geometry_msgs::msg::Point> where = pose.position;
            if (!expected)
            {
                const std::string frame =
                    landed->header.frame_id.empty() ? objectsFrame() : landed->header.frame_id;
                const auto in_planning = toPlanningFrame(pose, frame);
                where = in_planning ? std::optional(in_planning->position) : std::nullopt;
            }
            if (!where)
            {
                result->success = false;
                result->message = std::format(
                    "{}: {} was found but its pose will not transform, so where it landed "
                    "cannot be confirmed",
                    Place::Feedback::PHASE_RETREAT,
                    held_id);
                RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
                goal_handle->abort(result);
                return;
            }
            {
                const double off = std::hypot(where->x - aim.x, where->y - aim.y, where->z - aim.z);
                // On a named surface the question is whether the object is ON it, which is a
                // footprint test rather than a distance one. A radius cannot answer it: an object
                // inside a container reads up to the container's own half-width off centre,
                // because the walls occlude it and the clipped mask drags the centroid toward the
                // rim. Measured, a block resting correctly on the box floor reported 83 mm out,
                // while a block abandoned on the table beside it was 258 mm out in y alone, so
                // the footprint separates them where a radius put both on the same side.
                bool landed_on_target = off <= place_tolerance_m_;
                if (!landed_on_target && surface)
                {
                    // Sideways only. The occlusion that biases the reading is the container's own
                    // walls cutting the mask, which moves the centroid across, not down, and the
                    // block stands 37 mm proud of this box so its top face is never hidden.
                    // Allowing the same slack in z let a block abandoned on the tabletop pass,
                    // 12 mm being all that separates the tabletop from the box floor.
                    const auto& extent = surface->bbox.size;
                    landed_on_target =
                        std::abs(where->x - aim.x) <= 0.5 * extent.x + place_tolerance_m_ &&
                        std::abs(where->y - aim.y) <= 0.5 * extent.y + place_tolerance_m_ &&
                        std::abs(where->z - aim.z) <= place_tolerance_m_;
                }
                if (!landed_on_target)
                {
                    result->success = false;
                    // Both positions, not just the gap: where it went says whether it was set
                    // down short, pushed aside, or dropped, and a single number says none of it.
                    result->message = std::format(
                        "{}: {} ended up {:.3f} m from where it was placed, at "
                        "({:.3f} {:.3f} {:.3f}) against ({:.3f} {:.3f} {:.3f})",
                        Place::Feedback::PHASE_RETREAT,
                        held_id,
                        off,
                        where->x,
                        where->y,
                        where->z,
                        aim.x,
                        aim.y,
                        aim.z);
                    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
                    goal_handle->abort(result);
                    return;
                }
                RCLCPP_INFO(
                    get_logger(),
                    "%s came to rest %.3f m from the target",
                    held_id.c_str(),
                    off);
            }
        }
    }

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

    // If this arm was carrying something, say whether it still is. Nothing between the pick and
    // the place looked, so a block shaken loose while the robot walked away from the bench was
    // reported three legs later as a place that could not start, and the mission had by then
    // crossed the building with an empty hand. Wherever it is lost, this is the first arm action
    // afterwards.
    ArmContext arm;
    const bool is_arm_group =
        resolveArm(goal->group.rfind("left", 0) == 0 ? "left" : "right", arm) &&
        goal->group == arm.arm_group;
    // Read from the monitored robot state, not from PlanningSceneInterface. The interface answers
    // over a service and lagged the release: after a place that genuinely worked it still listed
    // the block, so the tuck that ends the mission reported a drop and failed a run whose block
    // was already in the bench. executePlace detaches through this same state.
    std::vector<const moveit::core::AttachedBody*> carried;
    if (is_arm_group)
    {
        if (const moveit::core::RobotStatePtr state = group->getCurrentState())
        {
            state->getAttachedBodies(carried, state->getJointModelGroup(arm.arm_group));
        }
    }
    if (is_arm_group && !carried.empty())
    {
        if (std::string grip; !isHolding(arm, grip))
        {
            result->success = false;
            result->message = goal->group + " reached " + goal->named_target +
                              " but dropped what it held: " + grip;
            RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
            goal_handle->abort(result);
            return;
        }
    }

    result->success = true;
    result->message = goal->group + " is at " + goal->named_target;
    goal_handle->succeed(result);
}

}  // namespace g1_manipulation
