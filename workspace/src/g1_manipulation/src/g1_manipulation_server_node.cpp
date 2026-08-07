#include "g1_manipulation/g1_manipulation_server_node.hpp"

#include <moveit/robot_state/attached_body.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vector>

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
    object_timeout_s_  = declare_parameter<double>("object_timeout_ms", 1000.0) / 1000.0;
    approach_height_m_ = declare_parameter<double>("approach_height_m", 0.12);
    lift_height_m_     = declare_parameter<double>("lift_height_m", 0.15);
    // Well under the joint limits' own 0.8 rad/s cap. Arm motion disturbs a standing humanoid
    // measurably (docs/notes/arm-motion-and-balance.md), and slowing the whole path is
    // preferred over clamping joints, which would bend the path itself.
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.3);
    planning_time_s_  = declare_parameter<double>("planning_time_s", 5.0);

    // How the hand is held at the grasp, for the RIGHT hand; the left mirrors its roll.
    //
    // WHERE the hand grips is not here: it is the {side}_hand_grasp_frame link in the URDF,
    // which is a property of the Dex3's geometry rather than of this task. Goals are given for
    // that frame, so nothing in this file adds an offset to a palm pose.
    //
    // The fingers close toward the palm's +y, so it is the ROLL that turns the closing axis
    // downward for a grasp off a table, and the pitch that would be wrong.
    grasp_rpy_ = declare_parameter<std::vector<double>>("grasp_rpy", { -M_PI_2, 0.0, 0.0 });

    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "/objects",
        objectsQos(),
        std::bind(&G1ManipulationServer::onObjects, this, std::placeholders::_1));

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    get_scene_   = create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    apply_scene_ = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
}

bool G1ManipulationServer::allowHandContact(
    const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed)
{
    // The links that unavoidably enter occupied space during a grasp: the hand itself, and
    // the wrist that carries it. Taken from the robot model rather than listed, so a renamed
    // link is a build-time problem rather than a silently ineffective exemption.
    MoveGroup* hand = groupFor(arm.hand_group);
    if (hand == nullptr)
    {
        return false;
    }
    std::vector<std::string> links =
        hand->getRobotModel()->getJointModelGroup(arm.hand_group)->getLinkModelNames();
    const std::string side = arm.is_left ? "left" : "right";
    links.push_back(arm.palm_link);
    links.push_back(side + "_wrist_pitch_link");
    links.push_back(side + "_wrist_yaw_link");

    // The current matrix has to be read first: ApplyPlanningScene replaces the whole ACM
    // rather than merging into it, so sending only our entries would drop every
    // self-collision rule the SRDF set up.
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = get_scene_->async_send_request(request);
    // Waited on WITHOUT spinning: the executor already owns this node, and spinning it from
    // here would re-enter the executor from inside its own callback.
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "/get_planning_scene did not answer");
        return false;
    }

    moveit_msgs::msg::AllowedCollisionMatrix acm = future.get()->scene.allowed_collision_matrix;

    // A name the matrix has never seen has to be added as a full row and column first,
    // otherwise the indices below run off the end.
    const auto ensure_entry = [&acm](const std::string& name) {
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
    };
    const auto index_of = [&acm](const std::string& name) {
        return static_cast<std::size_t>(
            std::find(acm.entry_names.begin(), acm.entry_names.end(), name) -
            acm.entry_names.begin());
    };

    for (const std::string& touchable : touchables)
    {
        ensure_entry(touchable);
    }
    // The exempted things also have to be allowed to touch EACH OTHER, not just the hand.
    // Once the object is attached it stops being a world object and starts moving with the
    // arm, and lifting it out of a surface drags it through the octomap voxels of that
    // surface -- which is a collision between two things the hand is already allowed to
    // touch, and would otherwise fail the lift with the grasp already made.
    for (std::size_t i = 0; i < touchables.size(); ++i)
    {
        for (std::size_t j = i + 1; j < touchables.size(); ++j)
        {
            const std::size_t a            = index_of(touchables[i]);
            const std::size_t b            = index_of(touchables[j]);
            acm.entry_values[a].enabled[b] = allowed;
            acm.entry_values[b].enabled[a] = allowed;
        }
    }
    for (const std::string& touchable : touchables)
    {
        const std::size_t other = index_of(touchable);
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

    auto apply           = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply->scene.is_diff = true;
    apply->scene.allowed_collision_matrix = acm;
    auto applied                          = apply_scene_->async_send_request(apply);
    if (applied.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "/apply_planning_scene did not answer");
        return false;
    }
    RCLCPP_INFO(
        get_logger(),
        "%s contact between the %s hand and %zu object(s)",
        allowed ? "allowing" : "restoring",
        side.c_str(),
        touchables.size());
    return applied.get()->success;
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
    if (grasp_rpy_.size() != 3)
    {
        throw std::runtime_error("grasp_rpy needs exactly 3 entries");
    }

    for (const std::string& name : { "left_arm", "right_arm", "left_hand", "right_hand" })
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
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Pick::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<Pick>>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<Pick>> handle) {
            std::thread{ [this, handle] { executePick(handle); } }.detach();
        });

    place_server_ = rclcpp_action::create_server<Place>(
        this,
        "~/place",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Place::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<Place>>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<Place>> handle) {
            std::thread{ [this, handle] { executePlace(handle); } }.detach();
        });

    posture_server_ = rclcpp_action::create_server<SetArmPosture>(
        this,
        "~/set_arm_posture",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const SetArmPosture::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle<SetArmPosture>>) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandle<SetArmPosture>> handle) {
            std::thread{ [this, handle] { executeSetArmPosture(handle); } }.detach();
        });

    RCLCPP_INFO(
        get_logger(),
        "Manipulation skills up, planning in '%s'. Executing needs the arm and hands already "
        "acquired; this node never takes control authority itself.",
        planning_frame_.c_str());
}

void G1ManipulationServer::onObjects(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_ = *msg;
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
    const geometry_msgs::msg::Pose& object_pose, const ArmContext& arm) const
{
    // Position passes straight through. The grasp frame is defined as the point the hand
    // closes on, so putting that frame at the object IS the grasp -- there is no offset to
    // apply, and the arithmetic that used to live here was the source of two separate bugs.
    geometry_msgs::msg::Pose goal;
    goal.position = object_pose.position;

    // Only the orientation is a choice, and it mirrors: the two hands close in opposite
    // directions, so the roll that points the closing axis at the floor flips sign.
    tf2::Quaternion rotation;
    rotation.setRPY((arm.is_left ? -1.0 : 1.0) * grasp_rpy_[0], grasp_rpy_[1], grasp_rpy_[2]);
    goal.orientation = tf2::toMsg(rotation);
    return goal;
}

bool G1ManipulationServer::moveTo(
    MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
    const std::string& what)
{
    group.setStartStateToCurrentState();
    // Named explicitly rather than relying on the group's default tip: the goal is for the
    // grasp frame, which hangs off the palm and is not what the group ends at.
    group.setPoseTarget(pose, link);

    MoveGroup::Plan plan;
    const auto      planned = group.plan(plan);
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

bool G1ManipulationServer::moveToNamed(MoveGroup& group, const std::string& named_target)
{
    group.setStartStateToCurrentState();
    if (!group.setNamedTarget(named_target))
    {
        RCLCPP_ERROR(
            get_logger(),
            "'%s' is not a named pose of group '%s'",
            named_target.c_str(),
            group.getName().c_str());
        return false;
    }
    return group.move() == moveit::core::MoveItErrorCode::SUCCESS;
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
        moveToNamed(*hand_group, "open");
        arm_group->detachObject(goal->object_id);
        allowHandContact(arm, { "<octomap>", goal->object_id }, false);
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
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
        detection->header.frame_id.empty() ? objects_.header.frame_id : detection->header.frame_id;
    const auto object_pose = toPlanningFrame(detection->results.front().pose.pose, object_frame);
    if (!object_pose)
    {
        fail(Pick::Feedback::PHASE_LOCATING, "could not transform the object pose");
        return;
    }
    const moveit_msgs::msg::CollisionObject object =
        publishCollisionObject(*detection, *object_pose);

    const geometry_msgs::msg::Pose grasp_goal    = graspFrameGoal(*object_pose, arm);
    geometry_msgs::msg::Pose       pregrasp_goal = grasp_goal;
    pregrasp_goal.position.z += approach_height_m_;

    feedback->phase = Pick::Feedback::PHASE_PREGRASP;
    goal_handle->publish_feedback(feedback);
    // Opened before the approach, not after: closing on the way in would knock the object off
    // the table before the hand is around it.
    //
    // Reported separately from the arm move rather than sharing one message. They fail for
    // completely different reasons -- a hand that will not open is usually an unacquired or
    // unpowered Dex3, an arm that will not reach is geometry -- and collapsing both into
    // "could not reach the pregrasp pose" sends anyone debugging it to the wrong place.
    if (!moveToNamed(*hand_group, "open"))
    {
        fail(Pick::Feedback::PHASE_PREGRASP, "the hand would not open");
        return;
    }
    if (!moveTo(*arm_group, pregrasp_goal, arm.grasp_frame, "pregrasp"))
    {
        fail(Pick::Feedback::PHASE_PREGRASP, "could not reach the pregrasp pose");
        return;
    }

    feedback->phase = Pick::Feedback::PHASE_APPROACH;
    goal_handle->publish_feedback(feedback);

    // Only NOW is contact allowed, and only for the last few centimetres.
    //
    // Turning it on earlier is what let a plan route the arm straight through the table on the
    // way to the pregrasp pose: with the hand exempt from the octomap for the whole skill, a
    // path through the surface costs the planner nothing. The transit above therefore runs
    // fully collision-checked, and the exemption covers only the short descent that has to end
    // in contact.
    //
    // The object's own collision geometry is REMOVED rather than exempted. It was added so the
    // pregrasp plan would route around it; from here it is in the way, and MoveIt's documented
    // pick sequence is remove, close, attach. Attaching re-adds it as an attached body, which
    // is also what gets PointCloudOctomapUpdater's ShapeMask to stop feeding it back into the
    // octomap -- world collision objects get no such filtering, attached bodies do.
    allowHandContact(arm, { "<octomap>" }, true);
    planning_scene_.removeCollisionObjects({ goal->object_id });

    if (!moveTo(*arm_group, grasp_goal, arm.grasp_frame, "approach"))
    {
        fail(Pick::Feedback::PHASE_APPROACH, "could not reach the grasp pose");
        return;
    }

    feedback->phase = Pick::Feedback::PHASE_GRASP;
    goal_handle->publish_feedback(feedback);
    if (!moveToNamed(*hand_group, "closed"))
    {
        fail(Pick::Feedback::PHASE_GRASP, "the hand did not close");
        return;
    }
    // Attached AFTER the hand closes, so the planner starts treating the object as part of the
    // arm at the same moment the hand does.
    //
    // Built explicitly rather than via attachObject(id, link), which promotes an object that is
    // still in the world -- and this one was removed before the approach, so there is nothing
    // left to promote. An attached body carries its own geometry, so the shape and pose come
    // from what publishCollisionObject built.
    //
    // touch_links is what tells the planner the fingers are SUPPOSED to be in contact with it.
    // Without them the attach itself reads as a collision the moment it takes effect.
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name        = arm.palm_link;
    attached.object           = object;
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached.touch_links =
        hand_group->getRobotModel()->getJointModelGroup(arm.hand_group)->getLinkModelNames();
    attached.touch_links.push_back(arm.palm_link);
    planning_scene_.applyAttachedCollisionObject(attached);

    // Extended to the object only now that it is attached and about to be lifted OUT of the
    // surface it was resting on. Its own voxels, and the table's underneath it, are still in
    // the octomap from before the grasp -- the ShapeMask stops new clouds re-adding it, but it
    // does not erase what is already there. Without this the lift fails before it starts, with
    // "start state appears to be in collision", which is the attached cube against the table
    // it is still sitting on.
    allowHandContact(arm, { "<octomap>", goal->object_id }, true);

    feedback->phase = Pick::Feedback::PHASE_LIFT;
    goal_handle->publish_feedback(feedback);
    geometry_msgs::msg::Pose lifted = grasp_goal;
    lifted.position.z += lift_height_m_;
    if (!moveTo(*arm_group, lifted, arm.grasp_frame, "lift"))
    {
        fail(Pick::Feedback::PHASE_LIFT, "could not lift clear of the surface");
        return;
    }

    allowHandContact(arm, { "<octomap>", goal->object_id }, false);
    result->success = true;
    result->message = "picked " + goal->object_id + " with the " + goal->arm + " hand";
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

    const auto fail = [&](const std::string& phase, const std::string& why) {
        allowHandContact(arm, { "<octomap>" }, false);
        result->success = false;
        result->message = phase + ": " + why;
        goal_handle->abort(result);
    };

    // Lowering onto a surface enters occupied space just as reaching into one does. Only the
    // octomap here: what is held is already attached, so MoveIt is treating it as part of the
    // arm rather than as an obstacle.
    allowHandContact(arm, { "<octomap>" }, true);

    // An empty frame means the goal is already in the planning frame. Anything else is
    // transformed rather than assumed: a target in odom treated as pelvis lands metres away,
    // and by the time that is visible the arm is already moving.
    const auto target = toPlanningFrame(goal->pose.pose, goal->pose.header.frame_id);
    if (!target)
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not transform the target pose");
        return;
    }

    const geometry_msgs::msg::Pose place_goal = graspFrameGoal(*target, arm);
    geometry_msgs::msg::Pose       preplace   = place_goal;
    preplace.position.z += approach_height_m_;

    feedback->phase = Place::Feedback::PHASE_PREPLACE;
    goal_handle->publish_feedback(feedback);
    if (!moveTo(*arm_group, preplace, arm.grasp_frame, "preplace"))
    {
        fail(Place::Feedback::PHASE_PREPLACE, "could not reach the pose above the target");
        return;
    }

    feedback->phase = Place::Feedback::PHASE_LOWER;
    goal_handle->publish_feedback(feedback);
    if (!moveTo(*arm_group, place_goal, arm.grasp_frame, "lower"))
    {
        fail(Place::Feedback::PHASE_LOWER, "could not lower onto the target");
        return;
    }

    feedback->phase = Place::Feedback::PHASE_RELEASE;
    goal_handle->publish_feedback(feedback);
    if (!moveToNamed(*hand_group, "open"))
    {
        fail(Place::Feedback::PHASE_RELEASE, "the hand did not open");
        return;
    }
    // Detached only after the hand is open. Detaching first would let the planner route the
    // arm through a volume the object is still occupying.
    //
    // What is held is read from the scene rather than remembered from the pick, so a place
    // still works against a server that did not do the picking.
    const moveit::core::RobotStatePtr              state = arm_group->getCurrentState();
    std::vector<const moveit::core::AttachedBody*> attached;
    state->getAttachedBodies(attached, state->getJointModelGroup(arm.arm_group));
    for (const moveit::core::AttachedBody* body : attached)
    {
        arm_group->detachObject(body->getName());
    }

    feedback->phase = Place::Feedback::PHASE_RETREAT;
    goal_handle->publish_feedback(feedback);
    if (!moveTo(*arm_group, preplace, arm.grasp_frame, "retreat"))
    {
        fail(Place::Feedback::PHASE_RETREAT, "could not retreat clear of the object");
        return;
    }

    allowHandContact(arm, { "<octomap>" }, false);
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
