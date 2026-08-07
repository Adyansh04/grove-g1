#include "g1_manipulation/g1_object_pose_source_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace g1_manipulation
{

namespace
{

// The simulator's frames. `world` is what g1_sensor_relay stamps ground truth with; `odom`
// is what the rest of the stack navigates and plans in.
constexpr const char* kSimSourceFrame = "world";

// Best-effort in, matching g1_sensor_relay's sensor QoS: a reliable subscriber against a
// best-effort publisher simply receives nothing, which is the usual reason a topic looks dead.
rclcpp::QoS sourceQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

// Reliable out, deliberately unlike the input. This is not a sensor stream a consumer samples;
// it is what a skill decides a grasp from at 10 Hz, and a dropped message costs a failed pick.
rclcpp::QoS outputQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

}  // namespace

bool parseObjectSource(const std::string& name, ObjectSource& out)
{
    if (name == "sim_ground_truth")
    {
        out = ObjectSource::SimGroundTruth;
        return true;
    }
    if (name == "hardware")
    {
        out = ObjectSource::Hardware;
        return true;
    }
    return false;
}

G1ObjectPoseSource::G1ObjectPoseSource(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("g1_object_pose_source", options)
{
    declare_parameter<std::string>("object_source", "hardware");
    declare_parameter<std::string>("output_frame_id", "odom");
}

bool G1ObjectPoseSource::readParameters()
{
    const std::string source_name = get_parameter("object_source").as_string();
    if (!parseObjectSource(source_name, source_))
    {
        RCLCPP_ERROR(
            get_logger(),
            "object_source='%s' is not a known source. Use 'sim_ground_truth' or 'hardware'.",
            source_name.c_str());
        return false;
    }

    if (source_ == ObjectSource::Hardware)
    {
        // Long on purpose. Anyone who reaches this is about to go looking for a perception
        // stack that does not exist yet, and the alternative -- publishing nothing, quietly --
        // reads as a broken topic rather than as an unbuilt milestone.
        RCLCPP_ERROR(
            get_logger(),
            "object_source='hardware' is not implemented: there is no object-detection "
            "pipeline on this robot yet. Manipulation-perception (instance segmentation and "
            "6D pose estimation, see docs/ARCHITECTURE.md Layer 3) is its own milestone. "
            "Refusing to configure rather than let a grasp planner run on simulator ground "
            "truth it cannot tell apart from a real measurement.");
        return false;
    }

    output_frame_id_ = get_parameter("output_frame_id").as_string();
    if (output_frame_id_.empty())
    {
        RCLCPP_ERROR(get_logger(), "output_frame_id must be non-empty");
        return false;
    }
    return true;
}

G1ObjectPoseSource::CallbackReturn G1ObjectPoseSource::on_configure(const rclcpp_lifecycle::State&)
{
    // Nothing is created before this returns true, so an unimplemented source leaves no
    // publisher behind for a consumer to wait on forever.
    if (!readParameters())
    {
        return CallbackReturn::FAILURE;
    }

    objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/objects", outputQos());
    source_sub_  = create_subscription<vision_msgs::msg::Detection3DArray>(
        "~/object_poses",
        sourceQos(),
        std::bind(&G1ObjectPoseSource::onGroundTruth, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Configured on simulator ground truth: %s -> %s in frame '%s'. These are exact MuJoCo "
        "body poses, not measurements -- no noise, no occlusion, no misdetection, and every "
        "listed object is always visible.",
        source_sub_->get_topic_name(),
        objects_pub_->get_topic_name(),
        output_frame_id_.c_str());
    return CallbackReturn::SUCCESS;
}

G1ObjectPoseSource::CallbackReturn G1ObjectPoseSource::on_cleanup(const rclcpp_lifecycle::State&)
{
    source_sub_.reset();
    objects_pub_.reset();
    return CallbackReturn::SUCCESS;
}

G1ObjectPoseSource::CallbackReturn
G1ObjectPoseSource::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    return LifecycleNode::on_activate(previous_state);
}

G1ObjectPoseSource::CallbackReturn
G1ObjectPoseSource::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    return LifecycleNode::on_deactivate(previous_state);
}

void G1ObjectPoseSource::onGroundTruth(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
    if (!objects_pub_->is_activated())
    {
        return;
    }
    if (msg->header.frame_id != kSimSourceFrame)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Dropping ground truth stamped '%s'; this source is defined in '%s'.",
            msg->header.frame_id.c_str(),
            kSimSourceFrame);
        return;
    }

    // A relabel, not a transform, and only because the two frames are the same one: the
    // converged track's odom IS the simulator's world origin, since g1_odometry_publisher
    // takes the pelvis position straight from MuJoCo with no offset. A TF lookup is not the
    // alternative -- `world` is deliberately not in the tree, because nothing may localise
    // against it.
    //
    // This is one of the things that stops being true with real perception: a detector
    // reports in a camera frame, and that node transforms for real.
    vision_msgs::msg::Detection3DArray out = *msg;
    out.header.frame_id                    = output_frame_id_;
    for (vision_msgs::msg::Detection3D& detection : out.detections)
    {
        detection.header.frame_id = output_frame_id_;
    }

    // The stamp is carried through rather than refreshed. Restamping here would launder a
    // stale pose as a fresh one, and consumers judge freshness for themselves: only the skill
    // about to grasp knows how old is too old.
    objects_pub_->publish(std::move(out));
}

}  // namespace g1_manipulation
