#include "g1_manipulation/g1_object_pose_source_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace g1_manipulation
{

namespace
{

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
    declare_parameter<std::string>("source_frame_id", "odom");
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
            "6D pose estimation, see the architecture notes Layer 3) is its own milestone. "
            "Refusing to configure rather than let a grasp planner run on simulator ground "
            "truth it cannot tell apart from a real measurement.");
        return false;
    }

    source_frame_id_ = get_parameter("source_frame_id").as_string();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    if (source_frame_id_.empty() || output_frame_id_.empty())
    {
        RCLCPP_ERROR(get_logger(), "source_frame_id and output_frame_id must be non-empty");
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

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/objects", outputQos());
    source_sub_  = create_subscription<vision_msgs::msg::Detection3DArray>(
        "~/object_poses",
        sourceQos(),
        std::bind(&G1ObjectPoseSource::onGroundTruth, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Configured on simulator ground truth: %s in '%s' -> %s in '%s'. These are exact "
        "MuJoCo body poses, not measurements -- no noise, no occlusion, no misdetection, and "
        "every listed object is always visible.",
        source_sub_->get_topic_name(),
        source_frame_id_.c_str(),
        objects_pub_->get_topic_name(),
        output_frame_id_.c_str());
    return CallbackReturn::SUCCESS;
}

G1ObjectPoseSource::CallbackReturn G1ObjectPoseSource::on_cleanup(const rclcpp_lifecycle::State&)
{
    source_sub_.reset();
    objects_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

void G1ObjectPoseSource::onGroundTruth(const vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
    if (!objects_pub_->is_activated())
    {
        return;
    }
    // The detector reports from the frame it measured in, which rides on the robot. Rewriting
    // that label to a fixed frame is only correct while the two coincide, and they stop
    // coinciding the moment odom is an estimate rather than ground truth -- an earlier version
    // relabelled, and the base approach then chased a point 2 m from where the object was.
    if (msg->header.frame_id != source_frame_id_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Dropping object poses stamped '%s'; this source is configured for '%s'.",
            msg->header.frame_id.c_str(),
            source_frame_id_.c_str());
        return;
    }

    geometry_msgs::msg::TransformStamped source_to_output;
    try
    {
        // At the message's own stamp, not the latest: the pose was measured when the camera
        // was somewhere specific, and composing it with a newer transform moves the object by
        // however far the robot walked in between.
        source_to_output = tf_buffer_->lookupTransform(
            output_frame_id_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.2));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Cannot place objects in '%s': %s",
            output_frame_id_.c_str(),
            ex.what());
        return;
    }

    vision_msgs::msg::Detection3DArray out = *msg;
    out.header.frame_id                    = output_frame_id_;
    for (vision_msgs::msg::Detection3D& detection : out.detections)
    {
        detection.header.frame_id = output_frame_id_;
        tf2::doTransform(detection.bbox.center, detection.bbox.center, source_to_output);
        for (vision_msgs::msg::ObjectHypothesisWithPose& hypothesis : detection.results)
        {
            tf2::doTransform(hypothesis.pose.pose, hypothesis.pose.pose, source_to_output);
        }
    }

    // The stamp is carried through rather than refreshed. Restamping here would launder a
    // stale pose as a fresh one, and consumers judge freshness for themselves: only the skill
    // about to grasp knows how old is too old.
    objects_pub_->publish(std::move(out));
}

}  // namespace g1_manipulation
