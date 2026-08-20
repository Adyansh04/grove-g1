/**
 * @file g1_object_pose_source_node.cpp
 * @brief Publishes /objects from the configured source, with no fallback between sources.
 */

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
        out = ObjectSource::kSimGroundTruth;
        return true;
    }
    if (name == "hardware")
    {
        out = ObjectSource::kHardware;
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
    declare_parameter<bool>("publish_markers", true);
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

    if (source_ == ObjectSource::kHardware)
    {
        // Long on purpose. Anyone who reaches this is about to go looking for a perception
        // stack that does not exist yet, and the alternative of publishing nothing quietly
        // reads as a broken topic rather than as a subsystem that does not exist yet.
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
    publish_markers_ = get_parameter("publish_markers").as_bool();
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
    if (publish_markers_)
    {
        // Transient local so rviz shows the markers when it connects late, which is the usual
        // way of looking at them.
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/object_markers",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    }
    source_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "~/object_poses",
        sourceQos(),
        [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) {
            onGroundTruth(std::move(msg));
        });

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
    markers_pub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

// A box at each object's pose and a label above it, drawn from the SAME message /objects
// carries, so what rviz shows is what a skill acts on rather than a second computation of it.
void G1ObjectPoseSource::publishMarkers(const vision_msgs::msg::Detection3DArray& objects)
{
    auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
    // Rebuilt every message, so a vanished object must not leave its marker on screen.
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers->markers.reserve(1 + 2 * objects.detections.size());
    markers->markers.push_back(clear);

    int id = 0;
    for (const vision_msgs::msg::Detection3D& detection : objects.detections)
    {
        visualization_msgs::msg::Marker box;
        box.header  = objects.header;
        box.ns      = "objects";
        box.id      = id++;
        box.type    = visualization_msgs::msg::Marker::CUBE;
        box.action  = visualization_msgs::msg::Marker::ADD;
        box.pose    = detection.bbox.center;
        box.scale   = detection.bbox.size;
        box.color.r = 0.1F;
        box.color.g = 0.8F;
        box.color.b = 1.0F;
        box.color.a = 0.6F;
        markers->markers.push_back(box);

        visualization_msgs::msg::Marker label = box;
        label.id                              = id++;
        label.ns                              = "object_labels";
        label.type                            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.text                            = detection.id.empty() ? "object" : detection.id;
        label.scale                           = geometry_msgs::msg::Vector3();
        label.scale.z                         = 0.06;
        label.pose.position.z += 0.5 * detection.bbox.size.z + 0.05;
        label.color.a = 1.0F;
        markers->markers.push_back(label);
    }
    markers_pub_->publish(std::move(markers));
}

// By value, not const-ref: this mutates the message in place, and rclcpp has no const-ref
// dispatch for a mutable pointee.
// NOLINTNEXTLINE(performance-unnecessary-value-param)
void G1ObjectPoseSource::onGroundTruth(vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
    if (!objects_pub_->is_activated())
    {
        return;
    }
    // The detector reports from the frame it measured in, which rides on the robot. Rewriting
    // that label to a fixed frame is only correct while the two coincide, and they stop
    // coinciding the moment odom is an estimate rather than ground truth.
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
            output_frame_id_,
            msg->header.frame_id,
            msg->header.stamp,
            tf2::durationFromSec(0.2));
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

    // Mutated in place and moved out rather than copied: the array carries a vector of
    // detections, each with its own vector of hypotheses and strings. Safe because this
    // subscription is inter-process, so the callback owns the only reference: if this node is
    // ever composed with intra-process comms on, the message becomes shared and this must go
    // back to a copy.
    vision_msgs::msg::Detection3DArray& out = *msg;
    out.header.frame_id                     = output_frame_id_;
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
    if (markers_pub_ && markers_pub_->is_activated())
    {
        publishMarkers(out);
    }
    objects_pub_->publish(std::make_unique<vision_msgs::msg::Detection3DArray>(std::move(out)));
}

}  // namespace g1_manipulation
