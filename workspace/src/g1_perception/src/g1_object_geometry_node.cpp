/**
 * @file g1_object_geometry_node.cpp
 * @brief Pairs each mask array with its depth frame, fits boxes, tracks them and publishes poses.
 */

#include "g1_perception/g1_object_geometry_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection3_d.hpp>

namespace g1_perception
{
namespace
{

/// Best effort, as the relay publishes; a reliable reader never matches it. Deep, since a mask
/// needs the exact frame it was cut from.
rclcpp::QoS sensorQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(8)).best_effort().durability_volatile();
}

/// Reliable: masks are small and under 1 Hz, so a dropped one is seconds blind.
rclcpp::QoS maskQos() { return rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile(); }

/// Reliable, matching what g1_object_pose_source and the skills above it expect.
rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

geometry_msgs::msg::Pose poseFrom(const OrientedBox& box)
{
    const Point3         up = cross(box.axis_x, box.axis_y);
    const tf2::Matrix3x3 rotation(
        box.axis_x.x,
        box.axis_y.x,
        up.x,
        box.axis_x.y,
        box.axis_y.y,
        up.y,
        box.axis_x.z,
        box.axis_y.z,
        up.z);
    tf2::Quaternion orientation;
    rotation.getRotation(orientation);

    geometry_msgs::msg::Pose pose;
    pose.position.x  = box.centre.x;
    pose.position.y  = box.centre.y;
    pose.position.z  = box.centre.z;
    pose.orientation = tf2::toMsg(orientation.normalized());
    return pose;
}

}  // namespace

G1ObjectGeometry::G1ObjectGeometry(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_object_geometry", options)
  , depth_history_(
        declare_parameter<double>("depth_history_s", 5.0),
        declare_parameter<double>("stamp_tolerance_ms", 130.0) / 1000.0,
        static_cast<std::size_t>(declare_parameter<int>("depth_history_max_frames", 150)))
  , tracker_(
        declare_parameter<double>("track_match_radius_m", 0.08),
        declare_parameter<double>("track_timeout_s", 6.0))
{
    up_frame_                  = declare_parameter<std::string>("up_frame", "odom");
    mask_erosion_px_           = static_cast<int>(declare_parameter<int>("mask_erosion_px", 2));
    support_ring_px_           = static_cast<int>(declare_parameter<int>("support_ring_px", 6));
    min_depth_m_               = declare_parameter<double>("min_depth_m", 0.15);
    max_depth_m_               = declare_parameter<double>("max_depth_m", 2.5);
    depth_gate_m_              = declare_parameter<double>("depth_gate_m", 0.15);
    min_points_                = static_cast<int>(declare_parameter<int>("min_points", 150));
    min_support_points_        = static_cast<int>(declare_parameter<int>("min_support_points", 50));
    support_band_m_            = declare_parameter<double>("support_band_m", 0.06);
    min_extent_m_              = declare_parameter<double>("min_extent_m", 0.01);
    max_extent_m_              = declare_parameter<double>("max_extent_m", 0.40);
    min_score_                 = declare_parameter<double>("min_score", 0.30);
    transform_timeout_s_       = declare_parameter<double>("transform_timeout_s", 0.2);
    publish_bare_phrase_alias_ = declare_parameter<bool>("publish_bare_phrase_alias", true);

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    objects_pub_ =
        create_publisher<vision_msgs::msg::Detection3DArray>("~/object_poses", objectsQos());
    tracked_pub_ = create_publisher<g1_msgs::msg::InstanceMaskArray>("~/tracked_masks", maskQos());

    masks_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mask_options;
    mask_options.callback_group = masks_group_;
    masks_sub_                  = create_subscription<g1_msgs::msg::InstanceMaskArray>(
        "~/instance_masks",
        maskQos(),
        [this](const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks) { onMasks(masks); },
        mask_options);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image_raw",
        sensorQos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr depth) { onDepth(std::move(depth)); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info",
        sensorQos(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
            onCameraInfo(std::move(info));
        });
}

void G1ObjectGeometry::onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth)
{
    if (depth->encoding != "32FC1")
    {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "depth is %s; this node reads 32FC1 metres",
            depth->encoding.c_str());
        return;
    }
    const std::lock_guard<std::mutex> lock(frames_mutex_);
    depth_history_.push(std::move(depth));
}

void G1ObjectGeometry::onCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    const std::lock_guard<std::mutex> lock(frames_mutex_);
    camera_info_ = std::move(info);
}

std::optional<OrientedBox> G1ObjectGeometry::measure(
    const g1_msgs::msg::InstanceMask& instance, const sensor_msgs::msg::Image& depth,
    const Intrinsics& intrinsics, const Point3& up) const
{
    const DepthView view{ depth.data, depth.width, depth.height, depth.step };
    const MaskView  mask{ instance.data,
                         instance.roi.x_offset,
                         instance.roi.y_offset,
                         instance.roi.width,
                         instance.roi.height };
    if (mask.data.size() != static_cast<std::size_t>(mask.width) * mask.height)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "instance '%s' carries %zu bytes for a %ux%u region",
            instance.label.c_str(),
            mask.data.size(),
            mask.width,
            mask.height);
        return std::nullopt;
    }

    const std::vector<std::uint8_t> eroded = erodeMask(mask, mask_erosion_px_);
    std::vector<Point3>             points =
        deproject(view, mask, eroded, intrinsics, min_depth_m_, max_depth_m_);
    // Counted after the gate, so a mask half on the background cannot pass on points it drops.
    gateByMedianDepth(points, depth_gate_m_);
    if (static_cast<int>(points.size()) < min_points_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "instance '%s' left %zu points at one depth, under the %d needed",
            instance.label.c_str(),
            points.size(),
            min_points_);
        return std::nullopt;
    }

    const std::vector<Point3> ring =
        supportRing(view, mask, intrinsics, support_ring_px_, min_depth_m_, max_depth_m_);
    const std::optional<double> support =
        supportHeight(ring, points, up, support_band_m_, min_support_points_);

    return fitOrientedBox(points, up, support, min_extent_m_, max_extent_m_);
}

void G1ObjectGeometry::onMasks(const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks)
{
    const double          stamp_s = DepthHistory::stampSeconds(masks->header);
    std::vector<Measured> measured;

    // An empty array still reaches the tracker: it is how objects disappear.
    if (!masks->instances.empty())
    {
        // Snapshot, then let the lock go: the TF wait below must not hold up depth ingest.
        sensor_msgs::msg::Image::ConstSharedPtr      depth;
        sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
        {
            const std::lock_guard<std::mutex> lock(frames_mutex_);
            depth       = depth_history_.at(stamp_s);
            camera_info = camera_info_;
        }

        if (depth == nullptr)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "no depth frame within tolerance of the mask stamp %.3f",
                stamp_s);
            return;
        }
        if (camera_info == nullptr)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no camera_info yet");
            return;
        }
        if (camera_info->width != masks->image_width || camera_info->height != masks->image_height)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "masks are indexed against %ux%u but camera_info says %ux%u",
                masks->image_width,
                masks->image_height,
                camera_info->width,
                camera_info->height);
            return;
        }

        geometry_msgs::msg::TransformStamped to_up_frame;
        try
        {
            to_up_frame = tf_buffer_->lookupTransform(
                up_frame_,
                masks->header.frame_id,
                masks->header.stamp,
                rclcpp::Duration::from_seconds(transform_timeout_s_));
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
            return;
        }

        // Up in the camera's own frame, which is what the fit needs.
        tf2::Quaternion rotation;
        tf2::fromMsg(to_up_frame.transform.rotation, rotation);
        const tf2::Vector3 up_in_camera = tf2::quatRotate(rotation.inverse(), { 0.0, 0.0, 1.0 });
        const Point3       up{ up_in_camera.x(), up_in_camera.y(), up_in_camera.z() };

        const Intrinsics intrinsics{ camera_info->k[0],
                                     camera_info->k[4],
                                     camera_info->k[2],
                                     camera_info->k[5] };

        for (std::size_t index = 0; index < masks->instances.size(); ++index)
        {
            const g1_msgs::msg::InstanceMask& instance = masks->instances[index];
            if (instance.score < min_score_)
            {
                continue;
            }
            const std::optional<OrientedBox> box = measure(instance, *depth, intrinsics, up);
            if (!box.has_value())
            {
                continue;
            }
            const geometry_msgs::msg::Pose in_camera = poseFrom(*box);
            geometry_msgs::msg::Pose       in_up_frame;
            tf2::doTransform(in_camera, in_up_frame, to_up_frame);
            measured.push_back(
                { instance.label,
                  instance.score,
                  *box,
                  in_camera,
                  { in_up_frame.position.x, in_up_frame.position.y, in_up_frame.position.z },
                  index });
        }
    }

    std::vector<Observation> observations;
    observations.reserve(measured.size());
    for (const Measured& object : measured)
    {
        observations.push_back({ object.phrase, object.position_in_up_frame });
    }
    const std::vector<std::string> ids = tracker_.update(observations, now().seconds());
    publish(*masks, measured, ids);
}

void G1ObjectGeometry::publish(
    const g1_msgs::msg::InstanceMaskArray& masks, const std::vector<Measured>& measured,
    const std::vector<std::string>& ids)
{
    vision_msgs::msg::Detection3DArray objects;
    // The image's stamp, not now(), so consumers see the detector's latency.
    objects.header = masks.header;

    g1_msgs::msg::InstanceMaskArray tracked = masks;

    for (std::size_t i = 0; i < measured.size(); ++i)
    {
        const Measured&               object = measured[i];
        vision_msgs::msg::Detection3D detection;
        detection.header      = masks.header;
        detection.id          = ids[i];
        detection.bbox.center = object.pose_in_camera;
        detection.bbox.size.x = object.box.size_x;
        detection.bbox.size.y = object.box.size_y;
        detection.bbox.size.z = object.box.size_z;

        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = ids[i];
        hypothesis.hypothesis.score    = object.score;
        hypothesis.pose.pose           = detection.bbox.center;
        detection.results.push_back(hypothesis);
        objects.detections.push_back(detection);

        // Also under the bare phrase while it names this object: skills address objects by phrase.
        if (publish_bare_phrase_alias_ && tracker_.aliasFor(object.phrase) == ids[i])
        {
            vision_msgs::msg::Detection3D alias       = detection;
            alias.id                                  = slugify(object.phrase);
            alias.results.front().hypothesis.class_id = alias.id;
            objects.detections.push_back(alias);
        }

        tracked.instances[object.instance_index].label = ids[i];
    }

    objects_pub_->publish(objects);
    tracked_pub_->publish(tracked);
}

}  // namespace g1_perception
