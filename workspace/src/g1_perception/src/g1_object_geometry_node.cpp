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

/// Best-effort in, matching the relay's sensor QoS: a reliable subscriber never matches it.
///
/// Deeper than the usual one: a mask names the frame it was cut from, and a depth frame dropped
/// on arrival cannot be recovered later. At 2.9 MB a frame, two arriving together lose one.
rclcpp::QoS sensorQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(8)).best_effort().durability_volatile();
}

/// Reliable out and in for masks: 13 kB at under a hertz, and a dropped frame is seconds blind.
rclcpp::QoS maskQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
}

/// Reliable, matching what g1_object_pose_source and the skills above it expect.
rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

double dot(const Point3& a, const Point3& b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

geometry_msgs::msg::Pose poseFrom(const OrientedBox& box)
{
    const Point3 up{ (box.axis_x.y * box.axis_y.z) - (box.axis_x.z * box.axis_y.y),
                     (box.axis_x.z * box.axis_y.x) - (box.axis_x.x * box.axis_y.z),
                     (box.axis_x.x * box.axis_y.y) - (box.axis_x.y * box.axis_y.x) };
    const tf2::Matrix3x3 rotation(box.axis_x.x, box.axis_y.x, up.x, box.axis_x.y, box.axis_y.y,
                                  up.y, box.axis_x.z, box.axis_y.z, up.z);
    tf2::Quaternion orientation;
    rotation.getRotation(orientation);

    geometry_msgs::msg::Pose pose;
    pose.position.x = box.centre.x;
    pose.position.y = box.centre.y;
    pose.position.z = box.centre.z;
    pose.orientation = tf2::toMsg(orientation.normalized());
    return pose;
}

}  // namespace

G1ObjectGeometry::G1ObjectGeometry(const rclcpp::NodeOptions& options)
    : rclcpp::Node("g1_object_geometry", options),
      depth_history_(declare_parameter<double>("depth_history_s", 3.0),
                     declare_parameter<double>("stamp_tolerance_ms", 5.0) / 1000.0),
      tracker_(declare_parameter<double>("track_match_radius_m", 0.08),
               declare_parameter<double>("track_timeout_s", 2.0))
{
    up_frame_ = declare_parameter<std::string>("up_frame", "odom");
    mask_erosion_px_ = static_cast<int>(declare_parameter<int>("mask_erosion_px", 2));
    support_ring_px_ = static_cast<int>(declare_parameter<int>("support_ring_px", 6));
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.15);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 2.5);
    depth_gate_m_ = declare_parameter<double>("depth_gate_m", 0.05);
    min_points_ = static_cast<int>(declare_parameter<int>("min_points", 150));
    min_support_points_ = static_cast<int>(declare_parameter<int>("min_support_points", 50));
    support_band_m_ = declare_parameter<double>("support_band_m", 0.06);
    min_extent_m_ = declare_parameter<double>("min_extent_m", 0.01);
    max_extent_m_ = declare_parameter<double>("max_extent_m", 0.40);
    min_score_ = declare_parameter<double>("min_score", 0.30);
    transform_timeout_s_ = declare_parameter<double>("transform_timeout_s", 0.2);
    publish_bare_phrase_alias_ = declare_parameter<bool>("publish_bare_phrase_alias", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("~/object_poses",
                                                                       objectsQos());
    tracked_pub_ = create_publisher<g1_msgs::msg::InstanceMaskArray>("~/tracked_masks", maskQos());

    masks_sub_ = create_subscription<g1_msgs::msg::InstanceMaskArray>(
        "~/instance_masks", maskQos(),
        [this](const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks) { onMasks(masks); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image_raw", sensorQos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr depth) { onDepth(std::move(depth)); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", sensorQos(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { onCameraInfo(std::move(info)); });
}

void G1ObjectGeometry::onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth)
{
    if (depth->encoding != "32FC1")
    {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                              "depth is %s; this node reads 32FC1 metres",
                              depth->encoding.c_str());
        return;
    }
    depth_history_.push(std::move(depth));
}

void G1ObjectGeometry::onCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    camera_info_ = std::move(info);
}

std::optional<OrientedBox> G1ObjectGeometry::measure(
    const g1_msgs::msg::InstanceMask& instance,
    const sensor_msgs::msg::Image&    depth,
    const Intrinsics&                 intrinsics,
    const Point3&                     up) const
{
    const DepthView view{ depth.data, depth.width, depth.height, depth.step };
    const MaskView  mask{ instance.data, instance.roi.x_offset, instance.roi.y_offset,
                          instance.roi.width, instance.roi.height };
    if (mask.data.size() != static_cast<std::size_t>(mask.width) * mask.height)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "instance '%s' carries %zu bytes for a %ux%u region",
                             instance.label.c_str(), mask.data.size(), mask.width, mask.height);
        return std::nullopt;
    }

    const std::vector<std::uint8_t> eroded = erodeMask(mask, mask_erosion_px_);
    std::vector<Point3>             points =
        deproject(view, mask, eroded, intrinsics, min_depth_m_, max_depth_m_);
    if (static_cast<int>(points.size()) < min_points_)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "instance '%s' left %zu points with usable depth, under the %d needed",
                             instance.label.c_str(), points.size(), min_points_);
        return std::nullopt;
    }
    gateByMedianDepth(points, depth_gate_m_);

    double lowest = std::numeric_limits<double>::max();
    for (const Point3& point : points)
    {
        lowest = std::min(lowest, dot(point, up));
    }

    std::optional<double>     support;
    const std::vector<Point3> ring =
        supportRing(view, mask, intrinsics, support_ring_px_, min_depth_m_, max_depth_m_);
    std::vector<double> heights;
    heights.reserve(ring.size());
    for (const Point3& point : ring)
    {
        // Only what could be the surface this object stands on. Without the band the median is
        // dragged by the floor beyond the table on one side and by a taller neighbour on the
        // other, and either one puts the object's base somewhere it is not.
        const double height = dot(point, up);
        if (std::abs(height - lowest) <= support_band_m_)
        {
            heights.push_back(height);
        }
    }
    if (static_cast<int>(heights.size()) >= min_support_points_)
    {
        const std::size_t middle = heights.size() / 2;
        std::nth_element(heights.begin(), heights.begin() + static_cast<std::ptrdiff_t>(middle),
                         heights.end());
        support = heights[middle];
    }

    return fitOrientedBox(points, up, support, min_extent_m_, max_extent_m_);
}

void G1ObjectGeometry::onMasks(const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks)
{
    const double stamp_s = DepthHistory::stampSeconds(masks->header);
    std::vector<Measured> measured;

    // An empty answer is still an answer: it is how the stream says the objects are gone, and a
    // consumer that kept the last non-empty one would act on an object nobody can see.
    if (!masks->instances.empty())
    {
        const sensor_msgs::msg::Image::ConstSharedPtr depth = depth_history_.at(stamp_s);
        if (depth == nullptr)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "no depth frame within tolerance of the mask stamp %.3f", stamp_s);
            return;
        }
        if (camera_info_ == nullptr)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no camera_info yet");
            return;
        }
        if (camera_info_->width != masks->image_width || camera_info_->height != masks->image_height)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                  "masks are indexed against %ux%u but camera_info says %ux%u",
                                  masks->image_width, masks->image_height, camera_info_->width,
                                  camera_info_->height);
            return;
        }

        geometry_msgs::msg::TransformStamped to_up_frame;
        try
        {
            to_up_frame = tf_buffer_->lookupTransform(
                up_frame_, masks->header.frame_id, masks->header.stamp,
                rclcpp::Duration::from_seconds(transform_timeout_s_));
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
            return;
        }

        // Up in the camera's own frame, so the fit knows which way the surface holds the object.
        tf2::Quaternion rotation;
        tf2::fromMsg(to_up_frame.transform.rotation, rotation);
        const tf2::Vector3 up_in_camera = tf2::quatRotate(rotation.inverse(), { 0.0, 0.0, 1.0 });
        const Point3       up{ up_in_camera.x(), up_in_camera.y(), up_in_camera.z() };

        const Intrinsics intrinsics{ camera_info_->k[0], camera_info_->k[4], camera_info_->k[2],
                                     camera_info_->k[5] };

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
            geometry_msgs::msg::Pose in_camera = poseFrom(*box);
            geometry_msgs::msg::Pose in_up_frame;
            tf2::doTransform(in_camera, in_up_frame, to_up_frame);
            measured.push_back({ instance.label, instance.score, *box,
                                 { in_up_frame.position.x, in_up_frame.position.y,
                                   in_up_frame.position.z },
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
    const g1_msgs::msg::InstanceMaskArray& masks,
    const std::vector<Measured>&           measured,
    const std::vector<std::string>&        ids)
{
    vision_msgs::msg::Detection3DArray objects;
    // The image's stamp, not now(): a consumer judging staleness has to see the age of the
    // measurement, and the detector spends over a second of it.
    objects.header = masks.header;

    g1_msgs::msg::InstanceMaskArray tracked = masks;

    for (std::size_t i = 0; i < measured.size(); ++i)
    {
        const Measured& object = measured[i];
        vision_msgs::msg::Detection3D detection;
        detection.header = masks.header;
        detection.id = ids[i];
        detection.bbox.center = poseFrom(object.box);
        detection.bbox.size.x = object.box.size_x;
        detection.bbox.size.y = object.box.size_y;
        detection.bbox.size.z = object.box.size_z;

        vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis.class_id = ids[i];
        hypothesis.hypothesis.score = object.score;
        hypothesis.pose.pose = detection.bbox.center;
        detection.results.push_back(hypothesis);
        objects.detections.push_back(detection);

        // A second copy under the bare phrase while only one object answers to it. The skills
        // and the trees name objects by phrase, and an index they cannot predict would make
        // every existing goal unreachable.
        if (publish_bare_phrase_alias_ && tracker_.isSoleTrackFor(object.phrase))
        {
            vision_msgs::msg::Detection3D alias = detection;
            alias.id = slugify(object.phrase);
            alias.results.front().hypothesis.class_id = alias.id;
            objects.detections.push_back(alias);
        }

        tracked.instances[object.instance_index].label = ids[i];
    }

    objects_pub_->publish(objects);
    tracked_pub_->publish(tracked);
}

}  // namespace g1_perception
