/**
 * @file mock_detector_node.cpp
 * @brief Cuts instance masks from simulator ground truth against the rendered depth.
 */

#include "g1_perception/mock_detector_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "g1_perception/depth_history.hpp"
#include "g1_perception/object_geometry.hpp"

namespace g1_perception
{
namespace
{
/// Closer than this to the camera plane and the projection is not worth trusting.
constexpr double kMinProjectionDepthM = 0.01;

/// The name half of a "name=phrase" entry, or the whole thing when it carries no phrase.
std::string nameOf(const std::string& entry)
{
    const std::size_t split = entry.find('=');
    return split == std::string::npos ? entry : entry.substr(0, split);
}
}  // namespace

namespace
{

rclcpp::QoS sensorQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}

rclcpp::QoS maskQos() { return rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile(); }

/// Headroom over the configured latency, so timer and camera drift cannot lose the frame.
constexpr double kDepthHistoryHeadroomS = 2.0;

}  // namespace

G1MockDetector::G1MockDetector(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_mock_detector", options)
  // Sized from the latency: a shorter window would silently hand out the oldest frame instead.
  , depth_frames_(
        declare_parameter<double>("mock_latency_s", 0.0) + kDepthHistoryHeadroomS,
        /*tolerance_s=*/0.0)
{
    declare_parameter<std::vector<std::string>>("phrases", std::vector<std::string>{});
    latency_s_           = get_parameter("mock_latency_s").as_double();
    margin_m_            = declare_parameter<double>("mock_margin_m", 0.005);
    min_pixels_          = static_cast<int>(declare_parameter<int>("min_pixels", 50));
    const double rate_hz = declare_parameter<double>("mock_rate_hz", 10.0);

    masks_pub_ = create_publisher<g1_msgs::msg::InstanceMaskArray>("~/instance_masks", maskQos());
    truth_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "object_poses",
        sensorQos(),
        [this](vision_msgs::msg::Detection3DArray::ConstSharedPtr truth) {
            onTruth(std::move(truth));
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image_raw",
        sensorQos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr depth) { onDepth(std::move(depth)); });
    // Colour camera_info, whose frame the real detector stamps masks with; the depth optical
    // frame coincides with it in the URDF.
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
        sensorQos(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
            camera_info_ = std::move(info);
        });

    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate_hz, 0.1)), [this]() {
            publishMasks();
        });
}

void G1MockDetector::onTruth(vision_msgs::msg::Detection3DArray::ConstSharedPtr truth)
{
    truth_ = std::move(truth);
}

void G1MockDetector::onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth)
{
    depth_frames_.push(std::move(depth));
}

std::optional<g1_msgs::msg::InstanceMask> G1MockDetector::maskFor(
    const vision_msgs::msg::Detection3D& detection, const sensor_msgs::msg::Image& depth,
    const std::string& phrase) const
{
    const Intrinsics intrinsics{ camera_info_->k[0],
                                 camera_info_->k[4],
                                 camera_info_->k[2],
                                 camera_info_->k[5] };
    const DepthView  view{ depth.data, depth.width, depth.height, depth.step };

    tf2::Quaternion orientation;
    tf2::fromMsg(detection.bbox.center.orientation, orientation);
    const tf2::Vector3    centre{ detection.bbox.center.position.x,
                               detection.bbox.center.position.y,
                               detection.bbox.center.position.z };
    const tf2::Vector3    half{ (0.5 * detection.bbox.size.x) + margin_m_,
                             (0.5 * detection.bbox.size.y) + margin_m_,
                             (0.5 * detection.bbox.size.z) + margin_m_ };
    const tf2::Quaternion into_object = orientation.inverse();

    // Nothing beyond the object's bounding sphere can belong to it, whatever its depth says.
    const double radius = half.length();
    // An object level with or behind the camera plane has no usable projection.
    if (!(centre.z() > kMinProjectionDepthM) || !std::isfinite(intrinsics.fx) ||
        !std::isfinite(intrinsics.fy))
    {
        return std::nullopt;
    }
    const double spread_x = (radius / centre.z()) * intrinsics.fx;
    const double spread_y = (radius / centre.z()) * intrinsics.fy;
    const double centre_u = ((centre.x() / centre.z()) * intrinsics.fx) + intrinsics.cx;
    const double centre_v = ((centre.y() / centre.z()) * intrinsics.fy) + intrinsics.cy;
    if (!std::isfinite(centre_u) || !std::isfinite(centre_v) || !std::isfinite(spread_x) ||
        !std::isfinite(spread_y))
    {
        return std::nullopt;
    }

    // Clamp as doubles before narrowing: casting an out-of-range double is undefined.
    const auto clamp = [](double value, double lo, double hi) {
        return static_cast<std::int64_t>(std::clamp(value, lo, hi));
    };
    const auto width  = static_cast<double>(depth.width);
    const auto height = static_cast<double>(depth.height);

    const auto x0 = static_cast<std::uint32_t>(clamp(std::floor(centre_u - spread_x), 0.0, width));
    const auto y0 = static_cast<std::uint32_t>(clamp(std::floor(centre_v - spread_y), 0.0, height));
    const auto x1 =
        static_cast<std::uint32_t>(clamp(std::ceil(centre_u + spread_x) + 1.0, 0.0, width));
    const auto y1 =
        static_cast<std::uint32_t>(clamp(std::ceil(centre_v + spread_y) + 1.0, 0.0, height));
    if (x1 <= x0 || y1 <= y0)
    {
        return std::nullopt;
    }

    // Cropped to the hits: a loose region would put the geometry node's support ring on a
    // neighbouring object instead of the table.
    std::vector<std::uint8_t> found(static_cast<std::size_t>(x1 - x0) * (y1 - y0), 0);
    std::uint32_t             hit_x0 = x1;
    std::uint32_t             hit_y0 = y1;
    std::uint32_t             hit_x1 = x0;
    std::uint32_t             hit_y1 = y0;
    int                       filled = 0;
    for (std::uint32_t v = y0; v < y1; ++v)
    {
        for (std::uint32_t u = x0; u < x1; ++u)
        {
            const double z = view.at(u, v);
            if (!std::isfinite(z) || z <= 0.0)
            {
                continue;
            }
            // A pixel belongs to the object only if the surface the sensor saw is inside its box.
            const tf2::Vector3 point{ (u - intrinsics.cx) * z / intrinsics.fx,
                                      (v - intrinsics.cy) * z / intrinsics.fy,
                                      z };
            const tf2::Vector3 local = tf2::quatRotate(into_object, point - centre);
            if (std::abs(local.x()) > half.x() || std::abs(local.y()) > half.y() ||
                std::abs(local.z()) > half.z())
            {
                continue;
            }
            found[(static_cast<std::size_t>(v - y0) * (x1 - x0)) + (u - x0)] = 255;
            hit_x0                                                           = std::min(hit_x0, u);
            hit_y0                                                           = std::min(hit_y0, v);
            hit_x1                                                           = std::max(hit_x1, u);
            hit_y1                                                           = std::max(hit_y1, v);
            ++filled;
        }
    }
    if (filled < min_pixels_)
    {
        return std::nullopt;
    }

    g1_msgs::msg::InstanceMask out;
    out.label        = phrase;
    out.score        = detection.results.empty() ?
                           1.0F :
                           static_cast<float>(detection.results.front().hypothesis.score);
    out.roi.x_offset = hit_x0;
    out.roi.y_offset = hit_y0;
    out.roi.width    = hit_x1 - hit_x0 + 1;
    out.roi.height   = hit_y1 - hit_y0 + 1;
    out.data.assign(static_cast<std::size_t>(out.roi.width) * out.roi.height, 0);
    for (std::uint32_t v = 0; v < out.roi.height; ++v)
    {
        for (std::uint32_t u = 0; u < out.roi.width; ++u)
        {
            out.data[(static_cast<std::size_t>(v) * out.roi.width) + u] =
                found[(static_cast<std::size_t>(v + hit_y0 - y0) * (x1 - x0)) + (u + hit_x0 - x0)];
        }
    }
    return out;
}

void G1MockDetector::publishMasks()
{
    // Re-read every pass, as the real detector does, so a tree's writes to `phrases` apply.
    const std::vector<std::string> phrases = get_parameter("phrases").as_string_array();
    if (truth_ == nullptr || camera_info_ == nullptr || phrases.empty())
    {
        return;
    }

    // The frame a detector with this latency would just have finished: old masks, old depth.
    const sensor_msgs::msg::Image::ConstSharedPtr chosen =
        depth_frames_.atOrBefore(now().seconds() - latency_s_);
    if (chosen == nullptr || chosen->encoding != "32FC1")
    {
        return;
    }

    g1_msgs::msg::InstanceMaskArray masks;
    masks.header.stamp    = chosen->header.stamp;
    masks.header.frame_id = camera_info_->header.frame_id;
    masks.image_width     = chosen->width;
    masks.image_height    = chosen->height;
    masks.model           = "mock";

    for (const vision_msgs::msg::Detection3D& detection : truth_->detections)
    {
        if (detection.results.empty())
        {
            continue;
        }
        const std::string& class_id = detection.results.front().hypothesis.class_id;
        // Takes the real detector's "name=phrase" entries too; only the name matters here.
        const auto match =
            std::find_if(phrases.begin(), phrases.end(), [&class_id](const std::string& entry) {
                return slugify(nameOf(entry)) == class_id;
            });
        if (match == phrases.end())
        {
            continue;
        }
        std::optional<g1_msgs::msg::InstanceMask> instance =
            maskFor(detection, *chosen, nameOf(*match));
        if (instance)
        {
            masks.instances.push_back(std::move(*instance));
        }
    }
    masks_pub_->publish(masks);
}

}  // namespace g1_perception
