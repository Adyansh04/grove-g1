#include "g1_perception/perception_visualizer_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cv_bridge/cv_bridge.hpp>
#include <format>
#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string_view>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "g1_perception/object_geometry.hpp"
#include "g1_perception/object_tracker.hpp"

namespace g1_perception
{
namespace
{

/// @p stamped in @p frame, or nothing when TF cannot say.
template <typename Stamped>
std::optional<Stamped>
transformed(const tf2_ros::Buffer& tf, const Stamped& stamped, const std::string& frame)
{
    try
    {
        return tf.transform(stamped, frame);
    }
    catch (const tf2::TransformException&)
    {
        return std::nullopt;
    }
}

/// Best-effort in, matching the relay: a reliable subscriber never matches it.
rclcpp::QoS sensorQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(8)).best_effort().durability_volatile();
}

/// Matches what the geometry node publishes both topics with.
rclcpp::QoS reliableQos(std::size_t depth)
{
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

constexpr double kMaskAlpha      = 0.45;
constexpr int    kOutlinePx      = 2;
constexpr double kFontScale      = 0.45;
constexpr double kLabelLiftM     = 0.03;
constexpr float  kLabelHeightM   = 0.02F;
constexpr float  kTruthAlpha     = 0.25F;
constexpr int    kLabelPaddingPx = 2;

/// One colour per phrase, the same in every frame and for every instance of it.
cv::Scalar colourFor(std::string_view label)
{
    static constexpr std::array<std::array<double, 3>, 8> kPalette{ { { 230, 25, 75 },
                                                                      { 60, 180, 75 },
                                                                      { 255, 225, 25 },
                                                                      { 0, 130, 200 },
                                                                      { 245, 130, 48 },
                                                                      { 145, 30, 180 },
                                                                      { 70, 240, 240 },
                                                                      { 240, 50, 230 } } };
    const std::string phrase = slugify(ObjectTracker::phraseOf(label));
    const auto&       rgb    = kPalette.at(std::hash<std::string>{}(phrase) % kPalette.size());
    return { rgb[0], rgb[1], rgb[2] };
}

std::optional<cv::Point> project(const tf2::Vector3& point, const Intrinsics& intrinsics)
{
    // Behind or at the camera there is no pixel, and a line to one would cross the whole image.
    if (point.z() <= 1e-3)
    {
        return std::nullopt;
    }
    return cv::Point(
        cvRound((intrinsics.fx * point.x() / point.z()) + intrinsics.cx),
        cvRound((intrinsics.fy * point.y() / point.z()) + intrinsics.cy));
}

void drawBox(
    cv::Mat& image, const vision_msgs::msg::BoundingBox3D& box, const Intrinsics& intrinsics,
    const cv::Scalar& colour)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(box.center.orientation, orientation);
    const tf2::Matrix3x3 rotation(orientation);
    const tf2::Vector3 centre(box.center.position.x, box.center.position.y, box.center.position.z);

    std::array<cv::Point, 8> corners;
    for (std::size_t i = 0; i < corners.size(); ++i)
    {
        // Bit k of the index picks the sign along axis k, so an edge joins two indices one bit apart.
        const tf2::Vector3 local(
            ((i & 1U) != 0 ? 0.5 : -0.5) * box.size.x,
            ((i & 2U) != 0 ? 0.5 : -0.5) * box.size.y,
            ((i & 4U) != 0 ? 0.5 : -0.5) * box.size.z);
        const std::optional<cv::Point> pixel = project(centre + (rotation * local), intrinsics);
        if (!pixel)
        {
            return;
        }
        corners.at(i) = *pixel;
    }
    for (std::size_t from = 0; from < corners.size(); ++from)
    {
        for (std::size_t bit = 1; bit < corners.size(); bit <<= 1U)
        {
            if ((from & bit) == 0)
            {
                cv::line(image, corners.at(from), corners.at(from | bit), colour, 1, cv::LINE_AA);
            }
        }
    }
}

void drawMask(cv::Mat& image, const g1_msgs::msg::InstanceMask& instance, const cv::Scalar& colour)
{
    const cv::Rect roi(
        static_cast<int>(instance.roi.x_offset),
        static_cast<int>(instance.roi.y_offset),
        static_cast<int>(instance.roi.width),
        static_cast<int>(instance.roi.height));
    if ((roi & cv::Rect(0, 0, image.cols, image.rows)) != roi ||
        instance.data.size() != static_cast<std::size_t>(roi.area()))
    {
        return;
    }
    cv::Mat mask(roi.size(), CV_8UC1);
    std::memcpy(mask.data, instance.data.data(), instance.data.size());

    cv::Mat region = image(roi);
    cv::Mat tinted;
    cv::addWeighted(
        region,
        1.0 - kMaskAlpha,
        cv::Mat(region.size(), region.type(), colour),
        kMaskAlpha,
        0.0,
        tinted);
    tinted.copyTo(region, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, roi.tl());
    cv::drawContours(image, contours, -1, colour, kOutlinePx, cv::LINE_AA);
}

struct Label
{
    std::string text;
    cv::Point   anchor;  ///< Where the text's baseline starts.
    cv::Scalar  colour;
};

/// Draws each label on a filled box, moved down past any already drawn that it would cover.
void drawLabels(cv::Mat& image, const std::vector<Label>& labels)
{
    std::vector<cv::Rect> placed;
    placed.reserve(labels.size());
    for (const Label& label : labels)
    {
        int            baseline = 0;
        const cv::Size size =
            cv::getTextSize(label.text, cv::FONT_HERSHEY_SIMPLEX, kFontScale, 1, &baseline);
        // Clamped so an object at the top edge keeps its label inside the frame.
        cv::Rect box(
            label.anchor.x,
            std::max(label.anchor.y - size.height - kLabelPaddingPx, kLabelPaddingPx),
            size.width + (2 * kLabelPaddingPx),
            size.height + kLabelPaddingPx + baseline);
        const auto clash = [&box, &placed] {
            return std::find_if(placed.begin(), placed.end(), [&box](const cv::Rect& other) {
                return (box & other).area() > 0;
            });
        };
        // Each step lands below a label it overlapped, so the box only moves down and this ends.
        for (auto hit = clash(); hit != placed.end(); hit = clash())
        {
            box.y = hit->y + hit->height;
        }
        placed.push_back(box);

        cv::rectangle(image, box, label.colour, cv::FILLED);
        cv::putText(
            image,
            label.text,
            cv::Point(box.x + kLabelPaddingPx, box.y + kLabelPaddingPx + size.height),
            cv::FONT_HERSHEY_SIMPLEX,
            kFontScale,
            cv::Scalar(0, 0, 0),
            1,
            cv::LINE_AA);
    }
}

}  // namespace

const vision_msgs::msg::Detection3D*
measuredFor(const std::string& label, const vision_msgs::msg::Detection3DArray& objects)
{
    const vision_msgs::msg::Detection3D* found = nullptr;
    for (const vision_msgs::msg::Detection3D& detection : objects.detections)
    {
        if (detection.id == label)
        {
            found = &detection;
        }
        else if (ObjectTracker::phraseOf(detection.id) == label)
        {
            // Some track answers to this label, so it is that track's alias, not a measurement.
            return nullptr;
        }
    }
    return found;
}

G1PerceptionVisualizer::G1PerceptionVisualizer(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_perception_visualizer", options)
  , images_(
        declare_parameter<double>("image_history_s", 4.0),
        declare_parameter<double>("stamp_tolerance_ms", 50.0) / 1000.0,
        static_cast<std::size_t>(declare_parameter<int>("image_history_max_frames", 90)))
{
    fixed_frame_                  = declare_parameter<std::string>("fixed_frame", "odom");
    const std::string truth_topic = declare_parameter<std::string>("ground_truth_topic", "");

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "~/annotated_image",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "color/image_raw",
        sensorQos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr image) {
            images_.push(std::move(image));
            renderIfPaired();
        });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "color/camera_info",
        sensorQos(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
            camera_info_ = std::move(info);
        });
    masks_sub_ = create_subscription<g1_msgs::msg::InstanceMaskArray>(
        "tracked_masks",
        reliableQos(2),
        [this](g1_msgs::msg::InstanceMaskArray::ConstSharedPtr masks) {
            masks_ = std::move(masks);
            renderIfPaired();
        });
    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "object_poses",
        reliableQos(1),
        [this](vision_msgs::msg::Detection3DArray::ConstSharedPtr objects) {
            objects_ = std::move(objects);
            renderIfPaired();
        });

    if (!truth_topic.empty())
    {
        // Latched, so RViz shows it when it connects after the last update.
        truth_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/ground_truth",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        truth_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
            truth_topic,
            sensorQos(),
            [this](const vision_msgs::msg::Detection3DArray& truth) { storeTruth(truth); });
    }
}

void G1PerceptionVisualizer::renderIfPaired()
{
    // The geometry node publishes both for every frame, back to back but in no promised order.
    if (masks_ == nullptr || objects_ == nullptr ||
        masks_->header.stamp != objects_->header.stamp || rendered_stamp_ == masks_->header.stamp)
    {
        return;
    }
    const double stamp_s = DepthHistory::stampSeconds(masks_->header);
    const sensor_msgs::msg::Image::ConstSharedPtr frame = images_.at(stamp_s);
    if (frame == nullptr)
    {
        // Colour can land after masks cut from its depth twin, so the next image retries. Only
        // nothing at or before the stamp means the frame is gone for good.
        const sensor_msgs::msg::Image::ConstSharedPtr before = images_.atOrBefore(stamp_s);
        if (before == nullptr || DepthHistory::stampSeconds(before->header) > stamp_s)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "no colour frame is held from when the masks were cut; raise image_history_s");
        }
        return;
    }
    rendered_stamp_ = masks_->header.stamp;
    publishAnnotatedImage(*frame);
    if (truth_)
    {
        publishGroundTruth();
    }
}

void G1PerceptionVisualizer::publishAnnotatedImage(const sensor_msgs::msg::Image& frame)
{
    cv_bridge::CvImagePtr image;
    try
    {
        image = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::RGB8);
    }
    catch (const cv_bridge::Exception& error)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
        return;
    }

    const std::optional<Intrinsics> intrinsics =
        camera_info_ == nullptr ? std::nullopt :
                                  std::optional<Intrinsics>(Intrinsics{ camera_info_->k[0],
                                                                        camera_info_->k[4],
                                                                        camera_info_->k[2],
                                                                        camera_info_->k[5] });

    std::vector<Label> labels;
    labels.reserve(masks_->instances.size());
    for (const g1_msgs::msg::InstanceMask& instance : masks_->instances)
    {
        const cv::Scalar colour = colourFor(instance.label);
        drawMask(image->image, instance, colour);

        const vision_msgs::msg::Detection3D* measured = measuredFor(instance.label, *objects_);
        if (measured != nullptr && intrinsics)
        {
            drawBox(image->image, measured->bbox, *intrinsics, colour);
        }
        labels.push_back({ std::format(
                               "{} {:.2f}{}",
                               instance.label,
                               instance.score,
                               measured != nullptr ? "" : " unmeasured"),
                           cv::Point(
                               static_cast<int>(instance.roi.x_offset),
                               static_cast<int>(instance.roi.y_offset) - kLabelPaddingPx),
                           colour });
    }
    // Last, so no later mask or outline is drawn over a label.
    drawLabels(image->image, labels);
    annotated_pub_->publish(*image->toImageMsg());
}

void G1PerceptionVisualizer::storeTruth(const vision_msgs::msg::Detection3DArray& truth)
{
    vision_msgs::msg::Detection3DArray fixed;
    fixed.header.frame_id = fixed_frame_;
    for (const vision_msgs::msg::Detection3D& detection : truth.detections)
    {
        if (detection.results.empty())
        {
            continue;
        }
        geometry_msgs::msg::PoseStamped in_camera;
        // Unstamped for the newest transform: the relay stamps a millisecond ahead of TF.
        in_camera.header.frame_id = truth.header.frame_id;
        in_camera.pose            = detection.bbox.center;
        const std::optional<geometry_msgs::msg::PoseStamped> in_fixed =
            transformed(*tf_buffer_, in_camera, fixed_frame_);
        if (!in_fixed)
        {
            return;
        }
        fixed.detections.push_back(detection);
        fixed.detections.back().bbox.center = in_fixed->pose;
    }
    truth_ = std::move(fixed);
}

void G1PerceptionVisualizer::publishGroundTruth()
{
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.emplace_back().action = Marker::DELETEALL;

    int id = 0;
    for (const vision_msgs::msg::Detection3D& truth : truth_->detections)
    {
        const std::string& phrase = truth.results.front().hypothesis.class_id;

        Marker box;
        box.header.frame_id = fixed_frame_;
        box.ns              = "ground_truth";
        box.id              = id++;
        box.type            = Marker::CUBE;
        box.action          = Marker::ADD;
        box.pose            = truth.bbox.center;
        box.scale           = truth.bbox.size;
        box.color.r = box.color.g = box.color.b = 0.95F;
        box.color.a                             = kTruthAlpha;
        markers.markers.push_back(box);

        const std::optional<double> error = errorTo(phrase, truth.bbox.center.position);
        Marker                      label = box;
        label.ns                          = "ground_truth_error";
        label.id                          = id++;
        label.type                        = Marker::TEXT_VIEW_FACING;
        label.scale                       = geometry_msgs::msg::Vector3();
        label.scale.z                     = kLabelHeightM;
        label.color.a                     = 1.0F;
        label.text = error ? std::format("{} {:.0f} mm off", phrase, *error * 1000.0) :
                             std::format("{} not seen", phrase);
        label.pose.position.z += (0.5 * truth.bbox.size.z) + kLabelLiftM;
        markers.markers.push_back(label);
    }
    truth_pub_->publish(markers);
}

std::optional<double> G1PerceptionVisualizer::errorTo(
    const std::string& phrase, const geometry_msgs::msg::Point& truth) const
{
    std::optional<double> best;
    // A sole track's alias sits exactly on it, so counting both leaves the minimum unchanged.
    for (const vision_msgs::msg::Detection3D& seen : objects_->detections)
    {
        if (ObjectTracker::phraseOf(seen.id) != phrase)
        {
            continue;
        }
        geometry_msgs::msg::PointStamped in_camera;
        // At the image's own time: the camera may have moved while the detector ran.
        in_camera.header = objects_->header;
        in_camera.point  = seen.bbox.center.position;
        const std::optional<geometry_msgs::msg::PointStamped> in_fixed =
            transformed(*tf_buffer_, in_camera, fixed_frame_);
        if (!in_fixed)
        {
            continue;
        }
        const double distance = std::hypot(
            in_fixed->point.x - truth.x,
            in_fixed->point.y - truth.y,
            in_fixed->point.z - truth.z);
        best = std::min(best.value_or(distance), distance);
    }
    return best;
}

}  // namespace g1_perception
