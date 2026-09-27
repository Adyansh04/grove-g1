#ifndef CANOPY_PERCEPTION__MOCK_DETECTOR_NODE_HPP_
#define CANOPY_PERCEPTION__MOCK_DETECTOR_NODE_HPP_

/**
 * @file mock_detector_node.hpp
 * @brief A detector stand-in that needs no GPU, model or network.
 *
 * Cuts masks from simulator ground-truth poses against the rendered depth, so everything after
 * the mask runs for real. SIMULATION ONLY: the robot has no ground truth.
 */

#include <canopy_msgs/msg/instance_mask_array.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "canopy_perception/depth_history.hpp"

namespace canopy_perception
{

class MockDetector : public rclcpp::Node
{
public:
    explicit MockDetector(const rclcpp::NodeOptions& options);

private:
    void onTruth(vision_msgs::msg::Detection3DArray::ConstSharedPtr truth);
    void onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth);
    void publishMasks();

    /// The pixels of @p detection in @p depth as an InstanceMask, or nothing when it is out of
    /// view or under min_pixels.
    [[nodiscard]] std::optional<canopy_msgs::msg::InstanceMask> maskFor(
        const vision_msgs::msg::Detection3D& detection, const sensor_msgs::msg::Image& depth,
        const std::string& phrase) const;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr truth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr       info_sub_;
    rclcpp::Publisher<canopy_msgs::msg::InstanceMaskArray>::SharedPtr   masks_pub_;
    rclcpp::TimerBase::SharedPtr                                        timer_;

    DepthHistory                                       depth_frames_;
    vision_msgs::msg::Detection3DArray::ConstSharedPtr truth_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr       camera_info_;

    double latency_s_{ 0.0 };
    double margin_m_{ 0.0 };
    int    min_pixels_{ 50 };
};

}  // namespace canopy_perception

#endif  // CANOPY_PERCEPTION__MOCK_DETECTOR_NODE_HPP_
