#ifndef G1_PERCEPTION__PERCEPTION_VISUALIZER_NODE_HPP_
#define G1_PERCEPTION__PERCEPTION_VISUALIZER_NODE_HPP_

/**
 * @file perception_visualizer_node.hpp
 * @brief Draws perception's output for a person: the annotated frame and ground-truth boxes.
 *
 * Nothing downstream reads what it publishes.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <builtin_interfaces/msg/time.hpp>
#include <g1_msgs/msg/instance_mask_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "g1_perception/depth_history.hpp"

namespace g1_perception
{

/**
 * @brief The detection the geometry node measured for the instance labelled @p label.
 *
 * @return Null when that instance was rejected. A rejected instance keeps its raw label, which
 *         for a one-word phrase equals another track's bare-phrase alias; that is not a match.
 */
[[nodiscard]] const vision_msgs::msg::Detection3D*
measuredFor(const std::string& label, const vision_msgs::msg::Detection3DArray& objects);

class G1PerceptionVisualizer : public rclcpp::Node
{
public:
    explicit G1PerceptionVisualizer(const rclcpp::NodeOptions& options);

private:
    /// Renders once one frame's masks, poses and colour image are all in hand, in whichever order.
    void renderIfPaired();

    /// The frame the masks were cut from, tinted and outlined per object, boxed and labelled.
    void publishAnnotatedImage(const sensor_msgs::msg::Image& frame);

    /// Keeps @p truth in the fixed frame, moved there while the camera is still where it was.
    void storeTruth(const vision_msgs::msg::Detection3DArray& truth);

    /// Ground-truth boxes, each labelled with how far the perceived object is from it.
    void publishGroundTruth();

    /// How far the nearest perceived object named @p phrase is from @p truth, in metres.
    [[nodiscard]] std::optional<double>
    errorTo(const std::string& phrase, const geometry_msgs::msg::Point& truth) const;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr            image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr       info_sub_;
    rclcpp::Subscription<g1_msgs::msg::InstanceMaskArray>::SharedPtr    masks_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr truth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               annotated_pub_;
    /// Null without a ground-truth topic, which is every run on the robot.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr truth_pub_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    DepthHistory                                       images_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr       camera_info_;
    g1_msgs::msg::InstanceMaskArray::ConstSharedPtr    masks_;
    vision_msgs::msg::Detection3DArray::ConstSharedPtr objects_;
    /// In the fixed frame, and only objects that carry a class.
    std::optional<vision_msgs::msg::Detection3DArray> truth_;
    std::optional<builtin_interfaces::msg::Time>      rendered_stamp_;

    std::string fixed_frame_;
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__PERCEPTION_VISUALIZER_NODE_HPP_
