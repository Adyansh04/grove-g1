#ifndef G1_PERCEPTION__G1_OBJECT_GEOMETRY_NODE_HPP_
#define G1_PERCEPTION__G1_OBJECT_GEOMETRY_NODE_HPP_

/**
 * @file g1_object_geometry_node.hpp
 * @brief Turns instance masks plus aligned depth into the object poses the skills consume.
 *
 * The boundary between "which pixels" and "where in the world". Everything above this node works
 * in metres and frames, and nothing above it knows which model drew the masks.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <g1_msgs/msg/instance_mask_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_perception/depth_history.hpp"
#include "g1_perception/object_geometry.hpp"
#include "g1_perception/object_tracker.hpp"

namespace g1_perception
{

class G1ObjectGeometry : public rclcpp::Node
{
public:
    explicit G1ObjectGeometry(const rclcpp::NodeOptions& options);

private:
    /// One instance that survived every geometric check.
    struct Measured
    {
        std::string phrase;
        float       score{ 0.0F };
        OrientedBox box;
        /// The box's centre and axes as a pose, in the camera frame the masks were stamped with.
        geometry_msgs::msg::Pose pose_in_camera;
        Point3                   position_in_up_frame;
        std::size_t              instance_index{ 0 };
    };

    void onMasks(const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks);
    void onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth);
    void onCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

    /// Geometry for one instance, or nothing when it is too small, too far or off the surface.
    std::optional<OrientedBox> measure(
        const g1_msgs::msg::InstanceMask& instance, const sensor_msgs::msg::Image& depth,
        const Intrinsics& intrinsics, const Point3& up) const;

    void publish(
        const g1_msgs::msg::InstanceMaskArray& masks, const std::vector<Measured>& measured,
        const std::vector<std::string>& ids);

    rclcpp::Subscription<g1_msgs::msg::InstanceMaskArray>::SharedPtr masks_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr    info_sub_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
    rclcpp::Publisher<g1_msgs::msg::InstanceMaskArray>::SharedPtr    tracked_pub_;

    /// Masks run on their own group so their TF wait cannot stall depth ingest.
    rclcpp::CallbackGroup::SharedPtr masks_group_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// Guards depth_history_ and camera_info_: onMasks reads them while onDepth writes.
    std::mutex                                   frames_mutex_;
    DepthHistory                                 depth_history_;
    ObjectTracker                                tracker_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;

    std::string up_frame_;
    int         mask_erosion_px_{ 2 };
    int         support_ring_px_{ 6 };
    double      min_depth_m_{ 0.15 };
    double      max_depth_m_{ 2.5 };
    double      depth_gate_m_{ 0.15 };
    int         min_points_{ 150 };
    int         min_support_points_{ 50 };
    double      support_band_m_{ 0.06 };
    double      min_extent_m_{ 0.01 };
    double      max_extent_m_{ 0.40 };
    double      min_score_{ 0.30 };
    double      transform_timeout_s_{ 0.2 };
    bool        publish_bare_phrase_alias_{ true };
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__G1_OBJECT_GEOMETRY_NODE_HPP_
