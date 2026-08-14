#ifndef G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_
#define G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_

/**
 * @file g1_object_pose_source_node.hpp
 * @brief Publishes the poses of manipulable objects, and owns where they are allowed to come
 *        from.
 *
 * The boundary between manipulation and perception. Skills consume `/objects` and never learn
 * which source filled it, so a real detector replaces this node without touching them.
 *
 * Shaped after g1_state_estimation's odometry publisher, down to `hardware` being the default
 * and refusing to configure: a bring-up that forgets to say which source it has must fail
 * visibly rather than feed a grasp planner simulator ground truth.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace g1_manipulation
{

/// Where object poses come from. There is no "best available" fallback on purpose.
enum class ObjectSource
{
    /// MuJoCo body poses, sampled inside the simulator and carried by g1_sensor_relay.
    kSimGroundTruth,
    /// Not implemented. Refuses to configure; see the node's on_configure.
    kHardware,
};

/// False if the name is not a known source, leaving `out` untouched.
bool parseObjectSource(const std::string& name, ObjectSource& out);

class G1ObjectPoseSource : public rclcpp_lifecycle::LifecycleNode
{
public:
    explicit G1ObjectPoseSource(const rclcpp::NodeOptions& options);

    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

private:
    bool readParameters();
    void onGroundTruth(vision_msgs::msg::Detection3DArray::SharedPtr msg);
    void publishMarkers(const vision_msgs::msg::Detection3DArray& objects);

    ObjectSource source_{ ObjectSource::kHardware };
    bool         publish_markers_{ false };
    std::string  source_frame_id_;
    std::string  output_frame_id_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr                 source_sub_;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
    /// Only created when publish_markers is set: an rviz aid, not part of the interface, and
    /// nothing should grow a dependency on it.
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_
