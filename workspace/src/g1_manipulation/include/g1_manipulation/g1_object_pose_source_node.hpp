#ifndef G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_
#define G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_

/**
 * @file g1_object_pose_source_node.hpp
 * @brief Publishes the poses of manipulable objects, and owns where they may come from.
 *
 * Skills consume `/objects` and never learn which source filled it. The default source,
 * `hardware`, refuses to configure, so a bring-up that names none fails visibly.
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

/**
 * @brief Where object poses come from. Deliberately no "best available" fallback.
 */
enum class ObjectSource
{
    /// MuJoCo body poses, carried out of the simulator by g1_sensor_relay.
    kSimGroundTruth,
    /// Measured by g1_perception from the camera, in simulation or on the robot.
    kPerception,
    /// Not implemented; refuses to configure.
    kHardware,
};

/**
 * @brief Parses the `object_source` parameter.
 *
 * @param name Parameter value.
 * @param[out] out Set only when the name is recognised.
 * @return False if the name is not a known source, leaving @p out untouched.
 */
bool parseObjectSource(const std::string& name, ObjectSource& out);

/**
 * @brief Whether @p id is the bare-phrase alias of a tracked id in @p objects.
 *
 * Perception publishes a sole track twice, as `red_block_0` and as `red_block`; drawn both times,
 * the two labels stack into one unreadable one.
 */
[[nodiscard]] bool
isBarePhraseAlias(const std::string& id, const vision_msgs::msg::Detection3DArray& objects);

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
    void onObjectPoses(vision_msgs::msg::Detection3DArray::SharedPtr msg);
    void publishMarkers(const vision_msgs::msg::Detection3DArray& objects);

    ObjectSource source_{ ObjectSource::kHardware };
    bool         publish_markers_{ false };
    std::string  source_frame_id_;
    std::string  output_frame_id_;
    /// Wait for a transform at the detection's stamp. Must cover the gap between odom updates: a
    /// stamp newer than the last odom resolves only when the next one arrives.
    double transform_timeout_s_{ 0.5 };

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr source_sub_;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
    /// Null unless publish_markers is set: an RViz aid, not part of the interface.
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        markers_pub_;
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_
