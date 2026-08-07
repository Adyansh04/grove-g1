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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace g1_manipulation
{

/// Where object poses come from. There is no "best available" fallback on purpose.
enum class ObjectSource
{
    /// MuJoCo body poses, sampled inside the simulator and carried by g1_sensor_relay.
    SimGroundTruth,
    /// Not implemented. Refuses to configure; see the node's on_configure.
    Hardware,
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
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
    bool readParameters();
    void onGroundTruth(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

    ObjectSource source_{ ObjectSource::Hardware };
    std::string  output_frame_id_;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr                 source_sub_;
    rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_OBJECT_POSE_SOURCE_NODE_HPP_
