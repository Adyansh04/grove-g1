#ifndef G1_STATE_ESTIMATION__G1_ODOMETRY_PUBLISHER_NODE_HPP_
#define G1_STATE_ESTIMATION__G1_ODOMETRY_PUBLISHER_NODE_HPP_

/**
 * @file g1_odometry_publisher_node.hpp
 * @brief LifecycleNode publishing odom -> base_link, from a source it names explicitly.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "g1_state_estimation/odom_math.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace g1_state_estimation
{

/**
 * @brief Publishes odom -> base_link and nav_msgs/Odometry from the configured source.
 *
 * Lifecycle rather than a plain node because the fail-loud requirement needs to be
 * externally observable: with `odometry_source=hardware` this returns FAILURE from
 * on_configure and sits in `unconfigured` having created no publisher and no broadcaster,
 * which a test can assert. "Logged an error and carried on" cannot be.
 */
class G1OdometryPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit G1OdometryPublisher(const rclcpp::NodeOptions& options);

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
    /// Reads and validates every parameter. False means configure must fail.
    bool readParameters();

    void onBaseState(const sensor_msgs::msg::JointState::SharedPtr msg);
    void onSportModeState(const unitree_go::msg::SportModeState::SharedPtr msg);
    /// Shared tail of both callbacks: staleness bookkeeping against a new stamp.
    void noteSample(const rclcpp::Time& stamp);
    void onTimer();

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr            base_state_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr         sport_state_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                           tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                             timer_;

    OdometrySource source_ = OdometrySource::Hardware;
    /// Topic the configured source actually reads. Held as a string because only one
    /// of the two subscriptions exists, and the other is null.
    std::string              source_topic_;
    std::string              odom_frame_id_;
    std::string              base_frame_id_;
    std::vector<std::string> base_joint_names_;
    double                   base_height_m_    = 0.0;
    double                   publish_rate_hz_  = 50.0;
    bool                     publish_odom_msg_ = true;
    double                   source_timeout_s_ = 0.2;
    double                   wall_timeout_s_   = 2.0;
    std::array<double, 36>   pose_covariance_{};
    std::array<double, 36>   twist_covariance_{};

    PlanarPose pose_;
    /// Height and full orientation. The planar track has neither (its body has no z
    /// DoF and cannot tilt); a walking G1 has both, so they are carried separately
    /// rather than forced through PlanarPose.
    double       pose_z_ = 0.0;
    Quaternion   orientation_;
    PlanarTwist  world_twist_;
    bool         have_sample_ = false;
    rclcpp::Time last_sample_stamp_;
    /// Wall time at which the sample stamp last changed.
    std::chrono::steady_clock::time_point last_advance_wall_{};
    /// Throttling clock for the staleness warnings; the ROS clock freezes with the sim.
    rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
};

}  // namespace g1_state_estimation

#endif  // G1_STATE_ESTIMATION__G1_ODOMETRY_PUBLISHER_NODE_HPP_
