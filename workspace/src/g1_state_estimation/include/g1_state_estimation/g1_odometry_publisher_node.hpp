#ifndef G1_STATE_ESTIMATION__G1_ODOMETRY_PUBLISHER_NODE_HPP_
#define G1_STATE_ESTIMATION__G1_ODOMETRY_PUBLISHER_NODE_HPP_

/**
 * @file g1_odometry_publisher_node.hpp
 * @brief LifecycleNode publishing the odom -> base chain, from a source it names explicitly.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "g1_state_estimation/odom_math.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_state_estimation
{

/**
 * @brief Publishes the odom -> base chain and nav_msgs/Odometry from the configured source.
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

    void onSportModeState(const unitree_go::msg::SportModeState::SharedPtr& msg);
    void onLowState(const unitree_hg::msg::LowState::SharedPtr& msg);
    void onLidarOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg);
    /// Latches odom_from_lio_ so the first LiDAR sample lands at a canonical start pose.
    /// False until the IMU has supplied a gravity-aligned attitude to level it against.
    bool latchLidarOrigin(const Pose3d& lio_from_base);
    /// Stores an orientation and re-derives the heading from it, holding the last good
    /// heading past max_tilt_rad_. Shared by every source that carries a full attitude.
    void applyOrientation(const Quaternion& q);
    /// LiDAR attitude with its slow drift against gravity taken out, using the IMU as the
    /// reference. Returns the input unchanged until an IMU attitude has arrived. Stateful:
    /// advances tilt_correction_ by one step per call.
    Quaternion levelledAttitude(const Quaternion& lidar_attitude);
    /// The transform from the frame the LiDAR odometry reports to the body frame this node
    /// publishes. Identity unless lidar_body_frame_id_ names something else. Refreshed per
    /// sample rather than cached: on the robot that chain crosses the waist joints.
    bool lookUpLidarBodyOffset();
    /// Shared tail of the position callbacks: staleness bookkeeping against a new stamp.
    void noteSample(const rclcpp::Time& stamp);
    void onTimer();

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr         sport_state_sub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr               low_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                 lidar_odom_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>                           tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer>                                         tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>                              tf_listener_;
    rclcpp::TimerBase::SharedPtr                                             timer_;

    OdometrySource source_ = OdometrySource::kHardware;
    /// Topic the configured source actually reads. Held as a string because only one
    /// of the two subscriptions exists, and the other is null.
    std::string source_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    /// Body link hung under base_frame_id_, carrying the height and tilt the footprint drops.
    /// Empty publishes a single odom -> base_frame_id_ edge with the full pose; naming a link
    /// here splits it into a ground-projected edge plus a second edge carrying the height and
    /// tilt (see GroundSplit).
    std::string pelvis_frame_id_;
    /// Frame the LiDAR odometry reports the pose OF -- FAST-LIO's `body`, which is its IMU.
    /// Empty means that frame already is the body this node publishes. It is `mid360_imu` on
    /// both tracks: the simulator models an IMU in the sensor housing as the robot has one.
    std::string lidar_body_frame_id_;
    /// Beyond this the heading is ill-conditioned and the last good one is held instead.
    double                 max_tilt_rad_     = 0.0;
    double                 start_height_m_   = 0.0;
    double                 publish_rate_hz_  = 50.0;
    bool                   publish_odom_msg_ = true;
    double                 source_timeout_s_ = 0.2;
    double                 wall_timeout_s_   = 2.0;
    std::array<double, 36> pose_covariance_{};
    std::array<double, 36> twist_covariance_{};

    PlanarPose pose_;
    /// Height and full orientation, carried separately from PlanarPose: a walking G1 has both a
    /// height and a tilt that a flat 2D pose cannot express.
    double     pose_z_ = 0.0;
    Quaternion orientation_;
    /// Set once a usable orientation has arrived. Until then nothing is published:
    /// an unusable quaternion must not reach TF (see onLowState).
    bool have_orientation_ = false;
    /// Latest validated IMU attitude. The fast_lio source levels its odom frame against this at
    /// the latch, and keeps using it afterwards as the gravity reference that FAST-LIO's own
    /// estimate drifts away from.
    Quaternion imu_orientation_;
    bool       have_imu_orientation_ = false;
    /// Low-passed tilt error between FAST-LIO's attitude and the IMU's, applied to every
    /// published attitude. Slow on purpose -- see levelledAttitude().
    Quaternion tilt_correction_;
    /// Per-sample slerp fraction toward the instantaneous error, from `tilt_correction_gain`.
    double tilt_correction_gain_ = 0.05;
    /// odom -> the LiDAR odometry's own start frame. FAST-LIO's `camera_init` is wherever its
    /// IMU happened to be pointing when it initialised, not a gravity-aligned world frame, so
    /// this is what turns its output into something Nav2 can consume.
    Pose3d odom_from_lio_;
    bool   lidar_origin_latched_ = false;
    /// Body offset from lidar_body_frame_id_, refreshed from TF per sample.
    Pose3d lio_body_from_base_;
    /// Wall time the orientation last changed. Position and orientation come from separate
    /// topics, so one can die while the other keeps flowing; without this the node would
    /// publish a frozen orientation under a fresh stamp, which is the exact failure this
    /// publisher exists to refuse.
    std::chrono::steady_clock::time_point last_orientation_wall_{};
    PlanarTwist                           world_twist_;
    bool                                  have_sample_ = false;
    rclcpp::Time                          last_sample_stamp_;
    /// Wall time at which the sample stamp last changed.
    std::chrono::steady_clock::time_point last_advance_wall_{};
    /// Throttling clock for the staleness warnings; the ROS clock freezes with the sim.
    rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
};

}  // namespace g1_state_estimation

#endif  // G1_STATE_ESTIMATION__G1_ODOMETRY_PUBLISHER_NODE_HPP_
