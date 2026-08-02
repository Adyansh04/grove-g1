#include "g1_state_estimation/g1_odometry_publisher_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace g1_state_estimation
{

namespace
{
// Sensor-style: the base state is a stream of samples where only the newest matters, and
// a reliable subscriber against a best-effort publisher simply receives nothing.
rclcpp::QoS baseStateQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
}
}  // namespace

G1OdometryPublisher::G1OdometryPublisher(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("g1_odometry_publisher", options)
{
    declare_parameter<std::string>("odometry_source", "hardware");
    declare_parameter<std::string>("odom_frame_id", "odom");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<std::vector<std::string>>(
        "base_joint_names",
        { "base_x_joint", "base_y_joint", "base_yaw_joint" });
    // Height of base_link above the floor, making odom the ground plane. Defaults to 0,
    // which is correct for a real floating-base estimator (it measures z itself). The
    // perception sim's base is a planar body with no z DoF, so its launch supplies the
    // spawn height from g1_sim's sensor_mounts.yaml.
    declare_parameter<double>("base_height_m", 0.0);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<bool>("publish_odom_msg", true);
    declare_parameter<double>("source_timeout_ms", 200.0);
    declare_parameter<double>("pose_covariance", 1.0e-6);
    declare_parameter<double>("twist_covariance", 1.0e-6);
}

bool G1OdometryPublisher::readParameters()
{
    const std::string source_name = get_parameter("odometry_source").as_string();
    if (!parseOdometrySource(source_name, source_))
    {
        RCLCPP_ERROR(
            get_logger(),
            "odometry_source='%s' is not a known source. Use 'sim_ground_truth' or "
            "'hardware'.",
            source_name.c_str());
        return false;
    }

    if (source_ == OdometrySource::Hardware)
    {
        // Deliberately long. Anyone hitting this needs to know the topic they are about to
        // go looking for does not carry what they think it does.
        RCLCPP_ERROR(
            get_logger(),
            "odometry_source='hardware' is not implemented: the real G1 publishes no odometry. "
            "On hardware /sportmodestate carries unitree_hg::SportModeState_, which has only "
            "fsm_id, fsm_mode, task_id and task_time -- no pose and no velocity. "
            "rt/odommodestate does not exist. A real source (leg odometry + IMU EKF, or "
            "LiDAR-inertial odometry) is a future state-estimation milestone. Refusing to "
            "configure rather than publish a fabricated transform.");
        return false;
    }

    base_height_m_    = get_parameter("base_height_m").as_double();
    odom_frame_id_    = get_parameter("odom_frame_id").as_string();
    base_frame_id_    = get_parameter("base_frame_id").as_string();
    base_joint_names_ = get_parameter("base_joint_names").as_string_array();
    publish_rate_hz_  = get_parameter("publish_rate_hz").as_double();
    publish_odom_msg_ = get_parameter("publish_odom_msg").as_bool();
    source_timeout_s_ = get_parameter("source_timeout_ms").as_double() / 1000.0;

    if (base_joint_names_.size() != 3)
    {
        RCLCPP_ERROR(
            get_logger(),
            "base_joint_names needs exactly 3 entries (x, y, yaw), got %zu",
            base_joint_names_.size());
        return false;
    }
    if (publish_rate_hz_ <= 0.0)
    {
        RCLCPP_ERROR(get_logger(), "publish_rate_hz must be positive, got %f", publish_rate_hz_);
        return false;
    }

    pose_covariance_  = diagonalCovariance(get_parameter("pose_covariance").as_double());
    twist_covariance_ = diagonalCovariance(get_parameter("twist_covariance").as_double());
    return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_configure(const rclcpp_lifecycle::State&)
{
    // Nothing is created before this returns true. An unimplemented source must leave no
    // publisher and no broadcaster behind: advertising /tf and then going quiet is exactly
    // the silent failure this node exists to avoid.
    if (!readParameters())
    {
        return CallbackReturn::FAILURE;
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    if (publish_odom_msg_)
    {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(10));
    }
    base_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "~/base_state",
        baseStateQos(),
        std::bind(&G1OdometryPublisher::onBaseState, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Configured on sim ground truth: %s -> %s from %s. This is exact MuJoCo state, not an "
        "estimate; it has no drift, noise or latency.",
        odom_frame_id_.c_str(),
        base_frame_id_.c_str(),
        base_state_sub_->get_topic_name());
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_cleanup(const rclcpp_lifecycle::State&)
{
    timer_.reset();
    base_state_sub_.reset();
    odom_pub_.reset();
    tf_broadcaster_.reset();
    have_sample_ = false;
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    LifecycleNode::on_activate(previous_state);
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_            = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&G1OdometryPublisher::onTimer, this));
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    timer_.reset();
    LifecycleNode::on_deactivate(previous_state);
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_shutdown(const rclcpp_lifecycle::State&)
{
    timer_.reset();
    base_state_sub_.reset();
    odom_pub_.reset();
    tf_broadcaster_.reset();
    return CallbackReturn::SUCCESS;
}

void G1OdometryPublisher::onBaseState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Look joints up by name every message: joint_state_broadcaster does not promise a
    // stable ordering, and indexing by position would silently swap x and yaw the day it
    // changes.
    double     values[3]     = { 0.0, 0.0, 0.0 };
    double     velocities[3] = { 0.0, 0.0, 0.0 };
    const bool have_velocity = msg->velocity.size() == msg->name.size();

    for (std::size_t axis = 0; axis < base_joint_names_.size(); ++axis)
    {
        const auto it = std::find(msg->name.begin(), msg->name.end(), base_joint_names_[axis]);
        if (it == msg->name.end())
        {
            return;
        }
        const std::size_t index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
        if (index >= msg->position.size())
        {
            return;
        }
        values[axis]     = msg->position[index];
        velocities[axis] = have_velocity ? msg->velocity[index] : 0.0;
    }

    pose_.x      = values[0];
    pose_.y      = values[1];
    pose_.yaw    = values[2];
    world_twist_ = PlanarTwist{ velocities[0], velocities[1], velocities[2] };

    // The broadcaster stamps from this, so take the source's own time rather than now():
    // on simulated time the two can differ by more than the whole TF cache.
    const bool         unstamped = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0;
    const rclcpp::Time stamp =
        unstamped ? now() : rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());

    // Wall time of the last stamp CHANGE, not of the last message. A simulator whose
    // physics thread has wedged can keep republishing the same sample at the same stamp
    // forever; that is not fresh data, and treating message arrival as freshness would
    // hold the staleness guard open for exactly that failure.
    if (!have_sample_ || stamp != last_sample_stamp_)
    {
        last_advance_wall_ = std::chrono::steady_clock::now();
    }
    last_sample_stamp_ = stamp;
    have_sample_       = true;
}

void G1OdometryPublisher::onTimer()
{
    if (!have_sample_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "No base state received yet on %s; publishing nothing.",
            base_state_sub_->get_topic_name());
        return;
    }

    // Two budgets, because on this track they can fail independently. /clock is published
    // by the SAME process as the base state, so if that process wedges, sim time freezes
    // with it, `elapsed` stays pinned near zero and a sim-time-only check never fires --
    // the exact "confidently wrong map" this guard exists to prevent. The wall budget
    // measures time since the sample stamp last ADVANCED, so it catches both a silent
    // source and one still emitting a frozen sample.
    const double elapsed = (now() - last_sample_stamp_).seconds();
    const double wall_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_advance_wall_).count();
    if (isStale(elapsed, source_timeout_s_) || isStale(wall_elapsed, source_timeout_s_))
    {
        // Stop publishing rather than re-stamping the last pose. A frozen transform with a
        // fresh timestamp is indistinguishable from a stationary robot, which is how a dead
        // source turns into a confidently wrong map.
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Base state is %.3f s old (timeout %.3f s); stopped publishing %s -> %s.",
            elapsed,
            source_timeout_s_,
            odom_frame_id_.c_str(),
            base_frame_id_.c_str());
        return;
    }

    const Quaternion   orientation = yawToQuaternion(pose_.yaw);
    const PlanarTwist  body_twist  = toBodyTwist(world_twist_, pose_.yaw);
    const rclcpp::Time stamp       = last_sample_stamp_;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = stamp;
    tf.header.frame_id         = odom_frame_id_;
    tf.child_frame_id          = base_frame_id_;
    tf.transform.translation.x = pose_.x;
    tf.transform.translation.y = pose_.y;
    tf.transform.translation.z = base_height_m_;
    tf.transform.rotation.x    = orientation.x;
    tf.transform.rotation.y    = orientation.y;
    tf.transform.rotation.z    = orientation.z;
    tf.transform.rotation.w    = orientation.w;
    tf_broadcaster_->sendTransform(tf);

    if (!odom_pub_ || !odom_pub_->is_activated())
    {
        return;
    }

    // Nav2's costmap and controller server read velocity from Odometry, not from TF.
    nav_msgs::msg::Odometry odom;
    odom.header.stamp          = stamp;
    odom.header.frame_id       = odom_frame_id_;
    odom.child_frame_id        = base_frame_id_;
    odom.pose.pose.position.x  = pose_.x;
    odom.pose.pose.position.y  = pose_.y;
    odom.pose.pose.position.z  = base_height_m_;
    odom.pose.pose.orientation = tf.transform.rotation;
    odom.twist.twist.linear.x  = body_twist.vx;
    odom.twist.twist.linear.y  = body_twist.vy;
    odom.twist.twist.angular.z = body_twist.omega;
    std::copy(pose_covariance_.begin(), pose_covariance_.end(), odom.pose.covariance.begin());
    std::copy(twist_covariance_.begin(), twist_covariance_.end(), odom.twist.covariance.begin());
    odom_pub_->publish(odom);
}

}  // namespace g1_state_estimation
