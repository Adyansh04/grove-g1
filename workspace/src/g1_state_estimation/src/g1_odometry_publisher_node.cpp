/**
 * @file g1_odometry_publisher_node.cpp
 * @brief Publishes the odom -> base chain and nav_msgs/Odometry from the configured source.
 */

#include "g1_state_estimation/g1_odometry_publisher_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
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

// Validates and normalises an attitude before it can reach TF: a zero-norm quaternion becomes
// NaN inside tf2 and the transform is dropped against tf2's name rather than this one. A
// norm^2 test that admits 1.0 also admits 4.0, and nothing reports an unscaled /tf.
std::optional<Quaternion> normalisedAttitude(const geometry_msgs::msg::Quaternion& q)
{
    const double norm2 = (q.w * q.w) + (q.x * q.x) + (q.y * q.y) + (q.z * q.z);
    if (!std::isfinite(norm2) || norm2 < 0.5)
    {
        return std::nullopt;
    }
    const double inv = 1.0 / std::sqrt(norm2);
    return Quaternion{ q.x * inv, q.y * inv, q.z * inv, q.w * inv };
}
}  // namespace

G1OdometryPublisher::G1OdometryPublisher(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("g1_odometry_publisher", options)
{
    declare_parameter<std::string>("odometry_source", "hardware");
    declare_parameter<std::string>("odom_frame_id", "odom");
    // REP-105: gravity-aligned and on the ground. Nav2's robot_base_frame and slam_toolbox's
    // base_frame both default to a frame like this, and a 2D costmap has nowhere to put tilt.
    declare_parameter<std::string>("base_frame_id", "base_footprint");
    // Empty means one edge carrying the full pose; naming a link splits it in two (see
    // GroundSplit).
    declare_parameter<std::string>("pelvis_frame_id", "");
    declare_parameter<double>("max_tilt_deg", 80.0);
    // Empty means the LiDAR odometry already reports the frame this node publishes.
    declare_parameter<std::string>("lidar_body_frame_id", "");
    // Body height above the floor when the LiDAR odometry starts, which puts odom on the
    // ground plane. Only fast_lio uses it: that source measures height relative to wherever
    // it initialised and has no idea where the floor is.
    declare_parameter<double>("start_height_m", 0.0);
    // How fast the published tilt is pulled toward the IMU's. See levelledAttitude() for why
    // this has to be slow, and the shipped config for the numbers.
    declare_parameter<double>("tilt_correction_gain", 0.05);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<bool>("publish_odom_msg", true);
    declare_parameter<double>("source_timeout_ms", 200.0);
    declare_parameter<double>("wall_timeout_ms", 2000.0);
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
            "odometry_source='%s' is not a known source. Use 'ground_truth' (exact simulator "
            "state), 'fast_lio' (LiDAR-inertial, and the only one that runs on the robot) or "
            "'hardware'.",
            source_name.c_str());
        return false;
    }

    if (source_ == OdometrySource::kHardware)
    {
        // Deliberately long. Anyone hitting this needs to know the topic they are about to
        // go looking for does not carry what they think it does.
        RCLCPP_ERROR(
            get_logger(),
            "odometry_source='hardware' is not a source: the real G1 publishes no odometry of "
            "its own. Its sport-mode state carries only fsm_id, fsm_mode, task_id and "
            "task_time -- no pose and no velocity -- and rt/odommodestate does not exist. "
            "Use odometry_source='fast_lio' and bring up "
            "g1_state_estimation's fastlio_odometry.launch.py, which runs the LiDAR-inertial "
            "pipeline this reads. Refusing to configure rather than publish a fabricated "
            "transform.");
        return false;
    }

    start_height_m_       = get_parameter("start_height_m").as_double();
    tilt_correction_gain_ = get_parameter("tilt_correction_gain").as_double();
    odom_frame_id_        = get_parameter("odom_frame_id").as_string();
    base_frame_id_        = get_parameter("base_frame_id").as_string();
    pelvis_frame_id_      = get_parameter("pelvis_frame_id").as_string();
    lidar_body_frame_id_  = get_parameter("lidar_body_frame_id").as_string();
    max_tilt_rad_         = get_parameter("max_tilt_deg").as_double() * M_PI / 180.0;
    publish_rate_hz_      = get_parameter("publish_rate_hz").as_double();
    publish_odom_msg_     = get_parameter("publish_odom_msg").as_bool();
    source_timeout_s_     = get_parameter("source_timeout_ms").as_double() / 1000.0;
    wall_timeout_s_       = get_parameter("wall_timeout_ms").as_double() / 1000.0;

    if (publish_rate_hz_ <= 0.0)
    {
        RCLCPP_ERROR(get_logger(), "publish_rate_hz must be positive, got %f", publish_rate_hz_);
        return false;
    }
    // Empty or self-referential frame ids reach tf2 as an error naming tf2, not this node.
    if (base_frame_id_.empty() || pelvis_frame_id_ == base_frame_id_)
    {
        RCLCPP_ERROR(
            get_logger(),
            "base_frame_id ('%s') must be non-empty and different from pelvis_frame_id ('%s'). "
            "Leave pelvis_frame_id empty for a single transform.",
            base_frame_id_.c_str(),
            pelvis_frame_id_.c_str());
        return false;
    }
    if (max_tilt_rad_ <= 0.0 || max_tilt_rad_ >= M_PI)
    {
        RCLCPP_ERROR(
            get_logger(),
            "max_tilt_deg must be in (0, 180), got %f",
            get_parameter("max_tilt_deg").as_double());
        return false;
    }
    // 1.0 would substitute the IMU's tilt outright, which is the timing error levelledAttitude()
    // exists to avoid; 0.0 is the honest way to turn the correction off.
    if (tilt_correction_gain_ < 0.0 || tilt_correction_gain_ >= 1.0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "tilt_correction_gain must be in [0, 1), got %f",
            tilt_correction_gain_);
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
    // Every callback below takes its SharedPtr by value because rclcpp offers no const-ref
    // dispatch for a mutable pointee; the handlers they forward to do take const-ref.
    // NOLINTBEGIN(performance-unnecessary-value-param)
    if (source_ == OdometrySource::kGroundTruth)
    {
        // Pose and twist in one message, so unlike the estimator below there is no second
        // topic whose attitude could go stale independently.
        ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "~/base_state",
            baseStateQos(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { onGroundTruth(msg); });
    }
    else
    {
        lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "~/lidar_odometry",
            baseStateQos(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { onLidarOdometry(msg); });
        // The attitude is needed once, to level the odom frame at the latch: the LiDAR
        // odometry's own start frame is wherever its IMU was pointing, which is not gravity.
        // This must be the pelvis IMU, because what it levels is the pelvis attitude.
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "~/imu",
            baseStateQos(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { onImu(msg); });

        if (!lidar_body_frame_id_.empty())
        {
            tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
        }
    }
    // NOLINTEND(performance-unnecessary-value-param)

    source_topic_ =
        ground_truth_sub_ ? ground_truth_sub_->get_topic_name() : lidar_odom_sub_->get_topic_name();
    const std::string chain = pelvis_frame_id_.empty() ? odom_frame_id_ + " -> " + base_frame_id_ :
                                                         odom_frame_id_ + " -> " + base_frame_id_ +
                                                             " -> " + pelvis_frame_id_;
    if (source_ == OdometrySource::kGroundTruth)
    {
        RCLCPP_INFO(
            get_logger(),
            "Configured on sim ground truth: %s from %s. This is exact MuJoCo state, not an "
            "estimate; it has no drift, noise or latency.",
            chain.c_str(),
            source_topic_.c_str());
    }
    else
    {
        RCLCPP_INFO(
            get_logger(),
            "Configured on LiDAR-inertial odometry: %s from %s. Unlike the sim source this is "
            "an estimate and it drifts; correcting it is what map -> odom is for.",
            chain.c_str(),
            source_topic_.c_str());
    }
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_cleanup(const rclcpp_lifecycle::State&)
{
    timer_.reset();
    ground_truth_sub_.reset();
    imu_sub_.reset();
    lidar_odom_sub_.reset();
    odom_pub_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    have_sample_          = false;
    have_orientation_     = false;
    have_imu_orientation_ = false;
    // Neither is gated by a have_* flag, so a re-configure would otherwise publish the previous
    // session's tilt correction, and its first ~/odom the previous session's velocity: dt is
    // zero on the first sample, so the twist branch is skipped.
    tilt_correction_ = Quaternion{};
    world_twist_     = PlanarTwist{};
    // Cleared with the rest: a re-configure is a fresh start, and reusing the old origin would
    // silently place the new run in the previous run's odom frame.
    lidar_origin_latched_ = false;
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    // Checked, not discarded: this is what activates odom_pub_. Ignoring it starts the timer
    // and reports SUCCESS against a publisher that never came up, and TF still goes out.
    const auto base_result = LifecycleNode::on_activate(previous_state);
    if (base_result != CallbackReturn::SUCCESS)
    {
        return base_result;
    }
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {
            onTimer();
        });
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    timer_.reset();
    return LifecycleNode::on_deactivate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
G1OdometryPublisher::on_shutdown(const rclcpp_lifecycle::State&)
{
    timer_.reset();
    ground_truth_sub_.reset();
    imu_sub_.reset();
    lidar_odom_sub_.reset();
    odom_pub_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
}

bool G1OdometryPublisher::lookUpLidarBodyOffset()
{
    if (lidar_body_frame_id_.empty())
    {
        return true;  // identity, already the default
    }
    const std::string& body = pelvis_frame_id_.empty() ? base_frame_id_ : pelvis_frame_id_;
    try
    {
        // Not cached: the sensor-to-pelvis chain crosses the three waist joints, which the
        // policy drives through tens of degrees. TimePointZero pairs a fresh waist state with a
        // scan one FAST-LIO period old; the residual is bounded by one gait step.
        const auto tf = tf_buffer_->lookupTransform(lidar_body_frame_id_, body, tf2::TimePointZero);
        lio_body_from_base_.x = tf.transform.translation.x;
        lio_body_from_base_.y = tf.transform.translation.y;
        lio_body_from_base_.z = tf.transform.translation.z;
        lio_body_from_base_.q = Quaternion{ tf.transform.rotation.x,
                                            tf.transform.rotation.y,
                                            tf.transform.rotation.z,
                                            tf.transform.rotation.w };
        return true;
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "Waiting for %s -> %s: %s",
            lidar_body_frame_id_.c_str(),
            body.c_str(),
            e.what());
        return false;
    }
}

bool G1OdometryPublisher::latchLidarOrigin(const Pose3d& lio_from_base)
{
    if (!have_imu_orientation_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "Have LiDAR odometry but no IMU attitude yet; cannot level the odom frame.");
        return false;
    }

    // The latch happens once and is never revisited, so a garbage tilt here is baked into odom
    // for the whole run. Past this angle the robot is falling, not standing at an origin.
    const double tilt = tiltFromVertical(imu_orientation_);
    if (tilt > max_tilt_rad_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            2000,
            "Not latching the odom origin at %.1f degrees from vertical (limit %.1f); waiting "
            "for the robot to be upright.",
            tilt * 180.0 / M_PI,
            max_tilt_rad_ * 180.0 / M_PI);
        return false;
    }

    // Where the robot is declared to have been standing when odometry began: at the origin,
    // facing +x, at the configured height, tilted as the IMU says. Discarding only the heading
    // is what makes this odom and not a map.
    Pose3d start;
    start.z = start_height_m_;
    start.q =
        splitGroundProjection(0.0, 0.0, 0.0, imu_orientation_, quaternionToYaw(imu_orientation_))
            .tilt;

    odom_from_lio_        = composePose(start, invertPose(lio_from_base));
    lidar_origin_latched_ = true;
    RCLCPP_INFO(
        get_logger(),
        "Latched the odom origin against the IMU attitude; %s starts at (0, 0, %.3f).",
        (pelvis_frame_id_.empty() ? base_frame_id_ : pelvis_frame_id_).c_str(),
        start_height_m_);
    return true;
}

void G1OdometryPublisher::onLidarOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    if (!lookUpLidarBodyOffset())
    {
        return;
    }

    // Normalised, not merely range-checked: everything downstream takes the conjugate as the
    // inverse and assumes a unit rotation, so a norm-2 quaternion would scale every composed
    // translation by four and reach /tf unscaled, which nothing reports.
    const std::optional<Quaternion> lidar_attitude = normalisedAttitude(msg->pose.pose.orientation);

    Pose3d lio_from_lidar_body;
    lio_from_lidar_body.x = msg->pose.pose.position.x;
    lio_from_lidar_body.y = msg->pose.pose.position.y;
    lio_from_lidar_body.z = msg->pose.pose.position.z;
    lio_from_lidar_body.q = lidar_attitude.value_or(Quaternion{});

    // A diverged scan match reports NaN rather than failing. Rejected here, before it can
    // reach the origin latch, where a single bad sample is permanent.
    if (!lidar_attitude.has_value() || !isUsablePose(lio_from_lidar_body))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "Discarding an unusable LiDAR odometry sample (non-finite position, or a "
            "quaternion too close to zero to normalise).");
        return;
    }

    const Pose3d lio_from_base = composePose(lio_from_lidar_body, lio_body_from_base_);
    if (!lidar_origin_latched_ && !latchLidarOrigin(lio_from_base))
    {
        return;
    }
    const Pose3d base_in_odom = composePose(odom_from_lio_, lio_from_base);

    // FAST-LIO's own stamp, which is the scan time rather than the time its solver finished.
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const PlanarPose   previous      = pose_;
    const bool         have_previous = have_sample_;
    const double       dt            = have_previous ? (stamp - last_sample_stamp_).seconds() : 0.0;

    pose_.x = base_in_odom.x;
    pose_.y = base_in_odom.y;
    pose_z_ = base_in_odom.z;

    applyOrientation(levelledAttitude(base_in_odom.q));

    // Differenced, because FAST-LIO publishes an empty twist and Nav2's controller reads
    // velocity from the message. Coarse by construction: two ~10 Hz poses, not an estimate.
    //
    // The dt floor matters because one duplicated-then-corrected stamp pair turns a normal 4 cm
    // step into tens of m/s, labelled to the controller as measured velocity. Below it the
    // previous twist is kept rather than replaced by a fabricated one.
    constexpr double kMinTwistDtS = 0.005;
    if (dt >= kMinTwistDtS)
    {
        world_twist_ = PlanarTwist{ (pose_.x - previous.x) / dt,
                                    (pose_.y - previous.y) / dt,
                                    wrapAngle(pose_.yaw - previous.yaw) / dt };
    }
    noteSample(stamp);
}

Quaternion G1OdometryPublisher::levelledAttitude(const Quaternion& lidar_attitude)
{
    if (!have_imu_orientation_)
    {
        return lidar_attitude;
    }

    // FAST-LIO lets its gravity state wander: measured 1.32 degrees off horizontal against the
    // ground truth's 0.00. The costmap cuts the floor at an absolute height (0.08 m), so a
    // degree of tilt lifts the floor over that cut a few metres out and the robot paints rings
    // of its own floor as obstacle. AMCL is 2D and cannot correct it downstream.
    //
    // Only the slow part comes from the IMU. Its newest sample pairs with a scan a FAST-LIO
    // period older and the pelvis swings ~9 degrees per gait cycle, so substituting the tilt
    // outright would inject more timing error than drift. Low-passing the difference removes
    // only the drifting part.
    const double     lidar_yaw = quaternionToYaw(lidar_attitude);
    const Quaternion lidar_tilt =
        splitGroundProjection(0.0, 0.0, 0.0, lidar_attitude, lidar_yaw).tilt;
    const Quaternion imu_tilt =
        splitGroundProjection(0.0, 0.0, 0.0, imu_orientation_, quaternionToYaw(imu_orientation_))
            .tilt;

    const Quaternion error = composeRotation(imu_tilt, invertRotation(lidar_tilt));
    tilt_correction_       = slerp(tilt_correction_, error, tilt_correction_gain_);
    return composeAttitude(lidar_yaw, composeRotation(tilt_correction_, lidar_tilt));
}

void G1OdometryPublisher::applyOrientation(const Quaternion& q)
{
    orientation_ = q;

    // Only the heading is held: near the vertical-axis singularity yaw swings wildly for tiny
    // attitude changes, and that noise reaches the ground projection and the body twist. The
    // attitude itself still goes out, and the first sample always latches.
    if (tiltFromVertical(q) <= max_tilt_rad_ || !have_orientation_)
    {
        pose_.yaw = quaternionToYaw(q);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            2000,
            "Tilted %.1f degrees from vertical (limit %.1f); holding the last heading. The robot "
            "is falling, not turning.",
            tiltFromVertical(q) * 180.0 / M_PI,
            max_tilt_rad_ * 180.0 / M_PI);
    }
    have_orientation_ = true;
}

void G1OdometryPublisher::onGroundTruth(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    // Exact pelvis state, sampled inside the simulator off the same MuJoCo site the robot's
    // own IMU reports from. It reaches ROS over the sensor relay's socket rather than a DDS
    // topic, which is why it does not depend on which middleware is running.
    const auto& p = msg->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    {
        // A diverged MuJoCo publishes NaN here, and tf2 drops NaN transforms silently, so
        // the symptom would be a frame that simply stops existing.
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "Discarding a non-finite position sample.");
        return;
    }

    const std::optional<Quaternion> attitude = normalisedAttitude(msg->pose.pose.orientation);
    if (!attitude)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "Ground-truth quaternion is unusable; not publishing an orientation.");
        return;
    }

    pose_.x = p.x;
    pose_.y = p.y;
    pose_z_ = p.z;
    applyOrientation(*attitude);

    // The wire twist is body-frame, per nav_msgs/Odometry; the publish path re-derives that
    // from the world twist held here, so rotate once on the way in.
    const auto&  v = msg->twist.twist.linear;
    const double c = std::cos(pose_.yaw);
    const double s = std::sin(pose_.yaw);
    world_twist_ =
        PlanarTwist{ (v.x * c) - (v.y * s), (v.x * s) + (v.y * c), msg->twist.twist.angular.z };

    noteSample(rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()));
}

void G1OdometryPublisher::onImu(const sensor_msgs::msg::Imu::SharedPtr& msg)
{
    // The pelvis IMU, from ros2_control's broadcaster rather than the robot wire, so one topic
    // serves sim and hardware. It is the gravity reference levelledAttitude() corrects against;
    // heading never comes from it, because that is what the LiDAR solution does not drift in.
    const std::optional<Quaternion> attitude = normalisedAttitude(msg->orientation);
    if (!attitude)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "IMU quaternion is unusable; not levelling against it.");
        return;
    }
    imu_orientation_      = *attitude;
    have_imu_orientation_ = true;
}

void G1OdometryPublisher::noteSample(const rclcpp::Time& stamp)
{
    // Time of the last stamp CHANGE, not of the last message: a wedged simulator can keep
    // republishing the same sample forever. Order matters, rclcpp::Time::operator!= throws
    // on mismatched clock types and last_sample_stamp_ starts out RCL_SYSTEM_TIME.
    if (!have_sample_ || stamp != last_sample_stamp_)
    {
        last_advance_wall_ = std::chrono::steady_clock::now();
    }
    last_sample_stamp_ = stamp;
    have_sample_       = true;
}

void G1OdometryPublisher::onTimer()
{
    if (!have_orientation_)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "Waiting for a usable orientation.");
        return;
    }
    if (!have_sample_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            5000,
            "No base state received yet on %s; publishing nothing.",
            source_topic_.c_str());
        return;
    }

    // Measured on the steady clock from the last stamp change, not by comparing now() against
    // the stamp: that measures the offset between two clocks. Under the MuJoCo viewer the
    // simulator runs below real time, so steady samples carry stamps seconds behind wall clock.
    const double since_advance =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - last_advance_wall_).count();
    // Not std::min: isStale treats a non-positive timeout as disabled and min(0, 2) is 0, so
    // disabling one budget would silently disable the other. Each is judged on its own.
    if (isStale(since_advance, source_timeout_s_) || isStale(since_advance, wall_timeout_s_))
    {
        // Stop publishing rather than re-stamping the last pose. A frozen transform with a
        // fresh timestamp is indistinguishable from a stationary robot, which is how a dead
        // source turns into a confidently wrong map.
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            steady_clock_,
            2000,
            "Base state stamp has not advanced for %.3f s (limits: source %.3f, wall %.3f); "
            "stopped publishing %s -> %s.",
            since_advance,
            source_timeout_s_,
            wall_timeout_s_,
            odom_frame_id_.c_str(),
            base_frame_id_.c_str());
        return;
    }

    const Quaternion   orientation = orientation_;
    const PlanarTwist  body_twist  = toBodyTwist(world_twist_, pose_.yaw);
    const rclcpp::Time stamp       = last_sample_stamp_;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id  = base_frame_id_;

    if (pelvis_frame_id_.empty())
    {
        tf.transform.translation.x = pose_.x;
        tf.transform.translation.y = pose_.y;
        tf.transform.translation.z = pose_z_;
        tf.transform.rotation.x    = orientation.x;
        tf.transform.rotation.y    = orientation.y;
        tf.transform.rotation.z    = orientation.z;
        tf.transform.rotation.w    = orientation.w;
        tf_broadcaster_->sendTransform(tf);
    }
    else
    {
        const GroundSplit split =
            splitGroundProjection(pose_.x, pose_.y, pose_z_, orientation, pose_.yaw);
        const Quaternion heading = yawToQuaternion(split.footprint.yaw);

        tf.transform.translation.x = split.footprint.x;
        tf.transform.translation.y = split.footprint.y;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x    = heading.x;
        tf.transform.rotation.y    = heading.y;
        tf.transform.rotation.z    = heading.z;
        tf.transform.rotation.w    = heading.w;

        geometry_msgs::msg::TransformStamped body_tf;
        body_tf.header.stamp            = stamp;
        body_tf.header.frame_id         = base_frame_id_;
        body_tf.child_frame_id          = pelvis_frame_id_;
        body_tf.transform.translation.z = split.child_z;
        body_tf.transform.rotation.x    = split.tilt.x;
        body_tf.transform.rotation.y    = split.tilt.y;
        body_tf.transform.rotation.z    = split.tilt.z;
        body_tf.transform.rotation.w    = split.tilt.w;

        // One call: both edges share a stamp, and no consumer should see the chain half-updated.
        tf_broadcaster_->sendTransform({ tf, body_tf });
    }

    if (!odom_pub_ || !odom_pub_->is_activated())
    {
        return;
    }

    // Nav2's costmap and controller server read velocity from Odometry, not from TF.
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id  = base_frame_id_;
    // Taken from the transform published just above rather than rebuilt from pose_, so the two
    // cannot disagree: with a split chain that means the footprint, not the body. Dropping z and
    // the tilt is correct, since child_frame_id names the footprint and toBodyTwist is yaw-only.
    odom.pose.pose.position.x  = tf.transform.translation.x;
    odom.pose.pose.position.y  = tf.transform.translation.y;
    odom.pose.pose.position.z  = tf.transform.translation.z;
    odom.pose.pose.orientation = tf.transform.rotation;
    odom.twist.twist.linear.x  = body_twist.vx;
    odom.twist.twist.linear.y  = body_twist.vy;
    odom.twist.twist.angular.z = body_twist.omega;
    std::copy(pose_covariance_.begin(), pose_covariance_.end(), odom.pose.covariance.begin());
    std::copy(twist_covariance_.begin(), twist_covariance_.end(), odom.twist.covariance.begin());
    odom_pub_->publish(odom);
}

}  // namespace g1_state_estimation
