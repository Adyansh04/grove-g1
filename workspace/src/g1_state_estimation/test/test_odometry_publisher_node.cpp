/**
 * @file test_odometry_publisher_node.cpp
 * @brief In-process lifecycle tests for the odom -> base publisher, both sources.
 *
 * Runs on an isolated ROS_DOMAIN_ID so a sim or another test on the machine cannot feed it
 * real data.
 */

#include <gmock/gmock.h>

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "g1_state_estimation/g1_odometry_publisher_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

using g1_state_estimation::G1OdometryPublisher;
using namespace std::chrono_literals;

namespace
{

rclcpp::NodeOptions optionsWithSource(const std::string& source, bool use_sim_time = false)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("odometry_source", source),
        rclcpp::Parameter("publish_rate_hz", 100.0),
        rclcpp::Parameter("source_timeout_ms", 200.0),
        rclcpp::Parameter("wall_timeout_ms", 300.0),
        rclcpp::Parameter("start_height_m", 0.793),
        // One edge carrying the whole pose, so these suites assert against the pose the source
        // reports rather than against its ground projection. The split chain has its own
        // suites further down.
        rclcpp::Parameter("base_frame_id", "base_link"),
        rclcpp::Parameter("use_sim_time", use_sim_time),
    });
    return options;
}

/// Spins the node (and any helpers) for a wall duration.
void spinFor(
    const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr>& nodes,
    std::chrono::milliseconds                                                 duration)
{
    rclcpp::executors::SingleThreadedExecutor executor;
    for (const auto& node : nodes)
    {
        executor.add_node(node);
    }
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok())
    {
        executor.spin_some(10ms);
    }
}

/// What FAST-LIO publishes: the body pose in its own start frame, twist left empty.
nav_msgs::msg::Odometry
makeLidarOdometry(const rclcpp::Time& stamp, double x, double y, double z, double yaw)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp            = stamp;
    msg.header.frame_id         = "camera_init";
    msg.child_frame_id          = "body";
    msg.pose.pose.position.x    = x;
    msg.pose.pose.position.y    = y;
    msg.pose.pose.position.z    = z;
    msg.pose.pose.orientation.z = std::sin(yaw * 0.5);
    msg.pose.pose.orientation.w = std::cos(yaw * 0.5);
    return msg;
}

/// The simulator's ground-truth track: split chain, one source topic.
///
/// Frames and source mirror config/g1_odometry_publisher_converged.yaml; the rates and timeouts
/// are faster than the shipped ones so these suites do not wait on real budgets.
rclcpp::NodeOptions optionsForGroundTruth(double max_tilt_deg = 80.0)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("odometry_source", "ground_truth"),
        rclcpp::Parameter("publish_rate_hz", 100.0),
        rclcpp::Parameter("source_timeout_ms", 500.0),
        rclcpp::Parameter("wall_timeout_ms", 1000.0),
        rclcpp::Parameter("base_frame_id", "base_footprint"),
        rclcpp::Parameter("pelvis_frame_id", "pelvis"),
        rclcpp::Parameter("max_tilt_deg", max_tilt_deg),
        rclcpp::Parameter("use_sim_time", false),
    });
    return options;
}

geometry_msgs::msg::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = (cr * cp * cy) + (sr * sp * sy);
    q.x = (sr * cp * cy) - (cr * sp * sy);
    q.y = (cr * sp * cy) + (sr * cp * sy);
    q.z = (cr * cp * sy) - (sr * sp * cy);
    return q;
}

/// What the sensor relay sends: pose and twist in one message, twist in the body frame.
nav_msgs::msg::Odometry makeGroundTruth(
    const rclcpp::Time& stamp, double x, double y, double z, double roll, double pitch, double yaw)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp          = stamp;
    msg.pose.pose.position.x  = x;
    msg.pose.pose.position.y  = y;
    msg.pose.pose.position.z  = z;
    msg.pose.pose.orientation = quaternionFromRpy(roll, pitch, yaw);
    return msg;
}

sensor_msgs::msg::Imu makeImu(double roll, double pitch, double yaw)
{
    sensor_msgs::msg::Imu msg;
    msg.orientation = quaternionFromRpy(roll, pitch, yaw);
    return msg;
}

double yawOf(const geometry_msgs::msg::Quaternion& q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double tiltOf(const geometry_msgs::msg::Quaternion& q)
{
    return std::acos(std::max(-1.0, std::min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))));
}

/// Drives a ground-truth node with one attitude and collects what reaches /tf and ~/odom.
class GroundTruthHarness
{
public:
    explicit GroundTruthHarness(std::shared_ptr<G1OdometryPublisher> node, const std::string& name)
      : node_(std::move(node))
      , helper_(std::make_shared<rclcpp::Node>(name))
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        truth_pub_     = helper_->create_publisher<nav_msgs::msg::Odometry>(
            "/g1_odometry_publisher/base_state",
            qos);
        tf_sub_ = helper_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            rclcpp::QoS(200),
            [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg) {
                batches.push_back(msg->transforms);
            });
        odom_sub_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
            "/g1_odometry_publisher/odom",
            rclcpp::QoS(200),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odoms.push_back(*msg); });
        nodes_ = { node_->get_node_base_interface(), helper_->get_node_base_interface() };
        spinFor(nodes_, 200ms);
    }

    /// Publishes the pose `count` times, then settles so the 100 Hz timer has ticked on the
    /// last sample even when count is 1.
    void feed(double x, double y, double z, double roll, double pitch, double yaw, int count = 15)
    {
        for (int i = 0; i < count; ++i)
        {
            truth_pub_->publish(makeGroundTruth(helper_->now(), x, y, z, roll, pitch, yaw));
            spinFor(nodes_, 20ms);
        }
        spinFor(nodes_, 100ms);
    }

    /// The most recent transform with this parent/child, or nullopt.
    std::optional<geometry_msgs::msg::TransformStamped>
    latest(const std::string& parent, const std::string& child) const
    {
        // std::ranges::reverse_view breaks clang-tidy's Clang-14 parser against libstdc++ here.
        // NOLINTNEXTLINE(modernize-loop-convert)
        for (auto batch = batches.rbegin(); batch != batches.rend(); ++batch)
        {
            for (const auto& tf : *batch)
            {
                if (tf.header.frame_id == parent && tf.child_frame_id == child)
                {
                    return tf;
                }
            }
        }
        return std::nullopt;
    }

    std::vector<std::vector<geometry_msgs::msg::TransformStamped>> batches;
    std::vector<nav_msgs::msg::Odometry>                           odoms;

private:
    std::shared_ptr<G1OdometryPublisher>                               node_;
    std::shared_ptr<rclcpp::Node>                                      helper_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              truth_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr          tf_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
    std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes_;
};

/// Counts log lines containing a substring, for the throttled tilt warning. The rcutils
/// handler is process-global, so this restores whatever was installed on destruction.
class LogCapture
{
public:
    explicit LogCapture(std::string needle)
    {
        // Nesting would capture this class's own handler as previous_ and recurse forever.
        assert(instance_ == nullptr);
        needle_   = std::move(needle);
        instance_ = this;
        previous_ = rcutils_logging_get_output_handler();
        rcutils_logging_set_output_handler(&LogCapture::handler);
    }

    ~LogCapture()
    {
        rcutils_logging_set_output_handler(previous_);
        instance_ = nullptr;
    }

    LogCapture(const LogCapture&)            = delete;
    LogCapture& operator=(const LogCapture&) = delete;

    int count() const { return count_; }

private:
    static void handler(
        const rcutils_log_location_t* location, int severity, const char* name,
        rcutils_time_point_value_t timestamp, const char* format, va_list* args)
    {
        if (instance_ != nullptr)
        {
            // va_copy is required here: previous_ below still needs an unconsumed *args.
            va_list copy;
            // NOLINTNEXTLINE(clang-analyzer-valist.Uninitialized)
            va_copy(copy, *args);
            std::array<char, 1024> buffer;
            // Truncation is fine here: this only has to be long enough to find the needle in.
            (void)vsnprintf(buffer.data(), buffer.size(), format, copy);
            va_end(copy);
            if (std::string(buffer.data()).find(instance_->needle_) != std::string::npos)
            {
                ++instance_->count_;
            }
        }
        if (instance_ != nullptr && instance_->previous_ != nullptr)
        {
            instance_->previous_(location, severity, name, timestamp, format, args);
        }
    }

    std::string                      needle_;
    int                              count_    = 0;
    rcutils_logging_output_handler_t previous_ = nullptr;
    // Mutable and static because rcutils_logging_set_output_handler takes a bare function
    // pointer, so this is the only route from the handler back to the instance.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static LogCapture* instance_;
};

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
LogCapture* LogCapture::instance_ = nullptr;

/// Drives a fast_lio node with a LiDAR pose and an attitude, and collects what comes out.
class FastLioHarness
{
public:
    explicit FastLioHarness(std::shared_ptr<G1OdometryPublisher> node, const std::string& name)
      : node_(std::move(node))
      , helper_(std::make_shared<rclcpp::Node>(name))
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        lio_pub_       = helper_->create_publisher<nav_msgs::msg::Odometry>(
            "/g1_odometry_publisher/lidar_odometry",
            qos);
        imu_pub_ =
            helper_->create_publisher<sensor_msgs::msg::Imu>("/g1_odometry_publisher/imu", qos);
        tf_sub_ = helper_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            rclcpp::QoS(200),
            [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg) {
                transforms.insert(transforms.end(), msg->transforms.begin(), msg->transforms.end());
            });
        odom_sub_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
            "/g1_odometry_publisher/odom",
            rclcpp::QoS(200),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { odoms.push_back(*msg); });
        nodes_ = { node_->get_node_base_interface(), helper_->get_node_base_interface() };
        spinFor(nodes_, 200ms);
    }

    /// One LiDAR sample under a level attitude, stamped now. The attitude lands first, the
    /// ordering the robot gives for free: the IMU broadcaster runs far faster than 10 Hz scans.
    void feed(double x, double y, double z, double yaw, bool with_imu = true)
    {
        if (with_imu)
        {
            imu_pub_->publish(makeImu(0.0, 0.0, 0.0));
            spinFor(nodes_, 10ms);
        }
        lio_pub_->publish(makeLidarOdometry(helper_->now(), x, y, z, yaw));
        spinFor(nodes_, 40ms);
    }

    void spin(std::chrono::milliseconds duration) { spinFor(nodes_, duration); }

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    std::vector<nav_msgs::msg::Odometry>              odoms;

private:
    std::shared_ptr<G1OdometryPublisher>                               node_;
    std::shared_ptr<rclcpp::Node>                                      helper_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              lio_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                imu_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr          tf_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
    std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes_;
};

}  // namespace

TEST(OdometryPublisherHardwareBranch, ConfigureFailsAndCreatesNothing)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("hardware"));

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "hardware must not configure: the real G1 publishes no odometry";

    // The point of failing in on_configure rather than at first tick. Advertising /tf and
    // then never publishing is the silent mode this node exists to rule out.
    EXPECT_EQ(node->count_publishers("/tf"), 0U) << "a /tf publisher was created anyway";
    EXPECT_EQ(node->count_publishers("/g1_odometry_publisher/odom"), 0U)
        << "an odom publisher was created anyway";
}

TEST(OdometryPublisherHardwareBranch, IsTheDefaultSource)
{
    // A misconfigured hardware bring-up must never silently emit fabricated odometry, so
    // the safe branch is the one you get by saying nothing.
    auto node = std::make_shared<G1OdometryPublisher>(rclcpp::NodeOptions());
    EXPECT_EQ(node->get_parameter("odometry_source").as_string(), "hardware");
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(OdometryPublisherHardwareBranch, UnknownSourceAlsoFailsToConfigure)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("sim_ground_truth"));
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "a typo must not fall back to a working source";
    EXPECT_EQ(node->count_publishers("/tf"), 0U);
}

// --- fast_lio: the latch, and the odometry built on top of it -------------------------------

TEST(OdometryPublisherFastLio, ConfiguresAndActivates)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("fast_lio"));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST(OdometryPublisherFastLio, PublishesNothingUntilTheImuHasLevelledTheOrigin)
{
    // FAST-LIO's start frame is wherever its IMU happened to be pointing, so without an
    // attitude to level against there is no way to know which way is up. Publishing anyway
    // would tilt odom by the robot's initial lean, permanently.
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("fast_lio"));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    FastLioHarness harness(node, "fastlio_no_imu_helper");
    for (int i = 0; i < 5; ++i)
    {
        harness.feed(1.0, 2.0, 0.0, 0.3, /*with_imu=*/false);
    }
    EXPECT_TRUE(harness.transforms.empty()) << "published without ever seeing an attitude";

    harness.feed(1.0, 2.0, 0.0, 0.3);
    harness.spin(100ms);
    EXPECT_FALSE(harness.transforms.empty()) << "still silent once the attitude arrived";
}

TEST(OdometryPublisherFastLio, LatchesTheOriginThenReportsMotionRelativeToIt)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("fast_lio"));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    FastLioHarness harness(node, "fastlio_latch_helper");

    // A deliberately non-trivial start pose. In practice FAST-LIO's first sample is the
    // identity, since its start frame is the body frame at init, which would let a broken
    // composition pass by doing nothing at all.
    const double start_x   = 0.4;
    const double start_y   = -0.2;
    const double start_z   = 0.05;
    const double start_yaw = 0.6;
    harness.feed(start_x, start_y, start_z, start_yaw);
    harness.spin(100ms);

    ASSERT_FALSE(harness.transforms.empty()) << "nothing published on /tf";
    const auto& latched = harness.transforms.back();
    EXPECT_EQ(latched.header.frame_id, "odom");
    EXPECT_EQ(latched.child_frame_id, "base_link");
    EXPECT_NEAR(latched.transform.translation.x, 0.0, 1e-6) << "the origin must land at zero";
    EXPECT_NEAR(latched.transform.translation.y, 0.0, 1e-6);
    EXPECT_NEAR(latched.transform.translation.z, 0.793, 1e-6)
        << "start_height_m is what makes odom the ground plane; without it a floor return "
           "transformed into odom lands below the floor";
    EXPECT_NEAR(yawOf(latched.transform.rotation), 0.0, 1e-6) << "and facing +x";

    // Now drive one metre along the heading the robot started with. Whatever frame FAST-LIO
    // reports in, odom has to call that one metre straight ahead.
    harness.feed(start_x + std::cos(start_yaw), start_y + std::sin(start_yaw), start_z, start_yaw);
    harness.spin(100ms);

    const auto& moved = harness.transforms.back();
    EXPECT_NEAR(moved.transform.translation.x, 1.0, 1e-5);
    EXPECT_NEAR(moved.transform.translation.y, 0.0, 1e-5);
    EXPECT_NEAR(yawOf(moved.transform.rotation), 0.0, 1e-5) << "driving straight is not turning";

    // And a pure rotation in the LiDAR frame is a pure rotation in odom.
    harness.feed(
        start_x + std::cos(start_yaw),
        start_y + std::sin(start_yaw),
        start_z,
        start_yaw + 0.5);
    harness.spin(100ms);
    EXPECT_NEAR(yawOf(harness.transforms.back().transform.rotation), 0.5, 1e-5);
}

TEST(OdometryPublisherFastLio, DifferencesTheTwistIntoTheBodyFrame)
{
    // FAST-LIO leaves twist empty, and Nav2's controller server reads velocity from the
    // message rather than from TF, so this node has to supply it.
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("fast_lio"));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    FastLioHarness harness(node, "fastlio_twist_helper");
    // Latch facing +x, then turn to face +y and walk that way. The world velocity is +y and
    // the heading is +y, so in the body frame it has to read as forward.
    harness.feed(0.0, 0.0, 0.0, 0.0);
    harness.spin(50ms);
    for (int i = 1; i <= 6; ++i)
    {
        harness.feed(0.0, 0.1 * i, 0.0, M_PI_2);
        harness.spin(50ms);
    }

    ASSERT_FALSE(harness.odoms.empty()) << "nothing published on ~/odom";
    const auto& odom = harness.odoms.back();
    EXPECT_EQ(odom.child_frame_id, "base_link");
    EXPECT_GT(odom.twist.twist.linear.x, 0.05)
        << "twist must be in base_link, not odom: heading +y and moving +y is forward";
    EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, 0.05);
    EXPECT_GT(odom.pose.covariance[0], 0.0) << "all-zero covariance is a known Nav2 footgun";
    EXPECT_GT(odom.twist.covariance[0], 0.0);
}

TEST(OdometryPublisherFastLio, StopsPublishingWhenTheSourceGoesSilent)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("fast_lio"));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    FastLioHarness harness(node, "fastlio_stale_helper");
    for (int i = 0; i < 8; ++i)
    {
        harness.feed(0.01 * i, 0.0, 0.0, 0.0);
    }
    ASSERT_FALSE(harness.transforms.empty()) << "never published while the source was fresh";

    // Go quiet for well past source_timeout_ms, then check it actually stopped rather than
    // re-stamping the last pose forever.
    harness.spin(400ms);
    const std::size_t after_timeout = harness.transforms.size();
    harness.spin(300ms);
    EXPECT_EQ(harness.transforms.size(), after_timeout)
        << "kept publishing " << (harness.transforms.size() - after_timeout)
        << " transforms from a source that had gone silent";
}

TEST(OdometryPublisherSimTime, StopsPublishingWhenSimTimeItselfFreezes)
{
    // The failure the wall-clock test cannot see. On the simulator track /clock comes from the
    // SAME process as the sensor data, so when that process wedges, sim time stops with it:
    // `now() - last_sample_stamp_` stays pinned near zero and a sim-time-only staleness check
    // never fires, leaving a frozen pose broadcast as if it were live.
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("fast_lio", true));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto helper = std::make_shared<rclcpp::Node>("odom_test_helper_simtime");
    auto clock_pub =
        helper->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    const auto qos     = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto       lio_pub = helper->create_publisher<nav_msgs::msg::Odometry>(
        "/g1_odometry_publisher/lidar_odometry",
        qos);
    auto imu_pub =
        helper->create_publisher<sensor_msgs::msg::Imu>("/g1_odometry_publisher/imu", qos);

    std::size_t transform_count = 0;
    auto        tf_sub          = helper->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS(100),
        [&transform_count](const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg) {
            transform_count += msg->transforms.size();
        });

    const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes = {
        node->get_node_base_interface(),
        helper->get_node_base_interface()
    };

    // Drive sim time forward by hand and feed samples stamped with it.
    rclcpp::Time sim_now(0, 0, RCL_ROS_TIME);
    const auto   tick = rclcpp::Duration::from_seconds(0.02);
    for (int i = 0; i < 25; ++i)
    {
        sim_now = sim_now + tick;
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = sim_now;
        clock_pub->publish(clock_msg);
        imu_pub->publish(makeImu(0.0, 0.0, 0.0));
        lio_pub->publish(makeLidarOdometry(sim_now, 0.01 * i, 0.0, 0.0, 0.0));
        spinFor(nodes, 20ms);
    }
    ASSERT_GT(transform_count, 0U) << "never published while sim time was advancing";

    // Now the simulator wedges: /clock stops AND the stamp stops advancing, but samples
    // keep arriving, so the node still has fresh-looking data on a frozen clock. Wall time
    // is the only thing left that can notice.
    const std::size_t before_freeze = transform_count;
    for (int i = 0; i < 15; ++i)
    {
        imu_pub->publish(makeImu(0.0, 0.0, 0.0));
        lio_pub->publish(makeLidarOdometry(sim_now, 0.25, 0.0, 0.0, 0.0));
        spinFor(nodes, 40ms);
    }
    const std::size_t after_timeout = transform_count;
    for (int i = 0; i < 10; ++i)
    {
        imu_pub->publish(makeImu(0.0, 0.0, 0.0));
        lio_pub->publish(makeLidarOdometry(sim_now, 0.25, 0.0, 0.0, 0.0));
        spinFor(nodes, 40ms);
    }

    EXPECT_GT(after_timeout, before_freeze)
        << "sanity: the node should keep publishing for at least the timeout after the freeze";
    EXPECT_EQ(transform_count, after_timeout)
        << "published " << (transform_count - after_timeout)
        << " more transforms after sim time froze. With /clock stopped, elapsed sim time "
           "stays at zero forever, so only a wall-clock budget can catch this.";
}

// --- Ground truth: the split chain and the tilt guard ---------------------------------------

TEST(OdometryPublisherGroundTruth, PublishesTheSplitChainWithOneStamp)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsForGroundTruth());
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    GroundTruthHarness harness(node, "ground_truth_split_helper");
    // A walking attitude: a few degrees of roll and pitch under a real heading.
    const double roll   = -0.05;
    const double pitch  = 0.0847;
    const double yaw    = 1.2;
    const double height = 0.758;
    harness.feed(3.0, -4.0, height, roll, pitch, yaw);

    const auto foot = harness.latest("odom", "base_footprint");
    const auto body = harness.latest("base_footprint", "pelvis");
    ASSERT_TRUE(foot.has_value()) << "no odom -> base_footprint on /tf";
    ASSERT_TRUE(body.has_value()) << "no base_footprint -> pelvis on /tf";

    // The footprint carries position and heading only.
    EXPECT_NEAR(foot->transform.translation.x, 3.0, 1e-5);
    EXPECT_NEAR(foot->transform.translation.y, -4.0, 1e-5);
    EXPECT_NEAR(foot->transform.translation.z, 0.0, 1e-12) << "the footprint is on the floor";
    EXPECT_NEAR(tiltOf(foot->transform.rotation), 0.0, 1e-12) << "and gravity-aligned";
    EXPECT_NEAR(yawOf(foot->transform.rotation), yaw, 1e-5);

    // The body edge carries the height and the tilt the footprint dropped, and nothing else.
    EXPECT_NEAR(body->transform.translation.x, 0.0, 1e-12);
    EXPECT_NEAR(body->transform.translation.y, 0.0, 1e-12);
    EXPECT_NEAR(body->transform.translation.z, height, 1e-5);
    EXPECT_NEAR(
        tiltOf(body->transform.rotation),
        tiltOf(foot->transform.rotation) +
            std::acos(std::max(-1.0, std::min(1.0, std::cos(roll) * std::cos(pitch)))),
        1e-4)
        << "the residual holds the whole tilt";
    EXPECT_NEAR(yawOf(body->transform.rotation), 0.0, 1e-4) << "and none of the heading";

    // Both edges must go out together: a consumer that sees the chain half-updated composes a
    // fresh footprint with a stale body, which is a pose that never existed.
    bool found_pair = false;
    for (const auto& batch : harness.batches)
    {
        if (batch.size() == 2)
        {
            EXPECT_EQ(batch[0].header.stamp, batch[1].header.stamp);
            found_pair = true;
        }
    }
    EXPECT_TRUE(found_pair) << "the two edges were never published in one message";
}

TEST(OdometryPublisherGroundTruth, OdometryDescribesTheFootprintNotTheBody)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsForGroundTruth());
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    GroundTruthHarness harness(node, "ground_truth_odom_helper");
    harness.feed(1.0, 2.0, 0.75, 0.0, 0.1, 0.3);

    ASSERT_FALSE(harness.odoms.empty()) << "nothing published on ~/odom";
    const auto& odom = harness.odoms.back();
    // Nav2 reads child_frame_id and believes it. Publishing the pelvis pose under a
    // base_footprint label would put the robot 0.75 m into the air on every costmap.
    EXPECT_EQ(odom.child_frame_id, "base_footprint");
    EXPECT_NEAR(odom.pose.pose.position.z, 0.0, 1e-12);
    EXPECT_NEAR(tiltOf(odom.pose.pose.orientation), 0.0, 1e-12);

    const auto foot = harness.latest("odom", "base_footprint");
    ASSERT_TRUE(foot.has_value());
    EXPECT_NEAR(odom.pose.pose.position.x, foot->transform.translation.x, 1e-12)
        << "the message and the transform must not be able to disagree";
    EXPECT_NEAR(odom.pose.pose.position.y, foot->transform.translation.y, 1e-12);
}

TEST(OdometryPublisherGroundTruth, TiltGuardHoldsTheLastGoodHeadingAndWarnsOnce)
{
    // 10 degrees, so the "past the threshold" case is an ordinary attitude rather than a
    // near-singular one: this tests the guard, not the arithmetic at 90 degrees.
    auto node = std::make_shared<G1OdometryPublisher>(optionsForGroundTruth(10.0));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    GroundTruthHarness harness(node, "ground_truth_tilt_helper");

    // Upright and well inside the limit: the heading tracks.
    const double good_yaw = 0.4;
    harness.feed(0.0, 0.0, 0.75, 0.0, 0.05, good_yaw);
    const auto upright = harness.latest("odom", "base_footprint");
    ASSERT_TRUE(upright.has_value());
    ASSERT_NEAR(yawOf(upright->transform.rotation), good_yaw, 1e-4)
        << "below the limit the heading must follow the IMU";

    // Now past it, with a different heading. The heading must not follow.
    const std::size_t before = harness.batches.size();
    int               warns  = 0;
    {
        LogCapture capture("holding the last heading");
        harness.feed(0.0, 0.0, 0.75, 0.0, 0.6, good_yaw + 1.0);
        warns = capture.count();
    }

    const auto tilted = harness.latest("odom", "base_footprint");
    ASSERT_TRUE(tilted.has_value());
    ASSERT_GT(harness.batches.size(), before) << "publishing stopped instead of holding";
    EXPECT_NEAR(yawOf(tilted->transform.rotation), good_yaw, 1e-4)
        << "past the limit the last well-conditioned heading is held";

    // The attitude itself keeps going out, because a fallen robot really is tilted and hiding it
    // would be its own lie.
    const auto body = harness.latest("base_footprint", "pelvis");
    ASSERT_TRUE(body.has_value());
    EXPECT_GT(tiltOf(body->transform.rotation), 0.5) << "the real tilt must still reach TF";

    // Throttled at 2 s, so 15 samples inside that window give one line rather than fifteen.
    //
    // RCLCPP_WARN_THROTTLE keeps its timestamp in a static local at the call site, not per
    // node, so every test in this binary shares one window. Only one other test tilts the
    // robot and it stops at one sample; keep it that way or this assertion goes order-dependent.
    EXPECT_EQ(warns, 1) << "expected exactly one throttled warning, got " << warns;
}

TEST(OdometryPublisherGroundTruth, TiltGuardLatchesTheFirstSampleEvenMidFall)
{
    // The documented spawn-topple case: if the very first attitude is already past the limit
    // there is nothing to hold instead, so it latches rather than publishing a default zero
    // heading that no sensor ever reported.
    auto node = std::make_shared<G1OdometryPublisher>(optionsForGroundTruth(10.0));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    GroundTruthHarness harness(node, "ground_truth_first_sample_helper");
    const double       yaw = -0.9;
    // Exactly one sample. A second would take the hold branch and trip the warning throttle
    // that TiltGuardHoldsTheLastGoodHeadingAndWarnsOnce counts; see the note there.
    harness.feed(0.0, 0.0, 0.5, 0.0, 0.7, yaw, /*count=*/1);

    const auto foot = harness.latest("odom", "base_footprint");
    ASSERT_TRUE(foot.has_value()) << "nothing published at all";
    EXPECT_NEAR(yawOf(foot->transform.rotation), yaw, 1e-3);
}

TEST(OdometryPublisherGroundTruth, RejectsAnOutOfRangeMaxTilt)
{
    for (double degrees : { 0.0, -5.0, 180.0, 400.0 })
    {
        auto node = std::make_shared<G1OdometryPublisher>(optionsForGroundTruth(degrees));
        EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
            << "max_tilt_deg " << degrees << " should not configure";
    }
}

TEST(OdometryPublisherGroundTruth, RejectsAFrameChainThatCannotExist)
{
    auto same = optionsForGroundTruth();
    same.append_parameter_override("pelvis_frame_id", "base_footprint");
    EXPECT_EQ(
        std::make_shared<G1OdometryPublisher>(same)->configure().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "a self-loop edge would be rejected by tf2, with an error pointing at tf2";

    auto empty = optionsForGroundTruth();
    empty.append_parameter_override("base_frame_id", std::string(""));
    EXPECT_EQ(
        std::make_shared<G1OdometryPublisher>(empty)->configure().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char** argv)
{
    // Isolated domain: a running sim on the default domain must not be able to feed this.
    // Before any node or thread exists, so the thread-safety this warns about does not apply.
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    setenv("ROS_DOMAIN_ID", "77", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
