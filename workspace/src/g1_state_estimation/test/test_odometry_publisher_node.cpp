/**
 * @file test_odometry_publisher_node.cpp
 * @brief In-process lifecycle tests for the odom -> base publisher, both sources.
 *
 * Runs on an isolated ROS_DOMAIN_ID so a sim or another test on the machine cannot feed it
 * real data, same pattern as g1_locomotion's test_loco_bridge_node.
 */

#include <gmock/gmock.h>

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
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

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
        rclcpp::Parameter("base_height_m", 0.793),
        // The planar sandbox's own configuration, not the node default. Its base cannot tilt,
        // so it wants one edge carrying the whole pose; pelvis_frame_id stays empty and the
        // frame keeps the name g1_sim's launch and RViz config already use.
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

sensor_msgs::msg::JointState makeBaseState(
    const rclcpp::Time& stamp, double x, double y, double yaw, double vx = 0.0, double vy = 0.0,
    double omega = 0.0)
{
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    // Deliberately not in x, y, yaw order: the node must look joints up by name.
    msg.name     = { "base_yaw_joint", "base_x_joint", "base_y_joint" };
    msg.position = { yaw, x, y };
    msg.velocity = { omega, vx, vy };
    return msg;
}

/// The converged unitree_mujoco track: split chain, both source topics.
///
/// The frames and the source mirror config/g1_odometry_publisher_converged.yaml deliberately;
/// that the shipped file really loads is covered end to end by g1_navigation's
/// test_scan_pipeline, which asserts the same chain against a live sim. The rates and timeouts
/// are faster than the shipped ones on purpose, so these suites do not wait on real budgets.
rclcpp::NodeOptions optionsForConverged(double max_tilt_deg = 80.0)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("odometry_source", "sim_sportmodestate"),
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

unitree_go::msg::SportModeState makeSportState(double x, double y, double z)
{
    unitree_go::msg::SportModeState msg;
    msg.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    msg.velocity = { 0.0F, 0.0F, 0.0F };
    return msg;
}

/// LowState carrying only the attitude. quaternion is w-first, matching Unitree's wire order.
unitree_hg::msg::LowState makeLowState(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    unitree_hg::msg::LowState msg;
    msg.imu_state.quaternion[0] = static_cast<float>(cr * cp * cy + sr * sp * sy);
    msg.imu_state.quaternion[1] = static_cast<float>(sr * cp * cy - cr * sp * sy);
    msg.imu_state.quaternion[2] = static_cast<float>(cr * sp * cy + sr * cp * sy);
    msg.imu_state.quaternion[3] = static_cast<float>(cr * cp * sy - sr * sp * cy);
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

/// Drives a converged-source node with one attitude and collects what reaches /tf and ~/odom.
class ConvergedHarness
{
public:
    explicit ConvergedHarness(std::shared_ptr<G1OdometryPublisher> node, const std::string& name)
      : node_(std::move(node))
      , helper_(std::make_shared<rclcpp::Node>(name))
    {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        sport_pub_     = helper_->create_publisher<unitree_go::msg::SportModeState>(
            "/g1_odometry_publisher/sport_state",
            qos);
        low_pub_ = helper_->create_publisher<unitree_hg::msg::LowState>(
            "/g1_odometry_publisher/imu_state",
            qos);
        tf_sub_ = helper_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf",
            rclcpp::QoS(200),
            [this](tf2_msgs::msg::TFMessage::SharedPtr msg) { batches.push_back(msg->transforms); });
        odom_sub_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
            "/g1_odometry_publisher/odom",
            rclcpp::QoS(200),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odoms.push_back(*msg); });
        nodes_ = { node_->get_node_base_interface(), helper_->get_node_base_interface() };
        spinFor(nodes_, 200ms);
    }

    /// Publishes the pose `count` times, then settles so the 100 Hz timer has ticked on the
    /// last sample even when count is 1.
    void feed(double x, double y, double z, double roll, double pitch, double yaw, int count = 15)
    {
        for (int i = 0; i < count; ++i)
        {
            sport_pub_->publish(makeSportState(x, y, z));
            low_pub_->publish(makeLowState(roll, pitch, yaw));
            spinFor(nodes_, 20ms);
        }
        spinFor(nodes_, 100ms);
    }

    /// The most recent transform with this parent/child, or nullopt.
    std::optional<geometry_msgs::msg::TransformStamped>
    latest(const std::string& parent, const std::string& child) const
    {
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
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr      sport_pub_;
    rclcpp::Publisher<unitree_hg::msg::LowState>::SharedPtr            low_pub_;
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
            va_list copy;
            va_copy(copy, *args);
            char buffer[1024];
            vsnprintf(buffer, sizeof(buffer), format, copy);
            va_end(copy);
            if (std::string(buffer).find(instance_->needle_) != std::string::npos)
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
    static LogCapture*               instance_;
};

LogCapture* LogCapture::instance_ = nullptr;

class OdometryPublisherTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<G1OdometryPublisher>(optionsWithSource(source_));
    }

    void TearDown() override { node_.reset(); }

    std::string                          source_ = "sim_ground_truth";
    std::shared_ptr<G1OdometryPublisher> node_;
};

}  // namespace

TEST(OdometryPublisherHardwareBranch, ConfigureFailsAndCreatesNothing)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("hardware"));

    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "hardware must not configure: the real G1 publishes no odometry";

    // The point of failing in on_configure rather than at first tick. Advertising /tf and
    // then never publishing is the silent mode this node exists to rule out.
    EXPECT_EQ(node->count_publishers("/tf"), 0u) << "a /tf publisher was created anyway";
    EXPECT_EQ(node->count_publishers("/g1_odometry_publisher/odom"), 0u)
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
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("ground_truth"));
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "a typo must not fall back to a working source";
    EXPECT_EQ(node->count_publishers("/tf"), 0u);
}

TEST_F(OdometryPublisherTest, SimGroundTruthConfiguresAndActivates)
{
    ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(OdometryPublisherTest, PublishesTheSampledPoseOnTfAndOdom)
{
    ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto helper    = std::make_shared<rclcpp::Node>("odom_test_helper");
    auto state_pub = helper->create_publisher<sensor_msgs::msg::JointState>(
        "/g1_odometry_publisher/base_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    auto tf_sub = helper->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS(100),
        [&transforms](tf2_msgs::msg::TFMessage::SharedPtr msg) {
            transforms.insert(transforms.end(), msg->transforms.begin(), msg->transforms.end());
        });

    std::vector<nav_msgs::msg::Odometry> odoms;
    auto odom_sub = helper->create_subscription<nav_msgs::msg::Odometry>(
        "/g1_odometry_publisher/odom",
        rclcpp::QoS(100),
        [&odoms](nav_msgs::msg::Odometry::SharedPtr msg) { odoms.push_back(*msg); });

    const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes = {
        node_->get_node_base_interface(),
        helper->get_node_base_interface()
    };
    spinFor(nodes, 300ms);

    // Facing +y, driving world +x: the body twist must come out as -y.
    const double yaw = M_PI_2;
    for (int i = 0; i < 20; ++i)
    {
        state_pub->publish(makeBaseState(helper->now(), 1.5, -2.5, yaw, 1.0, 0.0, 0.25));
        spinFor(nodes, 20ms);
    }

    ASSERT_FALSE(transforms.empty()) << "nothing published on /tf";
    const auto& tf = transforms.back();
    EXPECT_EQ(tf.header.frame_id, "odom");
    EXPECT_EQ(tf.child_frame_id, "base_link");
    EXPECT_NEAR(tf.transform.translation.x, 1.5, 1e-9);
    EXPECT_NEAR(tf.transform.translation.y, -2.5, 1e-9);
    EXPECT_NEAR(tf.transform.rotation.z, std::sin(yaw / 2.0), 1e-9);
    EXPECT_NEAR(tf.transform.rotation.w, std::cos(yaw / 2.0), 1e-9);

    ASSERT_FALSE(odoms.empty()) << "nothing published on ~/odom";
    const auto& odom = odoms.back();
    EXPECT_EQ(odom.header.frame_id, "odom");
    EXPECT_EQ(odom.child_frame_id, "base_link");
    EXPECT_NEAR(odom.pose.pose.position.x, tf.transform.translation.x, 1e-9);
    EXPECT_NEAR(odom.pose.pose.position.y, tf.transform.translation.y, 1e-9);

    EXPECT_NEAR(odom.twist.twist.linear.x, 0.0, 1e-9)
        << "twist must be in base_link, not odom: at yaw pi/2 a world +x velocity is body -y";
    EXPECT_NEAR(odom.twist.twist.linear.y, -1.0, 1e-9);
    EXPECT_NEAR(odom.twist.twist.angular.z, 0.25, 1e-9) << "yaw rate is frame independent";

    EXPECT_GT(odom.pose.covariance[0], 0.0) << "all-zero covariance is a known Nav2 footgun";
    EXPECT_GT(odom.twist.covariance[0], 0.0);
}

TEST_F(OdometryPublisherTest, StopsPublishingWhenTheSourceGoesSilent)
{
    ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto helper    = std::make_shared<rclcpp::Node>("odom_test_helper_stale");
    auto state_pub = helper->create_publisher<sensor_msgs::msg::JointState>(
        "/g1_odometry_publisher/base_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    std::size_t transform_count = 0;
    auto        tf_sub          = helper->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS(100),
        [&transform_count](tf2_msgs::msg::TFMessage::SharedPtr msg) {
            transform_count += msg->transforms.size();
        });

    const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes = {
        node_->get_node_base_interface(),
        helper->get_node_base_interface()
    };
    spinFor(nodes, 300ms);

    for (int i = 0; i < 10; ++i)
    {
        state_pub->publish(makeBaseState(helper->now(), 0.0, 0.0, 0.0));
        spinFor(nodes, 20ms);
    }
    ASSERT_GT(transform_count, 0u) << "never published while the source was fresh";

    // Go quiet for well past source_timeout_ms, then check it actually stopped rather than
    // re-stamping the last pose forever.
    spinFor(nodes, 400ms);
    const std::size_t after_timeout = transform_count;
    spinFor(nodes, 300ms);
    EXPECT_EQ(transform_count, after_timeout)
        << "kept publishing " << (transform_count - after_timeout)
        << " transforms from a source that had gone silent";
}

TEST(OdometryPublisherSimTime, StopsPublishingWhenSimTimeItselfFreezes)
{
    // The failure the wall-clock test cannot see. /clock is published by the SAME process
    // as the base state on this track, so when that process wedges, sim time stops with
    // it: `now() - last_sample_stamp_` stays pinned near zero and a sim-time-only
    // staleness check never fires, leaving a frozen pose broadcast as if it were live.
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("sim_ground_truth", true));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto helper = std::make_shared<rclcpp::Node>("odom_test_helper_simtime");
    auto clock_pub =
        helper->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    auto state_pub = helper->create_publisher<sensor_msgs::msg::JointState>(
        "/g1_odometry_publisher/base_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    std::size_t transform_count = 0;
    auto        tf_sub          = helper->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS(100),
        [&transform_count](tf2_msgs::msg::TFMessage::SharedPtr msg) {
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
        state_pub->publish(makeBaseState(sim_now, 1.0, 2.0, 0.0));
        spinFor(nodes, 20ms);
    }
    ASSERT_GT(transform_count, 0u) << "never published while sim time was advancing";

    // Now the simulator wedges: /clock stops AND the stamp stops advancing, but samples
    // keep arriving, so the node still has fresh-looking data on a frozen clock. Wall time
    // is the only thing left that can notice.
    const std::size_t before_freeze = transform_count;
    for (int i = 0; i < 15; ++i)
    {
        state_pub->publish(makeBaseState(sim_now, 1.0, 2.0, 0.0));
        spinFor(nodes, 40ms);
    }
    const std::size_t after_timeout = transform_count;
    for (int i = 0; i < 10; ++i)
    {
        state_pub->publish(makeBaseState(sim_now, 1.0, 2.0, 0.0));
        spinFor(nodes, 40ms);
    }

    EXPECT_GT(after_timeout, before_freeze)
        << "sanity: the node should keep publishing for at least the timeout after the freeze";
    EXPECT_EQ(transform_count, after_timeout)
        << "published " << (transform_count - after_timeout)
        << " more transforms after sim time froze. With /clock stopped, elapsed sim time "
           "stays at zero forever, so only a wall-clock budget can catch this.";
}

TEST(OdometryPublisherGroundPlane, PublishesTheBaseHeightSoOdomIsTheGroundPlane)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsWithSource("sim_ground_truth"));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    auto helper    = std::make_shared<rclcpp::Node>("odom_test_helper_height");
    auto state_pub = helper->create_publisher<sensor_msgs::msg::JointState>(
        "/g1_odometry_publisher/base_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    auto tf_sub = helper->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf",
        rclcpp::QoS(100),
        [&transforms](tf2_msgs::msg::TFMessage::SharedPtr msg) {
            transforms.insert(transforms.end(), msg->transforms.begin(), msg->transforms.end());
        });

    const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes = {
        node->get_node_base_interface(),
        helper->get_node_base_interface()
    };
    spinFor(nodes, 300ms);
    for (int i = 0; i < 20; ++i)
    {
        state_pub->publish(makeBaseState(helper->now(), 0.0, 0.0, 0.0));
        spinFor(nodes, 20ms);
    }

    ASSERT_FALSE(transforms.empty());
    // Without this, a floor return transformed into odom lands at -0.793 and every Nav2
    // obstacle height band is off by the spawn height.
    EXPECT_NEAR(transforms.back().transform.translation.z, 0.793, 1e-9);
}

// --- Converged track: the split chain and the tilt guard ------------------------------------
//
// The planar suites above never reach this code. sim_sportmodestate is the only source that
// publishes two edges, and the tilt guard lives on a callback only that source subscribes to.

TEST(OdometryPublisherConverged, PublishesTheSplitChainWithOneStamp)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsForConverged());
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ConvergedHarness harness(node, "converged_split_helper");
    // A walking attitude: a few degrees of roll and pitch under a real heading.
    const double roll = -0.05, pitch = 0.0847, yaw = 1.2, height = 0.758;
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

TEST(OdometryPublisherConverged, OdometryDescribesTheFootprintNotTheBody)
{
    auto node = std::make_shared<G1OdometryPublisher>(optionsForConverged());
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ConvergedHarness harness(node, "converged_odom_helper");
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

TEST(OdometryPublisherConverged, TiltGuardHoldsTheLastGoodHeadingAndWarnsOnce)
{
    // 10 degrees, so the "past the threshold" case is an ordinary attitude rather than a
    // near-singular one -- this is testing the guard, not the arithmetic at 90 degrees.
    auto node = std::make_shared<G1OdometryPublisher>(optionsForConverged(10.0));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ConvergedHarness harness(node, "converged_tilt_helper");

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

    // The attitude itself keeps going out -- a fallen robot really is tilted, and hiding that
    // would be its own lie.
    const auto body = harness.latest("base_footprint", "pelvis");
    ASSERT_TRUE(body.has_value());
    EXPECT_GT(tiltOf(body->transform.rotation), 0.5) << "the real tilt must still reach TF";

    // Throttled at 2 s, so 15 samples inside that window give one line rather than fifteen.
    //
    // RCLCPP_WARN_THROTTLE keeps its last-logged timestamp in a static local at the CALL SITE,
    // not per node, so every test in this binary shares one 2 s window. Any other test that
    // drives this branch would silently zero this count. TiltGuardLatchesTheFirstSampleEvenMidFall
    // is the only other tilted test and it deliberately stops at one sample, which never reaches
    // the warn. Keep it that way, or this assertion becomes order-dependent.
    EXPECT_EQ(warns, 1) << "expected exactly one throttled warning, got " << warns;
}

TEST(OdometryPublisherConverged, TiltGuardLatchesTheFirstSampleEvenMidFall)
{
    // The documented spawn-topple case: if the very first attitude is already past the limit
    // there is nothing to hold instead, so it latches rather than publishing a default zero
    // heading that no sensor ever reported.
    auto node = std::make_shared<G1OdometryPublisher>(optionsForConverged(10.0));
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    ConvergedHarness harness(node, "converged_first_sample_helper");
    const double     yaw = -0.9;
    // Exactly one sample. A second would take the hold branch and trip the warning throttle
    // that TiltGuardHoldsTheLastGoodHeadingAndWarnsOnce counts -- see the note there.
    harness.feed(0.0, 0.0, 0.5, 0.0, 0.7, yaw, /*count=*/1);

    const auto foot = harness.latest("odom", "base_footprint");
    ASSERT_TRUE(foot.has_value()) << "nothing published at all";
    EXPECT_NEAR(yawOf(foot->transform.rotation), yaw, 1e-3);
}

TEST(OdometryPublisherConverged, RejectsAnOutOfRangeMaxTilt)
{
    for (double degrees : { 0.0, -5.0, 180.0, 400.0 })
    {
        auto node = std::make_shared<G1OdometryPublisher>(optionsForConverged(degrees));
        EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
            << "max_tilt_deg " << degrees << " should not configure";
    }
}

TEST(OdometryPublisherConverged, RejectsAFrameChainThatCannotExist)
{
    auto same = optionsForConverged();
    same.append_parameter_override("pelvis_frame_id", "base_footprint");
    EXPECT_EQ(
        std::make_shared<G1OdometryPublisher>(same)->configure().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
        << "a self-loop edge would be rejected by tf2, with an error pointing at tf2";

    auto empty = optionsForConverged();
    empty.append_parameter_override("base_frame_id", std::string(""));
    EXPECT_EQ(
        std::make_shared<G1OdometryPublisher>(empty)->configure().id(),
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

int main(int argc, char** argv)
{
    // Isolated domain: a running sim on the default domain must not be able to feed this.
    setenv("ROS_DOMAIN_ID", "77", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
