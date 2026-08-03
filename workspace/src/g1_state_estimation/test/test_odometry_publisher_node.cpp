/**
 * @file test_odometry_publisher_node.cpp
 * @brief In-process lifecycle tests for the odom -> base_link publisher.
 *
 * Runs on an isolated ROS_DOMAIN_ID so a sim or another test on the machine cannot feed it
 * real data, same pattern as g1_locomotion's test_loco_bridge_node.
 */

#include <gmock/gmock.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "g1_state_estimation/g1_odometry_publisher_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
