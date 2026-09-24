/**
 * @file test_object_pose_source_node.cpp
 * @brief In-process lifecycle tests for the object-pose source.
 *
 * On an isolated ROS_DOMAIN_ID, so a simulator running on the default domain cannot feed it.
 */

#include <gmock/gmock.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "g1_manipulation/g1_object_pose_source_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "vision_msgs/msg/detection3_d_array.hpp"

using g1_manipulation::G1ObjectPoseSource;
using g1_manipulation::isBarePhraseAlias;
using g1_manipulation::ObjectSource;
using g1_manipulation::parseObjectSource;
using namespace std::chrono_literals;

namespace
{

rclcpp::NodeOptions optionsWithSource(const std::string& source)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({ rclcpp::Parameter("object_source", source) });
    return options;
}

/// Distinct source and output frames, as deployed: a detector measures in a camera frame.
rclcpp::NodeOptions optionsWithFrames(const std::string& source, const std::string& output)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({ rclcpp::Parameter("object_source", "sim_ground_truth"),
                                  rclcpp::Parameter("source_frame_id", source),
                                  rclcpp::Parameter("output_frame_id", output),
                                  rclcpp::Parameter("publish_markers", false) });
    return options;
}

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

vision_msgs::msg::Detection3DArray makeGroundTruth(
    const std::string& frame_id, const std::string& object_id, double x, double y, double z)
{
    vision_msgs::msg::Detection3DArray msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp    = rclcpp::Time(123, 456);

    vision_msgs::msg::Detection3D detection;
    detection.header = msg.header;
    detection.id     = object_id;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id     = object_id;
    hypothesis.hypothesis.score        = 1.0;
    hypothesis.pose.pose.position.x    = x;
    hypothesis.pose.pose.position.y    = y;
    hypothesis.pose.pose.position.z    = z;
    hypothesis.pose.pose.orientation.w = 1.0;
    detection.bbox.center              = hypothesis.pose.pose;
    detection.results.push_back(hypothesis);
    msg.detections.push_back(detection);
    return msg;
}

/// Drives one node against a publisher on its source topic and captures /objects.
class Harness
{
public:
    explicit Harness(const rclcpp::NodeOptions& options)
      : node_(std::make_shared<G1ObjectPoseSource>(options))
      , peer_(std::make_shared<rclcpp::Node>("test_peer"))
    {
        source_pub_ = peer_->create_publisher<vision_msgs::msg::Detection3DArray>(
            "/g1_object_pose_source/object_poses",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        objects_sub_ = peer_->create_subscription<vision_msgs::msg::Detection3DArray>(
            "/g1_object_pose_source/objects",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) { last_ = *msg; });
    }

    unsigned int configure() { return node_->configure().id(); }
    unsigned int activate() { return node_->activate().id(); }

    void publishAndSpin(const vision_msgs::msg::Detection3DArray& msg)
    {
        // Discovery first: a publish before the subscription is discovered is lost.
        spinFor({ node_->get_node_base_interface(), peer_->get_node_base_interface() }, 300ms);
        source_pub_->publish(msg);
        spinFor({ node_->get_node_base_interface(), peer_->get_node_base_interface() }, 300ms);
    }

    const std::optional<vision_msgs::msg::Detection3DArray>& last() const { return last_; }

private:
    std::shared_ptr<G1ObjectPoseSource>                                 node_;
    std::shared_ptr<rclcpp::Node>                                       peer_;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr    source_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    std::optional<vision_msgs::msg::Detection3DArray>                   last_;
};

/// Publishes one static edge so the node has a real transform to apply.
std::shared_ptr<rclcpp::Node>
staticTf(const std::string& parent, const std::string& child, double x, double y, double z)
{
    auto node   = std::make_shared<rclcpp::Node>("test_tf");
    auto caster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = rclcpp::Time(0, 0);
    tf.header.frame_id         = parent;
    tf.child_frame_id          = child;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.w    = 1.0;
    caster->sendTransform(tf);
    node->set_parameter(rclcpp::Parameter("use_sim_time", false));
    // The latched publisher lives in the broadcaster, so it is kept for the whole run.
    static std::vector<std::shared_ptr<tf2_ros::StaticTransformBroadcaster>> keep;
    keep.push_back(caster);
    return node;
}

}  // namespace

// A relabel that left the numbers alone would leave x at 1.0.
TEST(ObjectPoseSource, TransformsThePoseRatherThanRelabellingTheFrame)
{
    auto    tf_node = staticTf("odom", "camera_color_optical_frame", 2.0, -3.0, 0.5);
    Harness harness{ optionsWithFrames("camera_color_optical_frame", "odom") };
    ASSERT_EQ(harness.configure(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(harness.activate(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    spinFor({ tf_node->get_node_base_interface() }, 300ms);

    harness.publishAndSpin(
        makeGroundTruth("camera_color_optical_frame", "red_block", 1.0, 0.5, 0.25));

    ASSERT_TRUE(harness.last().has_value());
    const auto& out = *harness.last();
    EXPECT_EQ(out.header.frame_id, "odom");
    ASSERT_EQ(out.detections.size(), 1U);
    const auto& pose = out.detections[0].results[0].pose.pose;
    EXPECT_NEAR(pose.position.x, 3.0, 1e-6);
    EXPECT_NEAR(pose.position.y, -2.5, 1e-6);
    EXPECT_NEAR(pose.position.z, 0.75, 1e-6);
    // The bbox centre is transformed too, not only the hypothesis.
    EXPECT_NEAR(out.detections[0].bbox.center.position.x, 3.0, 1e-6);
}

// A frame no test broadcasts: static transforms are transient local and outlive the test that
// sent them.
TEST(ObjectPoseSource, PublishesNothingWhenTheTransformIsMissing)
{
    Harness harness{ optionsWithFrames("unbroadcast_sensor_frame", "odom") };
    ASSERT_EQ(harness.configure(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(harness.activate(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    harness.publishAndSpin(makeGroundTruth("unbroadcast_sensor_frame", "red_block", 1.0, 0.5, 0.25));

    EXPECT_FALSE(harness.last().has_value());
}

TEST(ObjectSource, ParsesTheSourcesItKnowsAndRejectsTheRest)
{
    ObjectSource source = ObjectSource::kHardware;
    ASSERT_TRUE(parseObjectSource("sim_ground_truth", source));
    EXPECT_EQ(source, ObjectSource::kSimGroundTruth);
    ASSERT_TRUE(parseObjectSource("perception", source));
    EXPECT_EQ(source, ObjectSource::kPerception);
    ASSERT_TRUE(parseObjectSource("hardware", source));
    EXPECT_EQ(source, ObjectSource::kHardware);

    ObjectSource untouched = ObjectSource::kSimGroundTruth;
    EXPECT_FALSE(parseObjectSource("ground_truth", untouched));
    EXPECT_FALSE(parseObjectSource("", untouched));
    EXPECT_EQ(untouched, ObjectSource::kSimGroundTruth) << "a rejected name must not assign";
}

TEST(ObjectMarkers, SkipsOnlyTheBarePhraseAliasOfATrack)
{
    vision_msgs::msg::Detection3DArray objects;
    for (const char* id : { "red_block_0", "red_block", "red_block_top", "blue_sphere" })
    {
        objects.detections.emplace_back().id = id;
    }

    EXPECT_TRUE(isBarePhraseAlias("red_block", objects));
    EXPECT_FALSE(isBarePhraseAlias("red_block_0", objects));
    // A track of its own, not an alias: nothing named blue_sphere_<n> is present.
    EXPECT_FALSE(isBarePhraseAlias("blue_sphere", objects));
    // Only a numeric suffix makes an index; red_block_top does not make red_block an alias of it.
    objects.detections.erase(objects.detections.begin());
    EXPECT_FALSE(isBarePhraseAlias("red_block", objects));
}

TEST(ObjectPoseSource, RefusesToConfigureOnHardware)
{
    // Fails loudly rather than hand a planner simulator truth it cannot tell from a measurement.
    auto node = std::make_shared<G1ObjectPoseSource>(optionsWithSource("hardware"));
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ObjectPoseSource, RefusesToConfigureOnAnUnknownSource)
{
    auto node = std::make_shared<G1ObjectPoseSource>(optionsWithSource("ground_truth"));
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ObjectPoseSource, DefaultsToHardwareSoSimulationHasToOptIn)
{
    // No parameter overrides at all: the default has to be the refusing one.
    auto node = std::make_shared<G1ObjectPoseSource>(rclcpp::NodeOptions());
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST(ObjectPoseSource, RepublishesGroundTruthInTheOutputFrame)
{
    Harness harness{ optionsWithSource("sim_ground_truth") };
    ASSERT_EQ(harness.configure(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(harness.activate(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    harness.publishAndSpin(makeGroundTruth("odom", "red_block", 4.1, -4.65, 0.78));

    ASSERT_TRUE(harness.last().has_value());
    const auto& out = *harness.last();
    EXPECT_EQ(out.header.frame_id, "odom");
    ASSERT_EQ(out.detections.size(), 1U);
    // The per-detection header is relabelled too, not only the array's.
    EXPECT_EQ(out.detections[0].header.frame_id, "odom");
    ASSERT_EQ(out.detections[0].results.size(), 1U);
    EXPECT_EQ(out.detections[0].results[0].hypothesis.class_id, "red_block");
    EXPECT_DOUBLE_EQ(out.detections[0].results[0].pose.pose.position.x, 4.1);
    EXPECT_DOUBLE_EQ(out.detections[0].results[0].pose.pose.position.z, 0.78);
}

TEST(ObjectPoseSource, CarriesTheSourceStampRatherThanRestampingIt)
{
    // A restamp would pass a stale pose as fresh, and skills gate a grasp on freshness.
    Harness harness{ optionsWithSource("sim_ground_truth") };
    ASSERT_EQ(harness.configure(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(harness.activate(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    harness.publishAndSpin(makeGroundTruth("odom", "red_block", 1.0, 2.0, 0.75));

    ASSERT_TRUE(harness.last().has_value());
    // Field by field: rclcpp::Time throws comparing the message's RCL_ROS_TIME with a
    // RCL_SYSTEM_TIME literal.
    EXPECT_EQ(harness.last()->header.stamp.sec, 123);
    EXPECT_EQ(harness.last()->header.stamp.nanosec, 456U);
}

TEST(ObjectPoseSource, DropsPosesStampedWithAFrameItWasNotConfiguredFor)
{
    // optionsWithSource keeps the default source frame, odom, so a camera-frame sample is foreign.
    Harness harness{ optionsWithSource("sim_ground_truth") };
    ASSERT_EQ(harness.configure(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(harness.activate(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    harness.publishAndSpin(
        makeGroundTruth("camera_color_optical_frame", "red_block", 0.3, 0.0, 0.5));

    EXPECT_FALSE(harness.last().has_value());
}

TEST(ObjectPoseSource, StaysQuietUntilActivated)
{
    Harness harness{ optionsWithSource("sim_ground_truth") };
    ASSERT_EQ(harness.configure(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    harness.publishAndSpin(makeGroundTruth("odom", "red_block", 4.1, -4.65, 0.78));

    EXPECT_FALSE(harness.last().has_value());
}

int main(int argc, char** argv)
{
    // Set before any thread exists, so setenv's thread-safety warning does not apply.
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    setenv("ROS_DOMAIN_ID", "78", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
