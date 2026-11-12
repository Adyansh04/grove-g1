/**
 * @file test_system.cpp
 * @brief G1InspireSystem through its lifecycle against a fake hand, and its plugin export.
 */

#include <gmock/gmock.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <thread>

#include "fake_hand.hpp"
#include "g1_inspire_hand/g1_inspire_system.hpp"
#include "hardware_interface/system_interface.hpp"

namespace
{

using g1_inspire_hand::G1InspireSystem;
using g1_inspire_hand::test::FakeHand;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace reg = g1_inspire_hand::reg;
using namespace std::chrono_literals;

hardware_interface::ComponentInfo actuatorJoint(const std::string& name, double open, double closed)
{
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.command_interfaces.push_back({ .name = "position" });
    for (const char* state : { "position", "velocity", "effort" })
    {
        joint.state_interfaces.push_back({ .name = state });
    }
    joint.parameters = { { "open_rad", std::to_string(open) },
                         { "closed_rad", std::to_string(closed) },
                         { "lever_arm_m", "0.05" } };
    return joint;
}

hardware_interface::HardwareComponentInterfaceParams
leftHand(const std::string& port, const std::string& protocol = "inspire")
{
    hardware_interface::HardwareComponentInterfaceParams params;
    auto&                                                info = params.hardware_info;
    info.name                                                 = "G1InspireSystemLeft";
    info.type                                                 = "system";
    info.hardware_plugin_name                                 = "g1_inspire_hand/G1InspireSystem";
    info.hardware_parameters                                  = {
        { "side", "left" },
        { "serial_port", port },
        { "protocol", protocol },
        { "baud_rate", "115200" },
        { "slave_id", "1" },
        { "io_rate_hz", "200" },
        { "response_timeout_ms", "20" },
        { "state_timeout_ms", "300" },
        { "diagnostics_every", "5" },
        { "speed", "800" },
        { "force_limit_g", "300" },
        { "max_joint_velocity_rad_s", "3.0" },
    };
    for (std::size_t i = 0; i < g1_inspire_hand::kNumActuators; ++i)
    {
        const bool rotation = i == 5;
        const bool bend     = i == 4;
        info.joints.push_back(actuatorJoint(
            std::string("L_") + g1_inspire_hand::kActuatorJointSuffixes[i],
            rotation ? -0.1 : 0.0,
            rotation ? 1.3 : (bend ? 0.5 : 1.7)));
    }
    return params;
}

const rclcpp_lifecycle::State
    kInactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
const rclcpp_lifecycle::State kActive(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");

/// A component with its interfaces exported, as controller_manager holds it.
struct Exported
{
    G1InspireSystem                                   system;
    std::vector<hardware_interface::StateInterface>   states;
    std::vector<hardware_interface::CommandInterface> commands;

    double state(const std::string& name) const
    {
        for (const auto& s : states)
        {
            if (s.get_name() == name)
            {
                return s.get_optional<double>().value();
            }
        }
        ADD_FAILURE() << "no state interface " << name;
        return 0.0;
    }

    void command(const std::string& name, double value)
    {
        for (auto& c : commands)
        {
            if (c.get_name() == name)
            {
                static_cast<void>(c.set_value(value));
                return;
            }
        }
        ADD_FAILURE() << "no command interface " << name;
    }

    /// Runs the update loop at 100 Hz for @p duration.
    void run(std::chrono::milliseconds duration)
    {
        const rclcpp::Duration tick(10ms);
        for (auto spent = 0ms; spent < duration; spent += 10ms)
        {
            EXPECT_EQ(system.read(rclcpp::Time(0), tick), return_type::OK);
            EXPECT_EQ(system.write(rclcpp::Time(0), tick), return_type::OK);
            std::this_thread::sleep_for(10ms);
        }
    }
};

std::unique_ptr<Exported> activated(const FakeHand& fake, const std::string& protocol = "inspire")
{
    auto hand = std::make_unique<Exported>();
    EXPECT_EQ(hand->system.on_init(leftHand(fake.port(), protocol)), CallbackReturn::SUCCESS);
    hand->states   = hand->system.export_state_interfaces();
    hand->commands = hand->system.export_command_interfaces();
    EXPECT_EQ(hand->system.on_activate(kInactive), CallbackReturn::SUCCESS);
    return hand;
}

TEST(InspireSystem, IsFoundThroughItsPluginExport)
{
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
        "hardware_interface",
        "hardware_interface::SystemInterface");
    EXPECT_NE(loader.createUniqueInstance("g1_inspire_hand/G1InspireSystem"), nullptr);
}

TEST(InspireSystem, RefusesJointsOutOfRegisterOrder)
{
    auto params = leftHand("/dev/null");
    std::swap(params.hardware_info.joints[0], params.hardware_info.joints[3]);
    G1InspireSystem system;
    EXPECT_EQ(system.on_init(params), CallbackReturn::ERROR);
}

TEST(InspireSystem, RefusesAnUnknownProtocolAndAMissingPort)
{
    G1InspireSystem bad_protocol;
    EXPECT_EQ(bad_protocol.on_init(leftHand("/dev/null", "modbus")), CallbackReturn::ERROR);
    G1InspireSystem no_port;
    EXPECT_EQ(no_port.on_init(leftHand("")), CallbackReturn::ERROR);
}

TEST(InspireSystem, ActivationSetsTheLimitsAndSeedsFromTheMeasuredAngles)
{
    FakeHand fake;
    fake.setWords(reg::kAngleAct, { 1000, 1000, 500, 0, 1000, 0 });
    fake.setByte(reg::kError + 1, 0x01);
    auto hand = activated(fake);
    hand->run(100ms);

    EXPECT_NEAR(hand->state("L_pinky_proximal_joint/position"), 0.0, 1e-9);
    EXPECT_NEAR(hand->state("L_middle_proximal_joint/position"), 0.85, 1e-9);
    EXPECT_NEAR(hand->state("L_index_proximal_joint/position"), 1.7, 1e-9);
    EXPECT_NEAR(hand->state("L_thumb_proximal_yaw_joint/position"), 1.3, 1e-9);
    EXPECT_THAT(fake.words(reg::kSpeedSet), ::testing::Each(800));
    EXPECT_THAT(fake.words(reg::kForceSet), ::testing::Each(300));
    EXPECT_EQ(fake.byteAt(reg::kError + 1), 0);
    // Seeded commands equal the measurement, so nothing moves and nothing is sent.
    EXPECT_EQ(fake.angleWrites(), 0);
    hand->system.on_deactivate(kActive);
}

TEST(InspireSystem, ACommandIsSlewedToItsTarget)
{
    FakeHand fake;
    auto     hand = activated(fake);
    hand->command("L_index_proximal_joint/position", 1.7);
    // 3 rad/s from 0.85 rad: 0.3 s to close, so a tenth of a second gets about a third there.
    hand->run(100ms);
    const double midway = hand->state("L_index_proximal_joint/position");
    EXPECT_GT(midway, 0.95);
    EXPECT_LT(midway, 1.35);
    hand->run(400ms);
    EXPECT_NEAR(hand->state("L_index_proximal_joint/position"), 1.7, 1e-3);
    EXPECT_EQ(fake.words(reg::kAngleSet)[3], 0);
    // The other fingers were never commanded and stayed put.
    EXPECT_NEAR(hand->state("L_ring_proximal_joint/position"), 0.85, 1e-3);
    hand->system.on_deactivate(kActive);
}

TEST(InspireSystem, AStopLeavesTheHandWhereItIs)
{
    FakeHand fake;
    auto     hand = activated(fake);
    hand->command("L_middle_proximal_joint/position", 1.7);
    hand->run(150ms);
    ASSERT_EQ(hand->system.on_deactivate(kActive), CallbackReturn::SUCCESS);
    const auto held   = fake.words(reg::kAngleSet);
    const int  writes = fake.angleWrites();
    std::this_thread::sleep_for(100ms);
    // Nothing more goes out, an open least of all.
    EXPECT_EQ(fake.angleWrites(), writes);
    EXPECT_EQ(fake.words(reg::kAngleSet), held);
    EXPECT_LT(held[2], 500);
}

TEST(InspireSystem, AHandThatStopsAnsweringErrorsTheComponent)
{
    FakeHand fake;
    auto     hand = activated(fake);
    hand->run(50ms);
    fake.setSilent(true);
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(hand->system.read(rclcpp::Time(0), rclcpp::Duration(10ms)), return_type::ERROR);
    hand->system.on_error(kActive);
}

TEST(InspireSystem, AHandThatNeverAnswersFailsActivation)
{
    FakeHand fake;
    fake.setSilent(true);
    G1InspireSystem system;
    ASSERT_EQ(system.on_init(leftHand(fake.port())), CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_activate(kInactive), CallbackReturn::ERROR);
}

TEST(InspireSystem, RunsTheSameOverModbus)
{
    FakeHand fake;
    auto     hand = activated(fake, "modbus_rtu");
    hand->command("L_thumb_proximal_pitch_joint/position", 0.5);
    hand->run(400ms);
    EXPECT_NEAR(hand->state("L_thumb_proximal_pitch_joint/position"), 0.5, 1e-3);
    hand->system.on_deactivate(kActive);
}

}  // namespace
