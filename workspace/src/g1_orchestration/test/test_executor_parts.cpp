/**
 * @file test_executor_parts.cpp
 * @brief The executor's health, teleop and preview logic, without ROS running.
 */

#include <gmock/gmock.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <set>
#include <string>

#include "g1_orchestration/mission_preview.hpp"
#include "g1_orchestration/robot_health.hpp"
#include "g1_orchestration/teleop_driver.hpp"

namespace
{
using g1_orchestration::Health;
using g1_orchestration::MissionPreview;
using g1_orchestration::RobotHealth;
using g1_orchestration::Station;
using g1_orchestration::TeleopDriver;
using g1_orchestration::Velocity;

RobotHealth::Params g1Health()
{
    RobotHealth::Params params;
    params.imu_topic          = "/imu";
    params.max_tilt_deg       = 45.0;
    params.walk_controllers   = { "agile_controller" };
    params.stand_controllers  = { "locomotion_freeze_controller" };
    params.controller_manager = "/controller_manager";
    return params;
}

TEST(RobotHealth, TiltIsTheAngleBetweenTheBodysUpAndTheWorlds)
{
    EXPECT_NEAR(RobotHealth::tiltDeg(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9);
    const double half = std::numbers::pi / 4.0;  // 90 degrees about x
    EXPECT_NEAR(RobotHealth::tiltDeg(std::sin(half), 0.0, 0.0, std::cos(half)), 90.0, 1e-9);
    EXPECT_NEAR(RobotHealth::tiltDeg(0.0, 0.0, std::sin(half), std::cos(half)), 0.0, 1e-9)
        << "a turn about the vertical is not a tilt";
    EXPECT_NEAR(RobotHealth::tiltDeg(0.0, 2.0, 0.0, 0.0), 180.0, 1e-9) << "unnormalised";
    EXPECT_TRUE(std::isnan(RobotHealth::tiltDeg(0.0, 0.0, 0.0, 0.0)));
}

TEST(RobotHealth, WalksOnlyUprightWithTheBalancePolicyHoldingTheLegs)
{
    const auto   params = g1Health();
    const Health ok = RobotHealth::assess(3.0, std::set<std::string>{ "agile_controller" }, params);
    EXPECT_TRUE(ok.can_walk);
    EXPECT_TRUE(ok.reason.empty());
    EXPECT_EQ(ok.legs, "agile_controller");

    const Health fallen =
        RobotHealth::assess(104.0, std::set<std::string>{ "agile_controller" }, params);
    EXPECT_FALSE(fallen.can_walk);
    EXPECT_FALSE(fallen.can_stand);
    EXPECT_EQ(fallen.reason, "fallen: tilted 104 degrees");

    const Health frozen =
        RobotHealth::assess(2.0, std::set<std::string>{ "locomotion_freeze_controller" }, params);
    EXPECT_FALSE(frozen.can_walk);
    EXPECT_TRUE(frozen.can_stand) << "the arms may still move in place";
    EXPECT_THAT(frozen.reason, ::testing::HasSubstr("balance policy is not running"));

    const Health limp = RobotHealth::assess(2.0, std::set<std::string>{}, params);
    EXPECT_FALSE(limp.can_stand);
    EXPECT_EQ(limp.reason, "no controller holds the legs");
}

TEST(RobotHealth, AMissingReadingIsNotAnAllClear)
{
    const auto   params = g1Health();
    const Health no_imu =
        RobotHealth::assess(std::nullopt, std::set<std::string>{ "agile_controller" }, params);
    EXPECT_FALSE(no_imu.can_walk);
    EXPECT_EQ(no_imu.reason, "no tilt reading from /imu");

    const Health no_answer = RobotHealth::assess(1.0, std::nullopt, params);
    EXPECT_FALSE(no_answer.can_walk);
    EXPECT_EQ(no_answer.reason, "/controller_manager does not answer");

    auto unconfigured               = params;
    unconfigured.imu_topic          = "";
    unconfigured.controller_manager = "";
    EXPECT_TRUE(RobotHealth::assess(std::nullopt, std::nullopt, unconfigured).can_walk)
        << "a check without a source is passed";
}

TEST(TeleopDriver, ClampsToTheLimitsAndRampsInsteadOfSnapping)
{
    TeleopDriver::Params params;
    params.max_x    = 0.5;
    params.accel_xy = 1.0;
    Velocity v;
    v = TeleopDriver::step(v, { 2.0, 0.0, 0.0 }, 0.1, params);
    EXPECT_NEAR(v.x, 0.1, 1e-12) << "one period at 1 m/s^2";
    for (int i = 0; i < 20; ++i)
    {
        v = TeleopDriver::step(v, { 2.0, 0.0, 0.0 }, 0.1, params);
    }
    EXPECT_NEAR(v.x, 0.5, 1e-12) << "held to the limit";
    v = TeleopDriver::step(v, {}, 0.1, params);
    EXPECT_NEAR(v.x, 0.4, 1e-12) << "slows at the limit too";
}

TEST(TeleopDriver, ACommandThatIsNotANumberIsNoCommand)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto   v   = TeleopDriver::step({}, { nan, nan, nan }, 0.1, TeleopDriver::Params{});
    EXPECT_TRUE(v.isZero());
}

TEST(MissionPreview, AWalkGoesAlongTheHeadingAndATurnWrapsAround)
{
    const Station north{ 1.0, 2.0, std::numbers::pi / 2.0 };
    const Station walked = MissionPreview::walkEnd(north, 1.5);
    EXPECT_NEAR(walked.x, 1.0, 1e-12);
    EXPECT_NEAR(walked.y, 3.5, 1e-12);
    const Station back = MissionPreview::walkEnd(north, -1.0);
    EXPECT_NEAR(back.y, 1.0, 1e-12);

    const Station turned = MissionPreview::turnEnd(north, 180.0);
    EXPECT_NEAR(turned.yaw, -std::numbers::pi / 2.0, 1e-12);
    EXPECT_DOUBLE_EQ(turned.x, north.x);
}
}  // namespace
