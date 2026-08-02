#include <gmock/gmock.h>

#include <cmath>

#include "g1_state_estimation/odom_math.hpp"

using g1_state_estimation::diagonalCovariance;
using g1_state_estimation::isStale;
using g1_state_estimation::OdometrySource;
using g1_state_estimation::parseOdometrySource;
using g1_state_estimation::PlanarTwist;
using g1_state_estimation::Quaternion;
using g1_state_estimation::quaternionToYaw;
using g1_state_estimation::toBodyTwist;
using g1_state_estimation::wrapAngle;
using g1_state_estimation::yawToQuaternion;

namespace
{
constexpr double kTol = 1e-12;
}

TEST(ParseOdometrySource, AcceptsTheTwoKnownNames)
{
    OdometrySource source = OdometrySource::Hardware;
    ASSERT_TRUE(parseOdometrySource("sim_ground_truth", source));
    EXPECT_EQ(source, OdometrySource::SimGroundTruth);

    ASSERT_TRUE(parseOdometrySource("hardware", source));
    EXPECT_EQ(source, OdometrySource::Hardware);
}

TEST(ParseOdometrySource, RejectsAnythingElseAndLeavesTheOutputAlone)
{
    // A typo must not silently become sim ground truth, which would fabricate transforms.
    OdometrySource source = OdometrySource::SimGroundTruth;
    for (const char* name : { "", "sim", "SIM_GROUND_TRUTH", "ground_truth", "hardware " })
    {
        EXPECT_FALSE(parseOdometrySource(name, source)) << "accepted " << name;
        EXPECT_EQ(source, OdometrySource::SimGroundTruth);
    }
}

TEST(YawQuaternion, RoundTripsOverTheFullCircle)
{
    for (double yaw = -M_PI + 1e-6; yaw < M_PI; yaw += 0.1)
    {
        EXPECT_NEAR(quaternionToYaw(yawToQuaternion(yaw)), yaw, 1e-9) << "yaw " << yaw;
    }
}

TEST(YawQuaternion, MatchesKnownValues)
{
    const Quaternion identity = yawToQuaternion(0.0);
    EXPECT_NEAR(identity.z, 0.0, kTol);
    EXPECT_NEAR(identity.w, 1.0, kTol);

    const Quaternion quarter = yawToQuaternion(M_PI_2);
    EXPECT_NEAR(quarter.z, std::sqrt(0.5), 1e-12);
    EXPECT_NEAR(quarter.w, std::sqrt(0.5), 1e-12);

    // x and y stay zero: this is a planar body, and a stray tilt here would tip every
    // sensor frame hanging off base_link.
    for (double yaw : { 0.0, 1.0, -2.5, M_PI })
    {
        const Quaternion q = yawToQuaternion(yaw);
        EXPECT_NEAR(q.x, 0.0, kTol);
        EXPECT_NEAR(q.y, 0.0, kTol);
    }
}

TEST(WrapAngle, MapsOntoTheHalfOpenInterval)
{
    EXPECT_NEAR(wrapAngle(0.0), 0.0, kTol);
    EXPECT_NEAR(wrapAngle(M_PI), M_PI, kTol);
    EXPECT_NEAR(wrapAngle(-M_PI), M_PI, kTol) << "-pi and +pi are the same rotation; pick one";
    EXPECT_NEAR(wrapAngle(3.0 * M_PI), M_PI, 1e-12);
    EXPECT_NEAR(wrapAngle(2.0 * M_PI + 0.25), 0.25, 1e-12);
    EXPECT_NEAR(wrapAngle(-2.0 * M_PI - 0.25), -0.25, 1e-12);

    // The yaw hinge is continuous, so many turns is the realistic input.
    EXPECT_NEAR(wrapAngle(20.0 * M_PI + 0.5), 0.5, 1e-9);
}

TEST(ToBodyTwist, IsIdentityAtZeroYaw)
{
    const PlanarTwist body = toBodyTwist({ 1.0, 2.0, 0.5 }, 0.0);
    EXPECT_NEAR(body.vx, 1.0, kTol);
    EXPECT_NEAR(body.vy, 2.0, kTol);
    EXPECT_NEAR(body.omega, 0.5, kTol);
}

TEST(ToBodyTwist, RotatesIntoTheBodyFrameAtRightAngles)
{
    // Facing +y in the world: driving world +x is driving body -y.
    const PlanarTwist quarter = toBodyTwist({ 1.0, 0.0, 0.0 }, M_PI_2);
    EXPECT_NEAR(quarter.vx, 0.0, 1e-12);
    EXPECT_NEAR(quarter.vy, -1.0, 1e-12);

    const PlanarTwist minus_quarter = toBodyTwist({ 1.0, 0.0, 0.0 }, -M_PI_2);
    EXPECT_NEAR(minus_quarter.vx, 0.0, 1e-12);
    EXPECT_NEAR(minus_quarter.vy, 1.0, 1e-12);

    const PlanarTwist half = toBodyTwist({ 1.0, 0.0, 0.0 }, M_PI);
    EXPECT_NEAR(half.vx, -1.0, 1e-12);
    EXPECT_NEAR(half.vy, 0.0, 1e-12);
}

TEST(ToBodyTwist, PreservesSpeedAndYawRate)
{
    const PlanarTwist world{ 0.3, -0.7, 0.9 };
    const double      world_speed = std::hypot(world.vx, world.vy);
    for (double yaw = -3.0; yaw < 3.0; yaw += 0.37)
    {
        const PlanarTwist body = toBodyTwist(world, yaw);
        EXPECT_NEAR(std::hypot(body.vx, body.vy), world_speed, 1e-12) << "yaw " << yaw;
        EXPECT_NEAR(body.omega, world.omega, kTol) << "yaw rate is frame independent";
    }
}

TEST(IsStale, TreatsTheBoundaryAsFresh)
{
    EXPECT_FALSE(isStale(0.199, 0.2));
    EXPECT_FALSE(isStale(0.2, 0.2)) << "exactly at the timeout is not yet stale";
    EXPECT_TRUE(isStale(0.2000001, 0.2));
}

TEST(IsStale, NonPositiveTimeoutDisablesTheCheck)
{
    EXPECT_FALSE(isStale(1e6, 0.0));
    EXPECT_FALSE(isStale(1e6, -1.0));
}

TEST(DiagonalCovariance, FillsOnlyTheDiagonal)
{
    const std::array<double, 36> covariance = diagonalCovariance(1.0e-6);
    for (std::size_t row = 0; row < 6; ++row)
    {
        for (std::size_t col = 0; col < 6; ++col)
        {
            const double expected = (row == col) ? 1.0e-6 : 0.0;
            EXPECT_DOUBLE_EQ(covariance[row * 6 + col], expected) << row << "," << col;
        }
    }
}
