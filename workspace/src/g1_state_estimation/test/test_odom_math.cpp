#include <gmock/gmock.h>

#include <cmath>
#include <limits>

#include "g1_state_estimation/odom_math.hpp"

using g1_state_estimation::composePose;
using g1_state_estimation::diagonalCovariance;
using g1_state_estimation::GroundSplit;
using g1_state_estimation::invertPose;
using g1_state_estimation::isStale;
using g1_state_estimation::OdometrySource;
using g1_state_estimation::parseOdometrySource;
using g1_state_estimation::PlanarTwist;
using g1_state_estimation::Pose3d;
using g1_state_estimation::Quaternion;
using g1_state_estimation::quaternionToYaw;
using g1_state_estimation::splitGroundProjection;
using g1_state_estimation::tiltFromVertical;
using g1_state_estimation::toBodyTwist;
using g1_state_estimation::wrapAngle;
using g1_state_estimation::yawToQuaternion;

namespace
{
constexpr double kTol = 1e-12;
}

TEST(ParseOdometrySource, AcceptsEveryKnownName)
{
    OdometrySource source = OdometrySource::kHardware;
    ASSERT_TRUE(parseOdometrySource("sim_sportmodestate", source));
    EXPECT_EQ(source, OdometrySource::kSimSportModeState);

    ASSERT_TRUE(parseOdometrySource("fast_lio", source));
    EXPECT_EQ(source, OdometrySource::kFastLio);

    ASSERT_TRUE(parseOdometrySource("hardware", source));
    EXPECT_EQ(source, OdometrySource::kHardware);
}

TEST(ParseOdometrySource, RejectsAnythingElseAndLeavesTheOutputAlone)
{
    // A typo must not silently become a working source, which would fabricate transforms.
    // sim_ground_truth is in the list because it named a source this node no longer has: a
    // stale config still naming it has to fail rather than quietly pick something else.
    OdometrySource source = OdometrySource::kSimSportModeState;
    for (const char* name : { "",
                              "sim",
                              "sim_ground_truth",
                              "SIM_SPORTMODESTATE",
                              "ground_truth",
                              "hardware ",
                              "fastlio",
                              "sportmodestate",
                              "sim_sportmode" })
    {
        EXPECT_FALSE(parseOdometrySource(name, source)) << "accepted " << name;
        EXPECT_EQ(source, OdometrySource::kSimSportModeState);
    }
}

TEST(YawQuaternion, RoundTripsOverTheFullCircle)
{
    for (int i = 0; - M_PI + 1e-6 + i * 0.1 < M_PI; ++i)
    {
        const double yaw = -M_PI + 1e-6 + i * 0.1;
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

    // x and y stay zero: this is a planar body, and a stray tilt here would tip
    // every sensor frame hanging off base_link.
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
    for (int i = 0; - 3.0 + i * 0.37 < 3.0; ++i)
    {
        const double      yaw  = -3.0 + i * 0.37;
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

namespace
{
/// Roll-pitch-yaw to quaternion, ZYX order -- the convention quaternionToYaw()
/// inverts.
Quaternion rpyToQuaternion(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    Quaternion   q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

/// Hamilton product, for recomposing a split chain.
Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    Quaternion out;
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return out;
}
}  // namespace

TEST(QuaternionToYaw, IgnoresRollAndPitch)
{
    // The measured standing attitude on the converged track: a few degrees of
    // pitch under a real heading. The old 2*atan2(z, w) form was exact only at
    // zero tilt.
    for (int i = 0; - 3.0 + i * 0.41 < 3.0; ++i)
    {
        const double     yaw = -3.0 + i * 0.41;
        const Quaternion q   = rpyToQuaternion(0.0, 0.0794, yaw);
        EXPECT_NEAR(quaternionToYaw(q), wrapAngle(yaw), 1e-9) << "yaw " << yaw;
    }
}

TEST(TiltFromVertical, MeasuresRollAndPitchTogether)
{
    EXPECT_NEAR(tiltFromVertical(yawToQuaternion(2.0)), 0.0, kTol) << "pure yaw is upright";
    EXPECT_NEAR(tiltFromVertical(rpyToQuaternion(0.0, 0.3, 1.1)), 0.3, 1e-9);
    EXPECT_NEAR(tiltFromVertical(rpyToQuaternion(0.3, 0.0, -2.0)), 0.3, 1e-9);
    // Combined roll and pitch tilt further than either alone.
    EXPECT_GT(tiltFromVertical(rpyToQuaternion(0.3, 0.3, 0.0)), 0.3);
}

TEST(SplitGroundProjection, FootprintIsGravityAlignedAndTheOffsetIsPurelyVertical)
{
    const Quaternion  q     = rpyToQuaternion(-0.05, 0.0847, 1.2);
    const GroundSplit split = splitGroundProjection(3.0, -4.0, 0.747, q, quaternionToYaw(q));

    EXPECT_DOUBLE_EQ(split.footprint.x, 3.0);
    EXPECT_DOUBLE_EQ(split.footprint.y, -4.0);
    // The body sits directly above the footprint, so inverting the footprint
    // transform leaves the height untouched and contributes nothing to x or y.
    EXPECT_DOUBLE_EQ(split.child_z, 0.747);
    EXPECT_NEAR(tiltFromVertical(yawToQuaternion(split.footprint.yaw)), 0.0, kTol);
    // The residual carries the tilt and none of the heading.
    EXPECT_NEAR(tiltFromVertical(split.tilt), tiltFromVertical(q), 1e-9);
    EXPECT_NEAR(quaternionToYaw(split.tilt), 0.0, 1e-9);
}

TEST(SplitGroundProjection, RecomposesToTheOriginalPose)
{
    for (int ir = 0; - 0.4 + ir * 0.19 <= 0.4; ++ir)
    {
        const double roll = -0.4 + ir * 0.19;
        for (int ip = 0; - 0.4 + ip * 0.19 <= 0.4; ++ip)
        {
            const double pitch = -0.4 + ip * 0.19;
            for (int iy = 0; - 3.0 + iy * 0.91 < 3.0; ++iy)
            {
                const double      yaw = -3.0 + iy * 0.91;
                const Quaternion  q   = rpyToQuaternion(roll, pitch, yaw);
                const GroundSplit split =
                    splitGroundProjection(1.5, 2.5, 0.75, q, quaternionToYaw(q));
                const Quaternion recomposed =
                    multiply(yawToQuaternion(split.footprint.yaw), split.tilt);

                // Sign is not observable: q and -q are the same rotation.
                const double sign = (recomposed.w * q.w < 0.0) ? -1.0 : 1.0;
                EXPECT_NEAR(sign * recomposed.w, q.w, 1e-9) << roll << "," << pitch << "," << yaw;
                EXPECT_NEAR(sign * recomposed.x, q.x, 1e-9) << roll << "," << pitch << "," << yaw;
                EXPECT_NEAR(sign * recomposed.y, q.y, 1e-9) << roll << "," << pitch << "," << yaw;
                EXPECT_NEAR(sign * recomposed.z, q.z, 1e-9) << roll << "," << pitch << "," << yaw;
            }
        }
    }
}

TEST(SplitGroundProjection, HonoursAHeldHeading)
{
    // Mid-fall the node keeps the last well-conditioned yaw rather than the
    // current one, so the split has to project about what it is given, not about
    // the quaternion.
    const Quaternion  q     = rpyToQuaternion(0.0, 0.2, 1.0);
    const GroundSplit split = splitGroundProjection(0.0, 0.0, 0.5, q, 0.25);
    EXPECT_DOUBLE_EQ(split.footprint.yaw, 0.25);
    EXPECT_NEAR(quaternionToYaw(split.tilt), wrapAngle(1.0 - 0.25), 1e-9)
        << "the heading the footprint did not take stays in the residual";
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

// --- Pose composition, which is what turns a LiDAR odometry frame into odom ------------------

namespace
{
Pose3d makePose(double x, double y, double z, double roll, double pitch, double yaw)
{
    return Pose3d{ x, y, z, rpyToQuaternion(roll, pitch, yaw) };
}

void expectSamePose(const Pose3d& actual, const Pose3d& expected, double tolerance = 1e-9)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
    // Sign-insensitive: q and -q are the same rotation, and composePose is free to return
    // either. Comparing components directly makes this test fail on an equivalent answer.
    const double dot = std::abs(
        actual.q.x * expected.q.x + actual.q.y * expected.q.y + actual.q.z * expected.q.z +
        actual.q.w * expected.q.w);
    EXPECT_NEAR(dot, 1.0, tolerance);
}
}  // namespace

TEST(ComposePose, IdentityOnEitherSideChangesNothing)
{
    const Pose3d identity;
    const Pose3d pose = makePose(1.0, -2.0, 0.5, 0.1, -0.2, 0.7);
    expectSamePose(composePose(identity, pose), pose);
    expectSamePose(composePose(pose, identity), pose);
}

TEST(ComposePose, RotatesTheSecondTranslationByTheFirstRotation)
{
    // A quarter turn about +z, then one metre along the child's +x. The child's forward is
    // the parent's +y, so the result has to land there rather than at (1, 0).
    const Pose3d turn  = makePose(0.0, 0.0, 0.0, 0.0, 0.0, M_PI_2);
    const Pose3d ahead = makePose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    expectSamePose(composePose(turn, ahead), makePose(0.0, 1.0, 0.0, 0.0, 0.0, M_PI_2));
}

TEST(InvertPose, ComposesWithItsOwnInverseToIdentity)
{
    for (const Pose3d& pose : { makePose(1.0, -2.0, 0.5, 0.1, -0.2, 0.7),
                                makePose(0.0, 0.0, 0.0, 0.0, 0.0, -3.0),
                                makePose(-4.5, 0.25, -1.0, 1.2, 0.4, 2.9) })
    {
        expectSamePose(composePose(pose, invertPose(pose)), Pose3d{});
        expectSamePose(composePose(invertPose(pose), pose), Pose3d{});
    }
}

TEST(ComposePose, ReferencingAPoseToAStartPoseGivesRelativeMotion)
{
    // The latch, in miniature: given where the LiDAR said the robot started and where it says
    // it is now, odom must report the motion between them expressed in the start frame.
    const Pose3d start = makePose(0.4, -0.2, 0.05, 0.0, 0.0, 0.6);
    // One metre along the heading it started with.
    const Pose3d now = makePose(0.4 + std::cos(0.6), -0.2 + std::sin(0.6), 0.05, 0.0, 0.0, 0.6);

    const Pose3d relative = composePose(invertPose(start), now);
    EXPECT_NEAR(relative.x, 1.0, 1e-9) << "one metre straight ahead";
    EXPECT_NEAR(relative.y, 0.0, 1e-9);
    EXPECT_NEAR(quaternionToYaw(relative.q), 0.0, 1e-9) << "driving straight is not turning";
}

TEST(IsUsablePose, AcceptsAnOrdinaryPose)
{
    EXPECT_TRUE(isUsablePose(makePose(1.0, -2.0, 0.5, 0.1, -0.2, 0.7)));
    EXPECT_TRUE(isUsablePose(Pose3d{}));
}

TEST(IsUsablePose, RejectsWhatADivergedScanMatchProduces)
{
    // FAST-LIO reports NaN rather than failing when its filter diverges. tf2 would normalise
    // the result to NaN and drop the transform without naming a source, and at the origin
    // latch a single one of these would be permanent.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    Pose3d bad_x = makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    bad_x.x      = nan;
    EXPECT_FALSE(isUsablePose(bad_x));

    Pose3d bad_z = makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    bad_z.z      = inf;
    EXPECT_FALSE(isUsablePose(bad_z));

    Pose3d bad_q = makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    bad_q.q      = Quaternion{ nan, 0.0, 0.0, 1.0 };
    EXPECT_FALSE(isUsablePose(bad_q));
}

TEST(IsUsablePose, RejectsAnAllZeroQuaternion)
{
    // The default-constructed geometry_msgs quaternion, which is what an unfilled pose carries.
    Pose3d pose = makePose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);
    pose.q      = Quaternion{ 0.0, 0.0, 0.0, 0.0 };
    EXPECT_FALSE(isUsablePose(pose));
}

TEST(IsUsablePose, ToleratesAnUnnormalisedButRecoverableQuaternion)
{
    Pose3d pose = makePose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    pose.q      = Quaternion{ 0.0, 0.0, 0.0, 2.0 };
    EXPECT_TRUE(isUsablePose(pose)) << "scaled, not degenerate";
}

TEST(ComposeAttitude, RoundTripsWithSplitGroundProjection)
{
    for (const Quaternion& q : { yawToQuaternion(0.7),
                                 makePose(0, 0, 0, 0.2, -0.15, 1.1).q,
                                 makePose(0, 0, 0, -0.4, 0.3, -2.2).q })
    {
        const double     yaw   = quaternionToYaw(q);
        const auto       split = splitGroundProjection(0.0, 0.0, 0.0, q, yaw);
        const Quaternion back  = composeAttitude(yaw, split.tilt);
        // Sign is free in a quaternion, so compare the rotations rather than the components.
        EXPECT_NEAR(quaternionToYaw(back), yaw, 1e-9);
        EXPECT_NEAR(tiltFromVertical(back), tiltFromVertical(q), 1e-9);
    }
}

TEST(ComposeAttitude, TakesHeadingFromOneSourceAndTiltFromAnother)
{
    // What the fast_lio source does: the LiDAR's heading, the IMU's gravity.
    const Quaternion lidar = makePose(0, 0, 0, 0.05, 0.05, 1.3).q;   // heading, plus drifted tilt
    const Quaternion imu   = makePose(0, 0, 0, 0.10, -0.02, 0.4).q;  // true gravity, other heading

    const Quaternion imu_tilt =
        splitGroundProjection(0.0, 0.0, 0.0, imu, quaternionToYaw(imu)).tilt;
    const Quaternion out = composeAttitude(quaternionToYaw(lidar), imu_tilt);

    EXPECT_NEAR(quaternionToYaw(out), quaternionToYaw(lidar), 1e-9) << "heading from the lidar";
    EXPECT_NEAR(tiltFromVertical(out), tiltFromVertical(imu), 1e-9) << "gravity from the IMU";
}

TEST(Slerp, EndpointsAndShortestArc)
{
    const Quaternion a = yawToQuaternion(0.0);
    const Quaternion b = yawToQuaternion(1.0);
    EXPECT_NEAR(quaternionToYaw(slerp(a, b, 0.0)), 0.0, 1e-9);
    EXPECT_NEAR(quaternionToYaw(slerp(a, b, 1.0)), 1.0, 1e-9);
    EXPECT_NEAR(quaternionToYaw(slerp(a, b, 0.5)), 0.5, 1e-9);

    // Negated quaternion is the same rotation: the interpolation must not take the long way.
    const Quaternion b_flipped{ -b.x, -b.y, -b.z, -b.w };
    EXPECT_NEAR(quaternionToYaw(slerp(a, b_flipped, 0.5)), 0.5, 1e-9);
}

TEST(Slerp, ClampsOutsideTheUnitInterval)
{
    const Quaternion a = yawToQuaternion(0.0);
    const Quaternion b = yawToQuaternion(0.8);
    EXPECT_NEAR(quaternionToYaw(slerp(a, b, -5.0)), 0.0, 1e-9);
    EXPECT_NEAR(quaternionToYaw(slerp(a, b, 5.0)), 0.8, 1e-9);
}

TEST(Slerp, ConvergesOnRepeatedApplication)
{
    // How the tilt correction is actually used: a small step per sample toward the error.
    const Quaternion target = makePose(0, 0, 0, 0.03, -0.02, 0.0).q;
    Quaternion       state;
    for (int i = 0; i < 400; ++i)
    {
        state = slerp(state, target, 0.05);
    }
    EXPECT_NEAR(tiltFromVertical(state), tiltFromVertical(target), 1e-6);
}

TEST(InvertRotation, ComposesToIdentity)
{
    const Quaternion q = makePose(0, 0, 0, 0.2, -0.3, 1.1).q;
    const Quaternion r = composeRotation(q, invertRotation(q));
    EXPECT_NEAR(std::abs(r.w), 1.0, 1e-9);
    EXPECT_NEAR(tiltFromVertical(r), 0.0, 1e-9);
}
