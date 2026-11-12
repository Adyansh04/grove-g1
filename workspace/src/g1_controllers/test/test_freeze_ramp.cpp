/**
 * @file test_freeze_ramp.cpp
 * @brief The freeze controller's ramp to a target: the only motion it ever commands.
 */

#include <gmock/gmock.h>

#include "g1_controllers/g1_freeze_controller.hpp"

namespace
{

using g1_controllers::rampedPosition;

TEST(FreezeRamp, StartsFromTheActivationPose)
{
    EXPECT_DOUBLE_EQ(rampedPosition(-0.57, 0.0, 0.0, 3.0), -0.57);
}

TEST(FreezeRamp, IsHalfwayAtHalfTime)
{
    EXPECT_NEAR(rampedPosition(-0.6, 0.0, 1.5, 3.0), -0.3, 1e-12);
}

TEST(FreezeRamp, HoldsTheTargetOnceTheRampEnds)
{
    EXPECT_DOUBLE_EQ(rampedPosition(-0.57, 0.0, 3.0, 3.0), 0.0);
    EXPECT_DOUBLE_EQ(rampedPosition(-0.57, 0.0, 120.0, 3.0), 0.0);
}

TEST(FreezeRamp, WithoutATargetTheJointStaysWhereItWas)
{
    // No targets configured: the goal is the activation pose, so the ramp never moves the joint.
    EXPECT_DOUBLE_EQ(rampedPosition(0.25, 0.25, 1.0, 0.0), 0.25);
    EXPECT_DOUBLE_EQ(rampedPosition(0.25, 0.25, 1.0, 3.0), 0.25);
}

}  // namespace
