/**
 * @file test_velocity_gate.cpp
 * @brief Unit tests for VelocityGate: re-issue cadence, stale/zero idling, failure-streak
 * release, cmd_vel gating, and terminal-state coverage for the authority machine.
 */
#include <gmock/gmock.h>

#include "g1_locomotion/loco_api_ids.hpp"
#include "g1_locomotion/velocity_gate.hpp"

namespace g1_locomotion
{
namespace
{

constexpr double kCmdVelTimeoutS     = 0.5;
constexpr int    kFailureStreakLimit = 3;

VelocityGate makeHeldGate()
{
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    gate.beginAcquire();
    gate.onAcquireResult(/*success=*/true);
    return gate;
}

// -------------------------------------------------------------------------
// Re-issue faster than 1 Hz keeps producing the latest command
// -------------------------------------------------------------------------

TEST(VelocityGate, ContinuousReissueKeepsReturningTheLatestCommandWhileFresh)
{
    auto gate = makeHeldGate();
    auto now  = std::chrono::steady_clock::now();
    gate.setCommand(0.2, 0.0, 0.1, now);

    // Simulate a cmd_vel publisher and a re-issue timer both running at 10 Hz (every 100 ms) for
    // 1 s -- well under cmd_vel_timeout_s (0.5 s) between samples, so every tick should keep
    // seeing a live, non-stale command.
    for (int i = 0; i < 10; ++i)
    {
        now += std::chrono::milliseconds(100);
        gate.setCommand(0.2, 0.0, 0.1, now);
        const auto intent = gate.tick(now);
        ASSERT_TRUE(intent.has_value()) << "tick " << i;
        EXPECT_DOUBLE_EQ(intent->vx, 0.2);
        EXPECT_DOUBLE_EQ(intent->vyaw, 0.1);
    }
}

TEST(VelocityGate, ReissueWithoutFreshCommandGoesStaleAfterTimeout)
{
    auto       gate = makeHeldGate();
    const auto t0   = std::chrono::steady_clock::now();
    gate.setCommand(0.2, 0.0, 0.0, t0);

    ASSERT_TRUE(gate.tick(t0 + std::chrono::milliseconds(100)).has_value());
    // No further setCommand() calls -- this command goes stale after cmd_vel_timeout_s.
    const auto stale_tick = gate.tick(t0 + std::chrono::milliseconds(600));
    ASSERT_TRUE(stale_tick.has_value());
    EXPECT_DOUBLE_EQ(stale_tick->vx, 0.0);
    EXPECT_DOUBLE_EQ(stale_tick->vy, 0.0);
    EXPECT_DOUBLE_EQ(stale_tick->vyaw, 0.0);
}

// -------------------------------------------------------------------------
// stale/zero -> exactly one zero-velocity intent, then idle
// -------------------------------------------------------------------------

TEST(VelocityGate, StaleCommandProducesExactlyOneStopThenNullopt)
{
    auto       gate = makeHeldGate();
    const auto t0   = std::chrono::steady_clock::now();
    gate.setCommand(0.2, 0.0, 0.0, t0);

    const auto stop = gate.tick(t0 + std::chrono::milliseconds(600));
    ASSERT_TRUE(stop.has_value());
    EXPECT_DOUBLE_EQ(stop->vx, 0.0);
    EXPECT_DOUBLE_EQ(stop->vy, 0.0);
    EXPECT_DOUBLE_EQ(stop->vyaw, 0.0);

    EXPECT_FALSE(gate.tick(t0 + std::chrono::milliseconds(700)).has_value());
    EXPECT_FALSE(gate.tick(t0 + std::chrono::milliseconds(800)).has_value());
}

TEST(VelocityGate, ZeroTwistProducesExactlyOneStopThenNullopt)
{
    auto       gate = makeHeldGate();
    const auto t0   = std::chrono::steady_clock::now();
    gate.setCommand(0.0, 0.0, 0.0, t0);

    ASSERT_TRUE(gate.tick(t0 + std::chrono::milliseconds(10)).has_value());
    EXPECT_FALSE(gate.tick(t0 + std::chrono::milliseconds(20)).has_value());
    EXPECT_FALSE(gate.tick(t0 + std::chrono::milliseconds(30)).has_value());
}

TEST(VelocityGate, FreshNonZeroCommandAfterAStopResumesReissuing)
{
    auto gate = makeHeldGate();
    auto now  = std::chrono::steady_clock::now();
    gate.setCommand(0.0, 0.0, 0.0, now);
    ASSERT_TRUE(gate.tick(now).has_value());   // the one stop
    ASSERT_FALSE(gate.tick(now).has_value());  // idling

    now += std::chrono::milliseconds(50);
    gate.setCommand(0.15, 0.0, 0.0, now);
    const auto resumed = gate.tick(now);
    ASSERT_TRUE(resumed.has_value());
    EXPECT_DOUBLE_EQ(resumed->vx, 0.15);
}

TEST(VelocityGate, NeverReceivedAnyCommandStillProducesExactlyOneStop)
{
    // Freshly acquired, no cmd_vel ever set -- treated the same as stale.
    auto       gate = makeHeldGate();
    const auto now  = std::chrono::steady_clock::now();
    ASSERT_TRUE(gate.tick(now).has_value());
    EXPECT_FALSE(gate.tick(now).has_value());
}

// -------------------------------------------------------------------------
// failure streak -> idle
// -------------------------------------------------------------------------

TEST(VelocityGate, FailureStreakBelowLimitStaysHeld)
{
    auto gate = makeHeldGate();
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    EXPECT_EQ(gate.authority(), LocoAuthority::kHeld);
    EXPECT_EQ(gate.failureStreak(), 2);
}

TEST(VelocityGate, FailureStreakAtLimitReleasesAuthorityAndRecordsLastErrorCode)
{
    auto gate = makeHeldGate();
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    EXPECT_EQ(gate.authority(), LocoAuthority::kReleased);
    EXPECT_EQ(gate.lastErrorCode(), kCodeLocoStateNotAvailable);
}

TEST(VelocityGate, SuccessInBetweenResetsTheStreak)
{
    auto gate = makeHeldGate();
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    gate.onVelocityResult(0);
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    gate.onVelocityResult(kCodeLocoStateNotAvailable);
    EXPECT_EQ(gate.authority(), LocoAuthority::kHeld) << "streak should have reset on the success";
    EXPECT_EQ(gate.failureStreak(), 2);
}

// -------------------------------------------------------------------------
// cmd_vel ignored outside kHeld
// -------------------------------------------------------------------------

TEST(VelocityGate, TickReturnsNulloptWhileReleased)
{
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    const auto   now = std::chrono::steady_clock::now();
    gate.setCommand(0.2, 0.0, 0.0, now);
    EXPECT_FALSE(gate.tick(now).has_value());
}

TEST(VelocityGate, TickReturnsNulloptWhileAcquiring)
{
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    gate.beginAcquire();
    const auto now = std::chrono::steady_clock::now();
    gate.setCommand(0.2, 0.0, 0.0, now);
    EXPECT_FALSE(gate.tick(now).has_value());
}

TEST(VelocityGate, TickReturnsNulloptWhileReleasing)
{
    auto gate = makeHeldGate();
    gate.beginRelease();
    const auto now = std::chrono::steady_clock::now();
    gate.setCommand(0.2, 0.0, 0.0, now);
    EXPECT_FALSE(gate.tick(now).has_value());
}

// -------------------------------------------------------------------------
// Every terminal path from kAcquiring/kReleasing lands in a defined state
// -------------------------------------------------------------------------

TEST(VelocityGate, AcquireSuccessLandsAtHeld)
{
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    gate.beginAcquire();
    ASSERT_EQ(gate.authority(), LocoAuthority::kAcquiring);
    gate.onAcquireResult(true);
    EXPECT_EQ(gate.authority(), LocoAuthority::kHeld);
}

TEST(VelocityGate, AcquireFailureLandsAtReleasedNotStuckAcquiring)
{
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    gate.beginAcquire();
    gate.onAcquireResult(false);
    EXPECT_EQ(gate.authority(), LocoAuthority::kReleased);
}

TEST(VelocityGate, ReleaseSuccessLandsAtReleased)
{
    auto gate = makeHeldGate();
    gate.beginRelease();
    ASSERT_EQ(gate.authority(), LocoAuthority::kReleasing);
    gate.onReleaseResult();
    EXPECT_EQ(gate.authority(), LocoAuthority::kReleased);
}

TEST(VelocityGate, ForceReleaseLandsAtReleasedFromHeld)
{
    auto gate = makeHeldGate();
    gate.forceRelease();
    EXPECT_EQ(gate.authority(), LocoAuthority::kReleased);
}

// -------------------------------------------------------------------------
// ignoredCommandCount: the only externally visible sign that a publisher is
// talking to a gate holding no authority
// -------------------------------------------------------------------------

TEST(VelocityGate, CountsNonZeroCommandsInEveryStateThatIsNotHeld)
{
    const auto now = std::chrono::steady_clock::now();

    VelocityGate released(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    ASSERT_EQ(released.authority(), LocoAuthority::kReleased);
    released.setCommand(0.6, 0.0, 0.0, now);
    EXPECT_EQ(released.ignoredCommandCount(), 1U);

    VelocityGate acquiring(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    acquiring.beginAcquire();
    ASSERT_EQ(acquiring.authority(), LocoAuthority::kAcquiring);
    acquiring.setCommand(0.6, 0.0, 0.0, now);
    EXPECT_EQ(acquiring.ignoredCommandCount(), 1U) << "mid-acquire the command is still dropped";

    auto releasing = makeHeldGate();
    releasing.beginRelease();
    ASSERT_EQ(releasing.authority(), LocoAuthority::kReleasing);
    releasing.setCommand(0.6, 0.0, 0.0, now);
    EXPECT_EQ(releasing.ignoredCommandCount(), 1U);
}

TEST(VelocityGate, NeverCountsWhileHeld)
{
    auto       gate = makeHeldGate();
    const auto now  = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; ++i)
    {
        gate.setCommand(0.6, 0.1, 0.3, now);
    }
    EXPECT_EQ(gate.ignoredCommandCount(), 0U)
        << "a held gate acts on commands, it does not drop them";
}

TEST(VelocityGate, NeverCountsAZeroCommand)
{
    // Nav2 and teleop both idle at zero. Counting those would make the number climb whenever
    // anything is merely idle without authority, which tells a reader nothing.
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    const auto   now = std::chrono::steady_clock::now();
    for (int i = 0; i < 10; ++i)
    {
        gate.setCommand(0.0, 0.0, 0.0, now);
    }
    EXPECT_EQ(gate.ignoredCommandCount(), 0U);

    // One non-zero axis is enough to count.
    gate.setCommand(0.0, 0.0, 0.2, now);
    EXPECT_EQ(gate.ignoredCommandCount(), 1U);
}

TEST(VelocityGate, IsMonotonicAcrossAnAcquire)
{
    // A reader asserts "stopped increasing" once authority is held. Resetting on acquire would
    // make that assertion impossible to write.
    VelocityGate gate(VelocityGate::Config{ kCmdVelTimeoutS, kFailureStreakLimit });
    const auto   now = std::chrono::steady_clock::now();
    gate.setCommand(0.6, 0.0, 0.0, now);
    gate.setCommand(0.6, 0.0, 0.0, now);
    ASSERT_EQ(gate.ignoredCommandCount(), 2U);

    gate.beginAcquire();
    gate.onAcquireResult(/*success=*/true);
    EXPECT_EQ(gate.ignoredCommandCount(), 2U) << "acquiring must not reset the tally";

    gate.setCommand(0.6, 0.0, 0.0, now);
    EXPECT_EQ(gate.ignoredCommandCount(), 2U) << "and held commands do not add to it";

    gate.forceRelease();
    gate.setCommand(0.6, 0.0, 0.0, now);
    EXPECT_EQ(gate.ignoredCommandCount(), 3U) << "but dropping resumes after a release";
}

}  // namespace
}  // namespace g1_locomotion
