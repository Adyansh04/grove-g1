/**
 * @file test_loco_fsm.cpp
 * @brief Unit tests for the sim-only LocoClient FSM legality table: every legal SET_FSM_ID edge,
 * every illegal one, and SET_VELOCITY's own Start-only gate.
 */
#include <gmock/gmock.h>

#include <algorithm>
#include <array>
#include <utility>

#include "g1_bringup/loco_fsm.hpp"

namespace g1_bringup
{
namespace
{

// Squat/Sit/ZeroTorque have no named constant in loco_fsm.hpp itself (see its header comment) --
// local literals here, for the same reason nothing legitimate ever needs to reference them.
constexpr int kFsmZeroTorque = 0;
constexpr int kFsmSquat      = 2;
constexpr int kFsmSit        = 3;

constexpr std::array<int, 6> kAllFsmIds{ kFsmZeroTorque, kFsmDamp,    kFsmSquat,
                                         kFsmSit,        kFsmStandUp, kFsmStart };

// -------------------------------------------------------------------------
// Every legal SET_FSM_ID edge -> success and the requested new_state
// -------------------------------------------------------------------------

TEST(ApplySetFsmId, DampToStandUpIsLegal)
{
    const auto result = applySetFsmId(kFsmDamp, kFsmStandUp);
    EXPECT_EQ(result.new_state, kFsmStandUp);
    EXPECT_EQ(result.error_code, kLocoFsmCodeSuccess);
}

TEST(ApplySetFsmId, StandUpToStartIsLegal)
{
    const auto result = applySetFsmId(kFsmStandUp, kFsmStart);
    EXPECT_EQ(result.new_state, kFsmStart);
    EXPECT_EQ(result.error_code, kLocoFsmCodeSuccess);
}

TEST(ApplySetFsmId, StartToStandUpIsLegal)
{
    const auto result = applySetFsmId(kFsmStart, kFsmStandUp);
    EXPECT_EQ(result.new_state, kFsmStandUp);
    EXPECT_EQ(result.error_code, kLocoFsmCodeSuccess);
}

TEST(ApplySetFsmId, StartToDampIsLegal)
{
    const auto result = applySetFsmId(kFsmStart, kFsmDamp);
    EXPECT_EQ(result.new_state, kFsmDamp);
    EXPECT_EQ(result.error_code, kLocoFsmCodeSuccess);
}

TEST(ApplySetFsmId, StandUpToDampIsLegal)
{
    const auto result = applySetFsmId(kFsmStandUp, kFsmDamp);
    EXPECT_EQ(result.new_state, kFsmDamp);
    EXPECT_EQ(result.error_code, kLocoFsmCodeSuccess);
}

// -------------------------------------------------------------------------
// Every other edge -- incl. anything into Squat/Sit/ZeroTorque -- rejected 7302, state unchanged
// -------------------------------------------------------------------------

TEST(ApplySetFsmId, EveryEdgeNotInTheLegalityTableIsRejected)
{
    const std::array<std::pair<int, int>, 5> legal_edges{ {
        { kFsmDamp, kFsmStandUp },
        { kFsmStandUp, kFsmStart },
        { kFsmStart, kFsmStandUp },
        { kFsmStart, kFsmDamp },
        { kFsmStandUp, kFsmDamp },
    } };
    const auto                               is_legal = [&](int current, int requested) {
        return std::any_of(legal_edges.begin(), legal_edges.end(), [&](const auto& edge) {
            return edge.first == current && edge.second == requested;
        });
    };

    for (const int current : kAllFsmIds)
    {
        for (const int requested : kAllFsmIds)
        {
            if (is_legal(current, requested))
            {
                continue;
            }
            const auto result = applySetFsmId(current, requested);
            EXPECT_EQ(result.error_code, kLocoFsmCodeInvalidFsmId)
                << "current=" << current << " requested=" << requested;
            EXPECT_EQ(result.new_state, current)
                << "rejected transition changed state; current=" << current
                << " requested=" << requested;
        }
    }
}

// -------------------------------------------------------------------------
// SET_VELOCITY: legal only from Start
// -------------------------------------------------------------------------

TEST(CheckVelocityAllowed, SucceedsInStart)
{
    EXPECT_EQ(checkVelocityAllowed(kFsmStart), kLocoFsmCodeSuccess);
}

TEST(CheckVelocityAllowed, RejectedOutsideStart)
{
    for (const int current : { kFsmZeroTorque, kFsmDamp, kFsmSquat, kFsmSit, kFsmStandUp })
    {
        EXPECT_EQ(checkVelocityAllowed(current), kLocoFsmCodeLocoStateNotAvailable)
            << "current=" << current;
    }
}

}  // namespace
}  // namespace g1_bringup
