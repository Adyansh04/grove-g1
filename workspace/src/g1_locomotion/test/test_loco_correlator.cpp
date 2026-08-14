/**
 * @file test_loco_correlator.cpp
 * @brief Unit tests for LocoRequestCorrelator: overlap, ordering, timeout, orphaned responses,
 * supersede(), and the max_pending bound.
 */
#include <gmock/gmock.h>

#include <chrono>
#include <string>
#include <vector>

#include "g1_locomotion/loco_api_ids.hpp"
#include "g1_locomotion/loco_request_correlator.hpp"
#include "unitree_api/msg/response.hpp"

namespace g1_locomotion
{
namespace
{

constexpr double      kRequestTimeoutS = 5.0;
constexpr std::size_t kMaxPending      = 16;

LocoRequestCorrelator
makeCorrelator(double request_timeout_s = kRequestTimeoutS, std::size_t max_pending = kMaxPending)
{
    return LocoRequestCorrelator(LocoRequestCorrelator::Config{ request_timeout_s, max_pending });
}

unitree_api::msg::Response
makeResponse(std::int64_t id, std::int32_t code, const std::string& data = "")
{
    unitree_api::msg::Response response;
    response.header.identity.id = id;
    response.header.status.code = code;
    response.data               = data;
    return response;
}

struct Outcome
{
    int          call_count{ 0 };
    std::int32_t error_code{ 0 };
    std::string  data;
};

LocoRequestCorrelator::ResponseCallback recordInto(Outcome& outcome)
{
    return [&outcome](std::int32_t error_code, const std::string& data) {
        ++outcome.call_count;
        outcome.error_code = error_code;
        outcome.data       = data;
    };
}

// -------------------------------------------------------------------------
// Overlapping in-flight and out-of-order responses
// -------------------------------------------------------------------------

TEST(LocoRequestCorrelator, TwoOverlappingRequestsBothCompleteCorrectly)
{
    auto       correlator = makeCorrelator();
    const auto now        = std::chrono::steady_clock::now();

    Outcome    first;
    Outcome    second;
    const auto request_one =
        correlator.send(kApiIdSetFsmId, "{\"data\":4}", now, recordInto(first));
    const auto request_two = correlator.send(
        kApiIdSetVelocity,
        R"({"velocity":[0,0,0],"duration":1.0})",
        now,
        recordInto(second));
    ASSERT_TRUE(request_one.has_value());
    ASSERT_TRUE(request_two.has_value());
    EXPECT_NE(request_one->header.identity.id, request_two->header.identity.id);
    EXPECT_EQ(correlator.pendingCount(), 2U);

    // Out-of-order: the second request's response arrives first.
    correlator.onResponse(makeResponse(request_two->header.identity.id, 0, "second-data"));
    EXPECT_EQ(second.call_count, 1);
    EXPECT_EQ(second.error_code, 0);
    EXPECT_EQ(second.data, "second-data");
    EXPECT_EQ(first.call_count, 0) << "the first request's callback fired early";
    EXPECT_EQ(correlator.pendingCount(), 1U);

    correlator.onResponse(makeResponse(request_one->header.identity.id, kCodeInvalidFsmId, ""));
    EXPECT_EQ(first.call_count, 1);
    EXPECT_EQ(first.error_code, kCodeInvalidFsmId);
    EXPECT_EQ(correlator.pendingCount(), 0U);

    // Each callback fired exactly once, for its own request, never swapped.
    EXPECT_EQ(second.call_count, 1);
}

// -------------------------------------------------------------------------
// sweep() -- never-arrives timeout
// -------------------------------------------------------------------------

TEST(LocoRequestCorrelator, NeverArrivingResponseTimesOutViaSweep)
{
    auto       correlator = makeCorrelator(/*request_timeout_s=*/1.0);
    const auto now        = std::chrono::steady_clock::now();

    Outcome    outcome;
    const auto request =
        correlator.send(kApiIdSetFsmId, "{\"data\":500}", now, recordInto(outcome));
    ASSERT_TRUE(request.has_value());

    correlator.sweep(now + std::chrono::milliseconds(500));
    EXPECT_EQ(outcome.call_count, 0) << "swept before its own timeout elapsed";

    correlator.sweep(now + std::chrono::milliseconds(1001));
    EXPECT_EQ(outcome.call_count, 1);
    EXPECT_EQ(outcome.error_code, kCodeTaskTimeout);
    EXPECT_EQ(correlator.pendingCount(), 0U);
}

// -------------------------------------------------------------------------
// sweep() -- reentrancy: a timeout callback may itself call send()
// -------------------------------------------------------------------------

TEST(LocoRequestCorrelator, SweepTimeoutCallbackMaySendANewRequestWithoutCorruptingIteration)
{
    // Enough entries to trigger rehash during reentrant insert.
    constexpr int kCount     = 20;
    auto          correlator = makeCorrelator(
        /*request_timeout_s=*/1.0,
        /*max_pending=*/static_cast<std::size_t>(kCount) * 2);
    const auto now = std::chrono::steady_clock::now();

    int reissued = 0;
    for (int i = 0; i < kCount; ++i)
    {
        const auto request = correlator.send(
            kApiIdSetVelocity,
            "{}",
            now,
            [&correlator, &now, &reissued](std::int32_t, const std::string&) {
                ++reissued;
                // Reentrant insert from inside sweep()'s own callback loop -- exactly the
                // precondition documented as safe on the class's sweep() declaration. The
                // request itself is not the subject here, so the discard is explicit.
                static_cast<void>(correlator.send(
                    kApiIdSetVelocity,
                    "{}",
                    now,
                    [](std::int32_t, const std::string&) {}));
            });
        ASSERT_TRUE(request.has_value());
    }
    ASSERT_EQ(correlator.pendingCount(), static_cast<std::size_t>(kCount));

    EXPECT_NO_THROW(correlator.sweep(now + std::chrono::seconds(2)));
    EXPECT_EQ(reissued, kCount) << "not every expired entry's callback ran";
    // Each of the kCount original callbacks reissued exactly one fresh (unexpired) request.
    EXPECT_EQ(correlator.pendingCount(), static_cast<std::size_t>(kCount));
}

// -------------------------------------------------------------------------
// ORPHANED RESPONSE -- arrives after sweep() already timed the entry out
// -------------------------------------------------------------------------

TEST(LocoRequestCorrelator, ResponseArrivingAfterSweepTimeoutIsDroppedSafely)
{
    auto       correlator = makeCorrelator(/*request_timeout_s=*/1.0);
    const auto now        = std::chrono::steady_clock::now();

    Outcome    outcome;
    const auto request = correlator.send(kApiIdSetVelocity, "{}", now, recordInto(outcome));
    ASSERT_TRUE(request.has_value());
    const auto id = request->header.identity.id;

    correlator.sweep(now + std::chrono::seconds(2));
    ASSERT_EQ(outcome.call_count, 1);
    ASSERT_EQ(outcome.error_code, kCodeTaskTimeout);
    ASSERT_EQ(correlator.pendingCount(), 0U);
    const auto dropped_before = correlator.droppedResponseCount();

    // The real response finally shows up late -- must not crash, must not invoke the (already
    // resolved) callback a second time, and must be counted as dropped.
    EXPECT_NO_THROW(correlator.onResponse(makeResponse(id, 0, "too-late")));
    EXPECT_EQ(outcome.call_count, 1) << "orphaned response re-invoked an already-resolved callback";
    EXPECT_EQ(correlator.droppedResponseCount(), dropped_before + 1);
    EXPECT_EQ(correlator.pendingCount(), 0U);
}

TEST(LocoRequestCorrelator, ResponseForNeverSentIdIsDroppedAndCounted)
{
    auto       correlator     = makeCorrelator();
    const auto dropped_before = correlator.droppedResponseCount();

    EXPECT_NO_THROW(correlator.onResponse(makeResponse(123456789, 0, "")));
    EXPECT_EQ(correlator.droppedResponseCount(), dropped_before + 1);
    EXPECT_EQ(correlator.pendingCount(), 0U);
}

// -------------------------------------------------------------------------
// supersede()
// -------------------------------------------------------------------------

TEST(LocoRequestCorrelator, SupersededRequestNeverInvokesItsCallback)
{
    auto       correlator = makeCorrelator(/*request_timeout_s=*/1.0);
    const auto now        = std::chrono::steady_clock::now();

    Outcome    outcome;
    const auto request = correlator.send(kApiIdSetVelocity, "{}", now, recordInto(outcome));
    ASSERT_TRUE(request.has_value());
    ASSERT_EQ(correlator.pendingCount(), 1U);

    correlator.supersede(request->header.identity.id);
    EXPECT_EQ(correlator.pendingCount(), 0U);

    // Neither a late response nor a sweep past the original deadline should do anything: the
    // entry is simply gone.
    correlator.onResponse(makeResponse(request->header.identity.id, 0, "late"));
    correlator.sweep(now + std::chrono::seconds(5));
    EXPECT_EQ(outcome.call_count, 0);
}

TEST(LocoRequestCorrelator, SupersedeOfUnknownIdIsANoOp)
{
    auto correlator = makeCorrelator();
    EXPECT_NO_THROW(correlator.supersede(999));
    EXPECT_EQ(correlator.pendingCount(), 0U);
}

// -------------------------------------------------------------------------
// max_pending bound
// -------------------------------------------------------------------------

TEST(LocoRequestCorrelator, SendRefusedOncePendingReachesMaxPending)
{
    auto       correlator = makeCorrelator(kRequestTimeoutS, /*max_pending=*/2);
    const auto now        = std::chrono::steady_clock::now();

    Outcome unused;
    EXPECT_TRUE(correlator.send(kApiIdSetFsmId, "{}", now, recordInto(unused)).has_value());
    EXPECT_TRUE(correlator.send(kApiIdSetFsmId, "{}", now, recordInto(unused)).has_value());
    EXPECT_EQ(correlator.pendingCount(), 2U);

    const auto refused = correlator.send(kApiIdSetFsmId, "{}", now, recordInto(unused));
    EXPECT_FALSE(refused.has_value());
    EXPECT_EQ(correlator.pendingCount(), 2U) << "a refused send must not be tracked";
}

}  // namespace
}  // namespace g1_locomotion
