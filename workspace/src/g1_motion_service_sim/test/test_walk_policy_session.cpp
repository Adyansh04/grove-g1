/**
 * @file test_walk_policy_session.cpp
 * @brief Loads the vendored policy through ONNX Runtime and pins the properties the sim node
 * depends on: the 99 -> 29 shape contract, determinism, and that the graph normalises internally.
 */
#include <gmock/gmock.h>

#include <chrono>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "g1_motion_service_sim/walk_policy_session.hpp"

namespace g1_motion_service_sim
{
namespace
{

std::string modelPath()
{
    return ament_index_cpp::get_package_share_directory("g1_motion_service_sim") +
           "/policy/walker.onnx";
}

TEST(WalkPolicySessionTest, LoadsTheVendoredPolicyAndItsExternalWeights)
{
    // The weights live in walker.onnx.data alongside the graph; if the install rule ever ships
    // only the .onnx, construction throws here rather than at the first sim tick.
    EXPECT_NO_THROW({ WalkPolicySession session(modelPath()); });
}

TEST(WalkPolicySessionTest, MissingModelFailsAtConstruction)
{
    EXPECT_ANY_THROW({ WalkPolicySession session("/nonexistent/walker.onnx"); });
}

TEST(WalkPolicySessionTest, ReturnsOneActionPerBodyMotor)
{
    WalkPolicySession session(modelPath());
    const auto        action = session.run({});
    EXPECT_EQ(action.size(), kActionDim);
    for (std::size_t i = 0; i < action.size(); ++i)
    {
        EXPECT_TRUE(std::isfinite(action[i])) << "action[" << i << "] is not finite";
    }
}

TEST(WalkPolicySessionTest, IsDeterministicForAFixedObservation)
{
    WalkPolicySession          session(modelPath());
    std::array<float, kObsDim> obs{};
    for (std::size_t i = 0; i < obs.size(); ++i)
    {
        obs[i] = 0.01F * static_cast<float>(i);
    }
    const auto first  = session.run(obs);
    const auto second = session.run(obs);
    EXPECT_EQ(first, second);
}

TEST(WalkPolicySessionTest, MatchesTheGoldenActionForAZeroObservation)
{
    // IsDeterministicForAFixedObservation above compares two runs of the SAME build, so it
    // cannot see a change introduced by an edit to run(). These literals were captured from
    // the implementation before run() was switched to ORT's in-place Run() overload, and pin
    // the output across any future change to how inference is invoked.
    //
    // Hex float literals, and compared exactly: the point is bit-identity, and a tolerance
    // would let a real numerical change through. walker.onnx is vendored and pinned, so the
    // expected values only move if the model does.
    static constexpr std::array<float, kActionDim> kGoldenForZeros{
        0x1.36d39ep-3F,  0x1.4d967cp-2F,  -0x1.a9e434p-3F, -0x1.9bcb1ep-6F, 0x1.cebfdep-2F,
        -0x1.16305p-3F,  -0x1.e3930cp-3F, -0x1.57ba5ap-2F, 0x1.daa4a6p-5F,  -0x1.87a886p-2F,
        0x1.2936fcp-2F,  -0x1.00117cp-3F, -0x1.bdfe04p-5F, -0x1.368c24p-3F, -0x1.242b52p-5F,
        -0x1.b55166p-2F, 0x1.43c3ccp-2F,  0x1.95b334p-3F,  -0x1.83b4dcp-3F, 0x1.ab0618p-3F,
        0x1.1b57fcp-3F,  0x1.417ep-1F,    0x1.e8fb6p-8F,   -0x1.b82274p-2F, -0x1.211434p-3F,
        -0x1.268fa4p-2F, 0x1.f47108p-3F,  0x1.3149acp-1F,  0x1.b249b6p-2F
    };

    WalkPolicySession session(modelPath());
    const auto        action = session.run({});
    ASSERT_EQ(action.size(), kGoldenForZeros.size());
    for (std::size_t i = 0; i < action.size(); ++i)
    {
        EXPECT_EQ(action[i], kGoldenForZeros[i]) << "action[" << i << "] moved";
    }

    // The buffer the in-place overload writes into is reused across calls, so a second run
    // must not see the first one's values leak through.
    const auto again = session.run({});
    EXPECT_EQ(action, again);
}

TEST(WalkPolicySessionTest, NormalisationIsBakedIntoTheGraph)
{
    // A zero observation is far from the training mean, so if the graph did NOT normalise
    // internally its response to zero would be indistinguishable from its response to a small
    // perturbation. Sub(mean)/Div(std) up front amplifies that difference well past float noise.
    WalkPolicySession          session(modelPath());
    std::array<float, kObsDim> perturbed{};
    perturbed[kObsCommand] = 1.0F;  // a forward-velocity command

    const auto at_zero    = session.run({});
    const auto at_command = session.run(perturbed);
    EXPECT_NE(at_zero, at_command)
        << "a velocity command produced no change in the policy output -- the observation is "
           "probably not reaching the graph in the expected layout";
}

TEST(WalkPolicySessionTest, WarmInferenceFitsTheFiftyHertzBudget)
{
    // The policy timer runs at 50 Hz (20 ms) inside the same node that publishes /lowcmd at
    // 500 Hz (2 ms). Inference must stay well inside the shorter of the two or it stalls the
    // publish path. Generous bound: this is a regression guard, not a benchmark.
    WalkPolicySession session(modelPath());
    RecordProperty("warmup_seconds", std::to_string(session.warmupSeconds()));

    constexpr int kIterations = 200;
    const auto    start       = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i)
    {
        session.run({});
    }
    const double mean_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() /
        kIterations;

    RecordProperty("mean_inference_ms", std::to_string(mean_ms));
    EXPECT_LT(mean_ms, 1.0) << "mean inference " << mean_ms << " ms exceeds half the 2 ms "
                            << "/lowcmd period -- reconsider running it in-process";
}

}  // namespace
}  // namespace g1_motion_service_sim
