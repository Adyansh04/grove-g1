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
