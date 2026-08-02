/**
 * @file walk_policy_session.cpp
 * @brief ONNX Runtime session wrapper for the sim walking policy.
 */
#include "g1_bringup/walk_policy_session.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

namespace g1_bringup
{

namespace
{
/// Rejects a graph whose shape isn't the 99 -> 29 contract, so a swapped-in policy fails at
/// construction rather than by silently reading garbage off the end of a tensor.
void checkShape(const std::vector<std::int64_t>& shape, std::size_t expected, const char* what)
{
    // Leading batch dim is -1 (dynamic) or 1; the feature count is the last entry either way.
    if (shape.empty() || static_cast<std::size_t>(shape.back()) != expected)
    {
        throw std::runtime_error(
            std::string("walking policy ") + what + " must be " + std::to_string(expected) +
            " elements wide");
    }
}
}  // namespace

WalkPolicySession::WalkPolicySession(const std::string& model_path)
  : env_(ORT_LOGGING_LEVEL_WARNING, "g1_walk_policy")
  , memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    options_.SetIntraOpNumThreads(1);
    options_.SetInterOpNumThreads(1);
    options_.SetExecutionMode(ORT_SEQUENTIAL);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options_);

    if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1)
    {
        throw std::runtime_error("walking policy must have exactly one input and one output");
    }

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_  = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

    checkShape(
        session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
        kObsDim,
        "input");
    checkShape(
        session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape(),
        kActionDim,
        "output");

    // The first inference builds ORT's execution plan and is far slower than steady state; do it
    // here so the 50 Hz timer's first real tick isn't the one that pays for it.
    const auto start = std::chrono::steady_clock::now();
    run({});
    warmup_s_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

std::array<float, kActionDim> WalkPolicySession::run(const std::array<float, kObsDim>& observation)
{
    static constexpr std::array<std::int64_t, 2> kInputShape{ 1,
                                                              static_cast<std::int64_t>(kObsDim) };

    // const_cast because CreateTensor wraps the caller's buffer without taking ownership; ORT does
    // not write through this pointer for an input tensor.
    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info_,
        const_cast<float*>(observation.data()),
        observation.size(),
        kInputShape.data(),
        kInputShape.size());

    const char* input_names[]  = { input_name_.c_str() };
    const char* output_names[] = { output_name_.c_str() };

    auto outputs =
        session_->Run(Ort::RunOptions{ nullptr }, input_names, &input_tensor, 1, output_names, 1);

    std::array<float, kActionDim> action{};
    const float*                  data = outputs.front().GetTensorData<float>();
    std::copy(data, data + kActionDim, action.begin());
    return action;
}

}  // namespace g1_bringup
