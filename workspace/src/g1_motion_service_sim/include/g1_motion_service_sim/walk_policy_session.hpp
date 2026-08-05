#ifndef G1_BRINGUP__WALK_POLICY_SESSION_HPP_
#define G1_BRINGUP__WALK_POLICY_SESSION_HPP_

/**
 * @file walk_policy_session.hpp
 * @brief ONNX Runtime session wrapper for the sim walking policy.
 */

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>

#include "g1_motion_service_sim/walk_policy.hpp"

namespace g1_motion_service_sim
{

/**
 * @brief Runs the walking policy's ONNX graph on the CPU.
 *
 * Single-threaded on purpose (intra_op = inter_op = 1): inference runs on the
 * node's own executor thread from a 50 Hz timer, and letting ORT spawn a pool
 * would put unmanaged threads inside a node whose whole contract is that one
 * thread touches its state.
 *
 * The graph normalises internally (Sub(mean) then Div(std) precede the first
 * Gemm), so run() takes the raw observation from assembleObservation()
 * unchanged -- see that function's note on why the config's obs_mean/obs_std
 * must not be applied here.
 */
class WalkPolicySession
{
public:
    /**
     * @brief Loads the graph and warms it up.
     *
     * The weights live in an external data file (walker.onnx.data) that ONNX
     * Runtime resolves relative to `model_path`, so both files must sit in the
     * same directory.
     *
     * @param model_path  Filesystem path to walker.onnx.
     * @throws Ort::Exception If the model is missing, malformed, or its external data is absent.
     * @throws std::runtime_error If the graph's input/output shapes are not 99 -> 29.
     */
    explicit WalkPolicySession(const std::string& model_path);

    /**
     * @brief Runs one inference.
     *
     * @param observation  Raw 99-element observation.
     * @return The 29 raw policy actions.
     */
    std::array<float, kActionDim> run(const std::array<float, kObsDim>& observation);

    /// Seconds the warm-up inference took, for the startup budget log.
    double warmupSeconds() const { return warmup_s_; }

private:
    Ort::Env                      env_;
    Ort::SessionOptions           options_;
    std::unique_ptr<Ort::Session> session_;
    Ort::MemoryInfo               memory_info_;
    std::string                   input_name_;
    std::string                   output_name_;
    double                        warmup_s_{ 0.0 };

    /// Output storage bound once at construction. The Run() overload that returns its outputs
    /// allocates a std::vector every call, which this path cannot afford at 50 Hz.
    std::array<float, kActionDim> output_buffer_{};
    Ort::Value                    output_tensor_{ nullptr };
};

}  // namespace g1_motion_service_sim

#endif  // G1_BRINGUP__WALK_POLICY_SESSION_HPP_
