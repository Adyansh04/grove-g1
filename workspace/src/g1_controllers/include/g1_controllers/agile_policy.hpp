#ifndef G1_CONTROLLERS__AGILE_POLICY_HPP_
#define G1_CONTROLLERS__AGILE_POLICY_HPP_

/**
 * @file agile_policy.hpp
 * @brief Runs the AGILE velocity policy: observation packing, inference, history feedback.
 *
 * Wraps the shipped end-to-end ONNX, which carries its own normalisation, action scaling and
 * history buffers. Callers supply state in the policy's own joint orderings and get absolute
 * joint targets with the gains to hold them.
 */

#include <onnxruntime_cxx_api.h>

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace g1_controllers
{

/// Width of the policy's joint_pos / joint_vel observations: every body motor, arms included.
inline constexpr std::size_t kNumObsJoints = 29;
/// Width of the policy's action: 12 legs plus waist roll and pitch. Arms stay free for MoveIt.
inline constexpr std::size_t kNumActJoints = 14;
/// Ticks of history the policy carries internally.
inline constexpr std::size_t kHistoryLength = 5;

/// Policy tick rate and the controller_manager rate it decimates from, both fixed at training.
inline constexpr double kPolicyRateHz     = 50.0;
inline constexpr int    kPolicyDecimation = 4;

/// Joint ordering of the 29-wide observations. The trainer's breadth-first order, not SDK
/// index order, so it cannot be reused from the hardware component's table.
extern const std::array<std::string, kNumObsJoints> kAgileObsJointNames;

/// Joint ordering of the 14-wide action and gain outputs. Differs again from the observation order.
extern const std::array<std::string, kNumActJoints> kAgileActionJointNames;

/**
 * @brief Position in the policy's observation ordering.
 *
 * @param name Joint name to look up.
 * @return Index into a 29-wide observation, or nullopt if the policy does not observe that joint.
 */
[[nodiscard]] std::optional<std::size_t> agileObsIndex(std::string_view name) noexcept;

/**
 * @brief Position in the policy's action ordering.
 *
 * @param name Joint name to look up.
 * @return Index into a 14-wide action, or nullopt if the policy does not command that joint.
 */
[[nodiscard]] std::optional<std::size_t> agileActionIndex(std::string_view name) noexcept;

/**
 * @brief One tick of robot state, already in the policy's orderings.
 */
struct PolicyObservation
{
    /// Pelvis orientation in world, w first. The ros2_control IMU interface is x,y,z,w.
    std::array<float, 4> root_quat_wxyz{ 1.0F, 0.0F, 0.0F, 0.0F };
    /// Pelvis angular velocity in the body frame, straight off the gyro.
    std::array<float, 3> root_ang_vel_b{};
    /// Commanded v_x, v_y, w_z.
    std::array<float, 3> velocity_command{};

    std::array<float, kNumObsJoints> joint_position{};
    std::array<float, kNumObsJoints> joint_velocity{};
};

/**
 * @brief What the policy returns for its 14 joints: targets, and the gains to hold them.
 */
struct PolicyAction
{
    /// Absolute radians, not a scaled action: the graph applies its own scale and offset.
    std::array<float, kNumActJoints> joint_position{};
    std::array<float, kNumActJoints> kp{};
    std::array<float, kNumActJoints> kd{};
};

/**
 * @brief One loaded AGILE policy session.
 *
 * Every buffer is allocated at construction, so run() neither allocates nor throws and is safe
 * inside the controller update loop.
 */
class AgilePolicy
{
public:
    /**
     * @brief Loads the ONNX and checks its signature against the contract above.
     *
     * @param model_path Filesystem path to the policy ONNX.
     * @throws Ort::Exception if the model is unreadable, or std::runtime_error if its inputs and
     *         outputs do not match the expected names and widths.
     */
    explicit AgilePolicy(const std::string& model_path);

    /**
     * @brief Neither copyable nor movable.
     *
     * The bound tensors point into this object, so it must not be relocated after construction.
     */
    AgilePolicy(const AgilePolicy&)            = delete;
    AgilePolicy& operator=(const AgilePolicy&) = delete;
    AgilePolicy(AgilePolicy&&)                 = delete;
    AgilePolicy& operator=(AgilePolicy&&)      = delete;
    ~AgilePolicy()                             = default;

    /**
     * @brief Runs one inference and advances the internal history.
     *
     * @param observation State for this tick, in the policy's orderings.
     * @param action      Filled with the policy's joint targets and gains; untouched on failure.
     * @return false if inference failed, which the caller must escalate rather than ignore.
     */
    [[nodiscard]] bool run(const PolicyObservation& observation, PolicyAction& action) noexcept;

    /**
     * @brief Zeroes the carried history so the next run() starts as if freshly activated.
     */
    void reset() noexcept;

private:
    static constexpr std::size_t kHist3  = kHistoryLength * 3;
    static constexpr std::size_t kHist14 = kHistoryLength * kNumActJoints;
    /// last_actions plus the six history terms, the tensors the graph feeds back to itself.
    static constexpr std::size_t kStateFloats = kNumActJoints + (3 * kHist3) + (3 * kHist14);

    /**
     * @brief Checks the loaded graph against the IO this class binds.
     *
     * @throws std::runtime_error if the model's IO names or counts are not the ones bound below.
     */
    void verifySignature();

    /**
     * @brief Binds the input and output tensors to this object's own buffers.
     */
    void bindTensors();

    Ort::Env            env_;
    Ort::SessionOptions session_options_;
    Ort::Session        session_;
    Ort::MemoryInfo     memory_info_;

    /// Live observation, copied in by run() so the bound tensors never move.
    PolicyObservation observation_{};
    /// Carried policy state, in the graph's input order.
    std::array<float, kStateFloats> state_in_{};
    std::array<float, kStateFloats> state_out_{};

    std::array<float, kNumActJoints> action_position_{};
    std::array<float, kNumActJoints> action_kp_{};
    std::array<float, kNumActJoints> action_kd_{};

    std::vector<Ort::Value> inputs_;
    std::vector<Ort::Value> outputs_;
};

}  // namespace g1_controllers

#endif  // G1_CONTROLLERS__AGILE_POLICY_HPP_
