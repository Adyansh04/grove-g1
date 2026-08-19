/**
 * @file agile_policy.cpp
 * @brief Loads the AGILE ONNX, binds its tensors once, and runs one inference per tick.
 */

#include "g1_controllers/agile_policy.hpp"

#include <algorithm>
#include <stdexcept>

namespace g1_controllers
{
namespace
{

/// Graph input names, in the order inputs_ binds them.
constexpr std::array<const char*, 12> kInputNames{ "root_link_quat_w",
                                                   "root_ang_vel_b",
                                                   "velocity_commands",
                                                   "joint_pos",
                                                   "joint_vel",
                                                   "last_actions",
                                                   "base_ang_vel_history",
                                                   "projected_gravity_history",
                                                   "velocity_commands_history",
                                                   "controlled_joint_pos_history",
                                                   "controlled_joint_vel_history",
                                                   "actions_history" };

/// Graph output names. The seven trailing tensors are the history the graph feeds back to itself.
constexpr std::array<const char*, 10> kOutputNames{ "action_joint_pos",
                                                    "action_joint_pos_kp_gains",
                                                    "action_joint_pos_kd_gains",
                                                    "last_actions_out",
                                                    "base_ang_vel_history_out",
                                                    "projected_gravity_history_out",
                                                    "velocity_commands_history_out",
                                                    "controlled_joint_pos_history_out",
                                                    "controlled_joint_vel_history_out",
                                                    "actions_history_out" };

/// Element counts, in the name-array orders above. Inputs 5-11 and outputs 3-9 are the carried
/// state, and the two runs match element for element so feedback is one copy.
constexpr std::array<std::size_t, 12> kInputSizes{ 4, 3, 3, 29, 29, 14, 15, 15, 15, 70, 70, 70 };
constexpr std::array<std::size_t, 10> kOutputSizes{ 14, 14, 14, 14, 15, 15, 15, 70, 70, 70 };

/// Index of the first carried-state tensor in each name array.
constexpr std::size_t kFirstStateInput  = 5;
constexpr std::size_t kFirstStateOutput = 3;

constexpr std::array<std::int64_t, 2> kShape4{ 1, 4 };
constexpr std::array<std::int64_t, 2> kShape3{ 1, 3 };
constexpr std::array<std::int64_t, 2> kShape29{ 1, 29 };
constexpr std::array<std::int64_t, 2> kShape14{ 1, 14 };
constexpr std::array<std::int64_t, 3> kShape5x3{ 1, 5, 3 };
constexpr std::array<std::int64_t, 3> kShape5x14{ 1, 5, 14 };

struct TensorShape
{
    const std::int64_t* dims = nullptr;
    std::size_t         rank = 0;
};

/// Every width in this graph maps to exactly one shape, so the element count identifies it.
TensorShape shapeFor(std::size_t count)
{
    switch (count)
    {
        case 3:
            return { kShape3.data(), 2 };
        case 4:
            return { kShape4.data(), 2 };
        case 14:
            return { kShape14.data(), 2 };
        case 15:
            return { kShape5x3.data(), 3 };
        case 29:
            return { kShape29.data(), 2 };
        case 70:
            return { kShape5x14.data(), 3 };
        default:
            throw std::runtime_error("no AGILE tensor shape for width " + std::to_string(count));
    }
}

}  // namespace

const std::array<std::string, kNumObsJoints> kAgileObsJointNames{ "left_hip_pitch_joint",
                                                                  "right_hip_pitch_joint",
                                                                  "waist_yaw_joint",
                                                                  "left_hip_roll_joint",
                                                                  "right_hip_roll_joint",
                                                                  "waist_roll_joint",
                                                                  "left_hip_yaw_joint",
                                                                  "right_hip_yaw_joint",
                                                                  "waist_pitch_joint",
                                                                  "left_knee_joint",
                                                                  "right_knee_joint",
                                                                  "left_shoulder_pitch_joint",
                                                                  "right_shoulder_pitch_joint",
                                                                  "left_ankle_pitch_joint",
                                                                  "right_ankle_pitch_joint",
                                                                  "left_shoulder_roll_joint",
                                                                  "right_shoulder_roll_joint",
                                                                  "left_ankle_roll_joint",
                                                                  "right_ankle_roll_joint",
                                                                  "left_shoulder_yaw_joint",
                                                                  "right_shoulder_yaw_joint",
                                                                  "left_elbow_joint",
                                                                  "right_elbow_joint",
                                                                  "left_wrist_roll_joint",
                                                                  "right_wrist_roll_joint",
                                                                  "left_wrist_pitch_joint",
                                                                  "right_wrist_pitch_joint",
                                                                  "left_wrist_yaw_joint",
                                                                  "right_wrist_yaw_joint" };

const std::array<std::string, kNumActJoints> kAgileActionJointNames{
    "left_hip_pitch_joint",  "right_hip_pitch_joint",  "left_hip_roll_joint",
    "right_hip_roll_joint",  "waist_roll_joint",       "left_hip_yaw_joint",
    "right_hip_yaw_joint",   "waist_pitch_joint",      "left_knee_joint",
    "right_knee_joint",      "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint"
};

std::optional<std::size_t> agileObsIndex(std::string_view name) noexcept
{
    const auto* const it = std::ranges::find(kAgileObsJointNames, name);
    if (it == kAgileObsJointNames.end())
    {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(kAgileObsJointNames.begin(), it));
}

std::optional<std::size_t> agileActionIndex(std::string_view name) noexcept
{
    const auto* const it = std::ranges::find(kAgileActionJointNames, name);
    if (it == kAgileActionJointNames.end())
    {
        return std::nullopt;
    }
    return static_cast<std::size_t>(std::distance(kAgileActionJointNames.begin(), it));
}

AgilePolicy::AgilePolicy(const std::string& model_path)
  : env_(ORT_LOGGING_LEVEL_WARNING, "g1_agile_policy")
  , session_(nullptr)
  , memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    // One thread: update() is already the real-time thread, and ORT's pool would otherwise spawn
    // workers that do not inherit its scheduling.
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetInterOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, model_path.c_str(), session_options_);

    verifySignature();
    bindTensors();
}

void AgilePolicy::verifySignature()
{
    if (session_.GetInputCount() != kInputNames.size() ||
        session_.GetOutputCount() != kOutputNames.size())
    {
        throw std::runtime_error(
            "AGILE policy signature mismatch: expected " + std::to_string(kInputNames.size()) +
            " inputs and " + std::to_string(kOutputNames.size()) + " outputs, model has " +
            std::to_string(session_.GetInputCount()) + " and " +
            std::to_string(session_.GetOutputCount()));
    }

    Ort::AllocatorWithDefaultOptions allocator;
    for (std::size_t i = 0; i < kInputNames.size(); ++i)
    {
        const auto name = session_.GetInputNameAllocated(i, allocator);
        if (std::string_view(name.get()) != kInputNames.at(i))
        {
            throw std::runtime_error(
                "AGILE policy input " + std::to_string(i) + " is '" + name.get() + "', expected '" +
                kInputNames.at(i) + "'");
        }
    }
    for (std::size_t i = 0; i < kOutputNames.size(); ++i)
    {
        const auto name = session_.GetOutputNameAllocated(i, allocator);
        if (std::string_view(name.get()) != kOutputNames.at(i))
        {
            throw std::runtime_error(
                "AGILE policy output " + std::to_string(i) + " is '" + name.get() +
                "', expected '" + kOutputNames.at(i) + "'");
        }
    }
}

void AgilePolicy::bindTensors()
{
    const auto bind = [this](std::vector<Ort::Value>& into, float* data, std::size_t count) {
        const auto shape = shapeFor(count);
        into.push_back(
            Ort::Value::CreateTensor<float>(memory_info_, data, count, shape.dims, shape.rank));
    };

    inputs_.reserve(kInputNames.size());
    bind(inputs_, observation_.root_quat_wxyz.data(), kInputSizes.at(0));
    bind(inputs_, observation_.root_ang_vel_b.data(), kInputSizes.at(1));
    bind(inputs_, observation_.velocity_command.data(), kInputSizes.at(2));
    bind(inputs_, observation_.joint_position.data(), kInputSizes.at(3));
    bind(inputs_, observation_.joint_velocity.data(), kInputSizes.at(4));

    std::size_t offset = 0;
    for (std::size_t i = kFirstStateInput; i < kInputNames.size(); ++i)
    {
        bind(inputs_, state_in_.data() + offset, kInputSizes.at(i));
        offset += kInputSizes.at(i);
    }

    outputs_.reserve(kOutputNames.size());
    bind(outputs_, action_position_.data(), kOutputSizes.at(0));
    bind(outputs_, action_kp_.data(), kOutputSizes.at(1));
    bind(outputs_, action_kd_.data(), kOutputSizes.at(2));

    offset = 0;
    for (std::size_t i = kFirstStateOutput; i < kOutputNames.size(); ++i)
    {
        bind(outputs_, state_out_.data() + offset, kOutputSizes.at(i));
        offset += kOutputSizes.at(i);
    }
}

bool AgilePolicy::run(const PolicyObservation& observation, PolicyAction& action) noexcept
{
    observation_ = observation;

    try
    {
        // Outputs are bound to our own buffers, and ORT's arena reuses its scratch after the
        // first call, so a steady-state Run does not allocate.
        session_.Run(
            Ort::RunOptions{ nullptr },
            kInputNames.data(),
            inputs_.data(),
            inputs_.size(),
            kOutputNames.data(),
            outputs_.data(),
            outputs_.size());
    }
    catch (...)
    {
        // Not just Ort::Exception: ORT can throw std::bad_alloc and others, and this function is
        // noexcept on the 200 Hz thread: an escape is std::terminate, which takes down
        // controller_manager and with it rt/lowcmd, on a robot with nothing holding it up.
        return false;
    }

    // Feed the history back for the next tick. One ~1 KB copy at 50 Hz, which buys a single set of
    // bound tensors instead of alternating between two.
    state_in_ = state_out_;

    action.joint_position = action_position_;
    action.kp             = action_kp_;
    action.kd             = action_kd_;
    return true;
}

void AgilePolicy::reset() noexcept
{
    state_in_.fill(0.0F);
    state_out_.fill(0.0F);
}

}  // namespace g1_controllers
