#include "g1_orchestration/arm_authority.hpp"

#include <chrono>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

namespace
{

using SetHardwareComponentState = controller_manager_msgs::srv::SetHardwareComponentState;
using SwitchController          = controller_manager_msgs::srv::SwitchController;

constexpr const char* kComponentService  = "/controller_manager/set_hardware_component_state";
constexpr const char* kSwitchService     = "/controller_manager/switch_controller";
constexpr const char* kControlStackParam = "control_stack";

// Shorter than the arm's budget, on purpose: an absent hand should be reported quickly rather
// than waited out twice. Mirrors activate_arm's HAND_ACTIVATE_TIMEOUT_S.
constexpr double kHandTimeoutS = 5.0;

/// How long the arm needs after its controller activates before it can be commanded. Not tuning:
/// the measured drift 176 ms after the switch was 0.051 rad on the elbow, against MoveIt's
/// allowed_start_tolerance of 0.05.
constexpr double kAcquireSettleS = 3.0;

bool setComponentState(
    const rclcpp::Node::SharedPtr& node, const std::string& component, uint8_t state_id,
    const std::string& label, double timeout_s)
{
    auto request                = std::make_shared<SetHardwareComponentState::Request>();
    request->name               = component;
    request->target_state.id    = state_id;
    request->target_state.label = label;

    const auto response =
        callService<SetHardwareComponentState>(node, kComponentService, request, timeout_s);
    return response != nullptr && response->ok;
}

/// A one-or-zero element list, for the parts whose `component` or `displaces` is empty.
std::vector<std::string> nonEmpty(const std::string& name)
{
    return name.empty() ? std::vector<std::string>{} : std::vector<std::string>{ name };
}

bool switchController(
    const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate, double timeout_s)
{
    auto request                    = std::make_shared<SwitchController::Request>();
    request->activate_controllers   = activate;
    request->deactivate_controllers = deactivate;
    // BEST_EFFORT, not STRICT, and the difference matters here. This runs as a tree leaf, so
    // it has to be idempotent: the arm is very often already acquired (activate_arm:=true does
    // it at bring-up, and a retried or re-run mission re-enters this), and STRICT reports
    // activating an already-active controller as a failure. The bring-up script uses STRICT
    // because it IS the fresh acquire; a mission cannot assume it is first.
    request->strictness    = SwitchController::Request::BEST_EFFORT;
    request->activate_asap = true;
    request->timeout.sec   = static_cast<int>(timeout_s);

    const auto response = callService<SwitchController>(node, kSwitchService, request, timeout_s);
    return response != nullptr && response->ok;
}

}  // namespace

ControlStack controlStackFromString(std::string_view name)
{
    if (name == "arm_sdk")
    {
        return ControlStack::kArmSdk;
    }
    if (name == "lowcmd")
    {
        return ControlStack::kLowCmd;
    }
    throw std::invalid_argument(
        "control_stack must be 'arm_sdk' or 'lowcmd', got '" + std::string(name) + "'");
}

ControlStack controlStackOf(const rclcpp::Node::SharedPtr& node)
{
    if (!node->has_parameter(kControlStackParam))
    {
        node->declare_parameter<std::string>(kControlStackParam, "arm_sdk");
    }
    return controlStackFromString(node->get_parameter(kControlStackParam).as_string());
}

const std::vector<ControlledPart>& controlledParts(ControlStack stack)
{
    // Duplicated from g1_bringup/scripts/activate_arm, which is the other implementation of
    // this sequence. test_authority_drift reads both and fails if they diverge.
    //
    // The hands are identical on both stacks: a Dex3 is its own device on its own topics, so
    // which interface owns the body motors never reaches it.
    static const std::vector<ControlledPart> arm_sdk_parts = {
        { "G1ArmSdkSystem", "arm_trajectory_controller", "" },
        { "G1Dex3SystemLeft", "left_hand_controller", "" },
        { "G1Dex3SystemRight", "right_hand_controller", "" },
    };
    // No component to activate for the arm: G1LowCmdSystem owns all 29 motors and is active
    // from bring-up, holding the arms through arm_freeze_controller. Acquiring is therefore
    // one switch that trades the freeze for the trajectory controller, which ros2_control
    // applies atomically -- and it has to be one switch, because the two claim the same joints
    // and a joint this component sees unclaimed is a joint it leaves unpowered.
    static const std::vector<ControlledPart> lowcmd_parts = {
        { "", "arm_trajectory_controller", "arm_freeze_controller" },
        { "G1Dex3SystemLeft", "left_hand_controller", "" },
        { "G1Dex3SystemRight", "right_hand_controller", "" },
    };
    return stack == ControlStack::kLowCmd ? lowcmd_parts : arm_sdk_parts;
}

bool acquireArm(const rclcpp::Logger& logger, double timeout_s, ControlStack stack)
{
    const rclcpp::Node::SharedPtr      node  = makeClientNode("g1_arm_authority_client");
    const std::vector<ControlledPart>& parts = controlledParts(stack);

    // Component before controller, always. Command-interface availability is tied to hardware
    // component state, so switching the controller in first can fail the switch or strand a
    // controller claiming interfaces that do not exist yet.
    const ControlledPart& arm = parts.front();
    RCLCPP_INFO(logger, "acquiring %s", arm.controller.c_str());

    // Nothing to activate when the component is already up for other reasons, which is the
    // lowcmd case: there the switch below is the whole acquire.
    const bool component_ready =
        arm.component.empty() || setComponentState(
                                     node,
                                     arm.component,
                                     lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                     "active",
                                     timeout_s);

    if (!component_ready ||
        !switchController(node, { arm.controller }, nonEmpty(arm.displaces), timeout_s))
    {
        RCLCPP_ERROR(logger, "could not acquire the arm. Is the control stack up?");
        return false;
    }

    for (std::size_t i = 1; i < parts.size(); ++i)
    {
        const ControlledPart& hand = parts[i];
        if (!setComponentState(
                node,
                hand.component,
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                "active",
                kHandTimeoutS) ||
            !switchController(node, { hand.controller }, {}, kHandTimeoutS))
        {
            // Best-effort, exactly as activate_arm treats it: a hand that is absent, unpowered
            // or not publishing state leaves the arm usable, and only the arm fails the whole
            // acquire.
            RCLCPP_WARN(
                logger,
                "%s did not come up; the arm is still usable but this hand will not move",
                hand.component.c_str());
        }
    }

    // Settle before reporting success. Activating the controller does not leave the arm where it
    // was: rt/arm_sdk ramps its blend weight and the joints move to the held pose over the next
    // second or two. Command a trajectory into that and MoveIt validates the plan's start state
    // against a robot that has since moved, and refuses with "start point deviates from current
    // robot state more than 0.05" -- measured on the right elbow at 0.051 rad, 176 ms after the
    // switch returned, which is the whole margin.
    //
    // The same reason g1_loco_authority has settle_after_start_s: an authority handoff is not
    // complete when the service call returns, it is complete when the thing has stopped moving.
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kAcquireSettleS)));
    return true;
}

void releaseArm(const rclcpp::Logger& logger, double timeout_s, ControlStack stack)
{
    // Reverse of acquire: controllers first, then components. Deactivating a component while
    // its controller still claims its interfaces is the failure this order avoids.
    const rclcpp::Node::SharedPtr      node  = makeClientNode("g1_arm_authority_client");
    const std::vector<ControlledPart>& parts = controlledParts(stack);
    // std::ranges::reverse_view breaks clang-tidy's Clang-14 parser against libstdc++ here.
    // NOLINTNEXTLINE(modernize-loop-convert)
    for (auto it = parts.rbegin(); it != parts.rend(); ++it)
    {
        // Whatever was displaced comes straight back in the same switch, so the joints are
        // never momentarily unowned on a stack where unowned means unpowered.
        switchController(node, nonEmpty(it->displaces), { it->controller }, timeout_s);
        if (!it->component.empty())
        {
            setComponentState(
                node,
                it->component,
                lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                "inactive",
                timeout_s);
        }
    }
    RCLCPP_INFO(logger, "arm and hands released");
}

}  // namespace g1_orchestration
