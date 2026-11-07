/**
 * @file arm_authority.cpp
 * @brief The controller_manager calls that take and hand back the arm and hands.
 */

#include "g1_orchestration/arm_authority.hpp"

#include <algorithm>
#include <chrono>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <cstdint>
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

namespace
{

using ListControllers           = controller_manager_msgs::srv::ListControllers;
using SetHardwareComponentState = controller_manager_msgs::srv::SetHardwareComponentState;
using SwitchController          = controller_manager_msgs::srv::SwitchController;

constexpr const char* kComponentService = "/controller_manager/set_hardware_component_state";
constexpr const char* kSwitchService    = "/controller_manager/switch_controller";
constexpr const char* kListService      = "/controller_manager/list_controllers";

/// The one controller_manager state that means a controller is holding its joints.
constexpr const char* kActiveState = "active";

/// How often the settle looks for a stop.
constexpr std::chrono::milliseconds kSettleSlice{ 100 };

/// What a switch leaves of the client's budget for discovery and the answer, in seconds: the
/// server gives up first, so even its refusal arrives.
constexpr int kSwitchAnswerMarginS = 2;

std::map<std::string, std::string>
controllerStates(const rclcpp::Node::SharedPtr& node, double timeout_s);

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

bool switchController(
    const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate, double timeout_s, uint8_t strictness)
{
    auto request                    = std::make_shared<SwitchController::Request>();
    request->activate_controllers   = activate;
    request->deactivate_controllers = deactivate;
    request->strictness             = strictness;
    request->activate_asap          = true;
    request->timeout.sec = std::max(1, static_cast<int>(timeout_s) - kSwitchAnswerMarginS);

    const auto response = callService<SwitchController>(node, kSwitchService, request, timeout_s);
    if (response != nullptr)
    {
        return response->ok;
    }
    // No answer is not no swap: one made after the wait would leave the arm switched untracked.
    const auto states = controllerStates(node, timeout_s);
    const auto active = [&states](const std::string& name) {
        const auto it = states.find(name);
        return it != states.end() && it->second == kActiveState;
    };
    return !states.empty() && std::ranges::all_of(activate, active) &&
           std::ranges::none_of(deactivate, active);
}

// Controller states by name; empty if controller_manager did not answer.
std::map<std::string, std::string>
controllerStates(const rclcpp::Node::SharedPtr& node, double timeout_s)
{
    std::map<std::string, std::string> states;
    const auto                         response = callService<ListControllers>(
        node,
        kListService,
        std::make_shared<ListControllers::Request>(),
        timeout_s);
    if (response == nullptr)
    {
        return states;
    }
    for (const auto& state : response->controller)
    {
        states.emplace(state.name, state.state);
    }
    return states;
}

// Trades `outgoing` for `incoming` over the same joints, in one switch or not at all: kTaken when
// made now, kAlready when found made.
Acquired swapArmController(
    const rclcpp::Node::SharedPtr& node, const std::string& incoming, const std::string& outgoing,
    const rclcpp::Logger& logger, double timeout_s)
{
    // One listing for both, to keep this blocking, uninterruptible step short.
    const std::map<std::string, std::string> states = controllerStates(node, timeout_s);
    if (states.empty())
    {
        RCLCPP_ERROR(logger, "controller_manager did not list its controllers; not switching");
        return Acquired::kFailed;
    }

    const auto state_of = [&states](const std::string& name) {
        const auto it = states.find(name);
        return it == states.end() ? std::string{} : it->second;
    };
    const ArmSwitchPlan plan = planArmSwitch(state_of(incoming), state_of(outgoing));

    if (!plan.possible)
    {
        RCLCPP_ERROR(
            logger,
            "%s cannot take the arms, so %s keeps them",
            incoming.c_str(),
            outgoing.c_str());
        return Acquired::kFailed;
    }
    if (plan.already_held)
    {
        return Acquired::kAlready;
    }
    return switchController(
               node,
               { incoming },
               plan.displace ? std::vector<std::string>{ outgoing } : std::vector<std::string>{},
               timeout_s,
               SwitchController::Request::STRICT) ?
               Acquired::kTaken :
               Acquired::kFailed;
}

}  // namespace

ArmSwitchPlan planArmSwitch(const std::string& incoming_state, const std::string& outgoing_state)
{
    ArmSwitchPlan plan;
    // Unknown incoming controller: ask for nothing; deactivating the holder alone drops the arms.
    if (incoming_state.empty())
    {
        return plan;
    }
    plan.possible     = true;
    plan.already_held = incoming_state == kActiveState;
    plan.displace     = outgoing_state == kActiveState;
    return plan;
}

const std::vector<ControlledPart>& controlledParts()
{
    // Mirrors g1_bringup/scripts/activate_arm (test_authority_drift). The arm's motors belong to
    // the always-active body component, so the arm is acquired by a controller switch alone.
    static const std::vector<ControlledPart> parts = {
        { "", "arm_trajectory_controller", "arm_freeze_controller" },
        { "G1Dex3SystemLeft", "left_hand_controller", "" },
        { "G1Dex3SystemRight", "right_hand_controller", "" },
    };
    return parts;
}

Acquired
acquireArm(const rclcpp::Logger& logger, double timeout_s, const std::function<bool()>& stopped)
{
    const rclcpp::Node::SharedPtr      node  = makeClientNode("g1_arm_authority_client");
    const std::vector<ControlledPart>& parts = controlledParts();

    // Component before controller: its command interfaces exist only while it is active.
    const ControlledPart& arm = parts.front();
    RCLCPP_INFO(logger, "acquiring %s", arm.controller.c_str());

    const bool component_ready =
        arm.component.empty() || setComponentState(
                                     node,
                                     arm.component,
                                     lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                     "active",
                                     timeout_s);

    const Acquired swapped =
        component_ready ?
            swapArmController(node, arm.controller, arm.displaces, logger, timeout_s) :
            Acquired::kFailed;
    if (swapped == Acquired::kFailed)
    {
        RCLCPP_ERROR(logger, "could not acquire the arm. Is the control stack up?");
        return Acquired::kFailed;
    }

    // Both hands in one switch, each whose component came up: an absent hand costs one wait.
    std::vector<std::string> hands;
    for (std::size_t i = 1; i < parts.size(); ++i)
    {
        const ControlledPart& hand = parts[i];
        if (setComponentState(
                node,
                hand.component,
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                "active",
                kHandTimeoutS))
        {
            hands.push_back(hand.controller);
        }
        else
        {
            RCLCPP_WARN(
                logger,
                "%s did not come up; the arm is still usable but this hand will not move",
                hand.component.c_str());
        }
    }
    // BEST_EFFORT is safe here: nothing is displaced, so there is no pair to half-apply.
    if (!hands.empty() &&
        !switchController(node, hands, {}, kHandTimeoutS, SwitchController::Request::BEST_EFFORT))
    {
        RCLCPP_WARN(logger, "the hands' controllers did not all start; a hand may not move");
    }

    // Only a switch moves the joints; a stop does not wait the settle out.
    if (swapped == Acquired::kTaken)
    {
        const auto until = std::chrono::steady_clock::now() +
                           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(kAcquireSettleS));
        while (std::chrono::steady_clock::now() < until && !(stopped && stopped()))
        {
            rclcpp::sleep_for(kSettleSlice);
        }
    }
    return swapped;
}

bool releaseArm(const rclcpp::Logger& logger, double timeout_s)
{
    const rclcpp::Node::SharedPtr      node  = makeClientNode("g1_arm_authority_client");
    const std::vector<ControlledPart>& parts = controlledParts();

    // The freeze returns in the same switch, or no switch is made: an unowned joint is unpowered.
    const ControlledPart& arm = parts.front();
    if (swapArmController(node, arm.displaces, arm.controller, logger, timeout_s) ==
        Acquired::kFailed)
    {
        RCLCPP_ERROR(
            logger,
            "the arms stay with %s: they could not be handed back to %s",
            arm.controller.c_str(),
            arm.displaces.c_str());
        return false;
    }

    bool hands_down = true;
    for (std::size_t i = parts.size() - 1; i > 0; --i)
    {
        const ControlledPart& hand    = parts[i];
        const bool            stopped = switchController(
            node,
            {},
            { hand.controller },
            timeout_s,
            SwitchController::Request::BEST_EFFORT);
        const bool inactive = setComponentState(
            node,
            hand.component,
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            "inactive",
            timeout_s);
        if (!stopped || !inactive)
        {
            RCLCPP_WARN(logger, "%s did not go down cleanly", hand.component.c_str());
            hands_down = false;
        }
    }
    RCLCPP_INFO(logger, hands_down ? "arm and hands released" : "arm released; see the hands above");
    return true;
}

}  // namespace g1_orchestration
