/**
 * @file arm_authority.cpp
 * @brief The controller_manager calls that take and hand back the arm and hands.
 */

#include "g1_orchestration/arm_authority.hpp"

#include <chrono>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
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

/// Shorter than the arm's budget so an absent hand is reported quickly rather than waited out
/// twice. Mirrors activate_arm's HAND_ACTIVATE_TIMEOUT_S.
constexpr double kHandTimeoutS = 5.0;

/// Settle after the switch before the arm can be commanded. Measured: 0.051 rad of elbow drift
/// 176 ms after the switch, against MoveIt's allowed_start_tolerance of 0.05.
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

bool switchController(
    const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate, double timeout_s, uint8_t strictness)
{
    auto request                    = std::make_shared<SwitchController::Request>();
    request->activate_controllers   = activate;
    request->deactivate_controllers = deactivate;
    request->strictness             = strictness;
    request->activate_asap          = true;
    request->timeout.sec            = static_cast<int>(timeout_s);

    const auto response = callService<SwitchController>(node, kSwitchService, request, timeout_s);
    return response != nullptr && response->ok;
}

// Every controller controller_manager knows, by name. Empty when it did not answer, which is
// treated the same as knowing nothing: no controller able to take the arms.
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

// Trades `outgoing` for `incoming` over the same joints, in one switch or not at all.
bool swapArmController(
    const rclcpp::Node::SharedPtr& node, const std::string& incoming, const std::string& outgoing,
    const rclcpp::Logger& logger, double timeout_s)
{
    // One listing for both, not one call each: this runs inside a tick with the interrupt
    // unchecked, and the arm's budget is 15 s.
    const std::map<std::string, std::string> states = controllerStates(node, timeout_s);
    if (states.empty())
    {
        RCLCPP_ERROR(logger, "controller_manager did not list its controllers; not switching");
        return false;
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
        return false;
    }
    if (plan.already_held)
    {
        return true;
    }
    return switchController(
        node,
        { incoming },
        plan.displace ? std::vector<std::string>{ outgoing } : std::vector<std::string>{},
        timeout_s,
        SwitchController::Request::STRICT);
}

}  // namespace

ArmSwitchPlan planArmSwitch(const std::string& incoming_state, const std::string& outgoing_state)
{
    ArmSwitchPlan plan;
    // An unknown incoming controller is the dangerous case: the switch must then ask for
    // nothing, because deactivating the holder on its own is what drops the arms.
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
    // Mirrors g1_bringup/scripts/activate_arm; test_authority_drift fails if the two diverge.
    //
    // The arm has no component to activate: the body component owns all 29 motors and is
    // already active, so acquiring is one atomic switch trading the freeze for the trajectory
    // controller. It must be one switch, because both claim the same joints and a joint this
    // component sees unclaimed is a joint it leaves unpowered.
    static const std::vector<ControlledPart> parts = {
        { "", "arm_trajectory_controller", "arm_freeze_controller" },
        { "G1Dex3SystemLeft", "left_hand_controller", "" },
        { "G1Dex3SystemRight", "right_hand_controller", "" },
    };
    return parts;
}

bool acquireArm(const rclcpp::Logger& logger, double timeout_s)
{
    const rclcpp::Node::SharedPtr      node  = makeClientNode("g1_arm_authority_client");
    const std::vector<ControlledPart>& parts = controlledParts();

    // Component before controller wherever there is one: command-interface availability is tied
    // to component state, so switching first strands a controller claiming interfaces that do
    // not exist yet.
    const ControlledPart& arm = parts.front();
    RCLCPP_INFO(logger, "acquiring %s", arm.controller.c_str());

    const bool component_ready =
        arm.component.empty() || setComponentState(
                                     node,
                                     arm.component,
                                     lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                     "active",
                                     timeout_s);

    if (!component_ready ||
        !swapArmController(node, arm.controller, arm.displaces, logger, timeout_s))
    {
        RCLCPP_ERROR(logger, "could not acquire the arm. Is the control stack up?");
        return false;
    }

    for (std::size_t i = 1; i < parts.size(); ++i)
    {
        const ControlledPart& hand = parts[i];
        // BEST_EFFORT, unlike the arm: nothing is displaced here, so there is no half of a pair
        // to apply on its own, and a hand that will not come up must leave the arm usable.
        if (!setComponentState(
                node,
                hand.component,
                lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                "active",
                kHandTimeoutS) ||
            !switchController(
                node,
                { hand.controller },
                {},
                kHandTimeoutS,
                SwitchController::Request::BEST_EFFORT))
        {
            RCLCPP_WARN(
                logger,
                "%s did not come up; the arm is still usable but this hand will not move",
                hand.component.c_str());
        }
    }

    // The switch itself moves the joints as the trajectory controller takes over at its own
    // stiffness, and MoveIt validates a plan's start state against where the robot is now.
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(kAcquireSettleS)));
    return true;
}

void releaseArm(const rclcpp::Logger& logger, double timeout_s)
{
    // Reverse of acquire: controllers first, then components. Deactivating a component while
    // its controller still claims its interfaces is the failure this order avoids.
    const rclcpp::Node::SharedPtr      node  = makeClientNode("g1_arm_authority_client");
    const std::vector<ControlledPart>& parts = controlledParts();
    // std::ranges::reverse_view breaks clang-tidy's Clang-14 parser against libstdc++ here.
    // NOLINTNEXTLINE(modernize-loop-convert)
    for (auto it = parts.rbegin(); it != parts.rend(); ++it)
    {
        if (it->displaces.empty())
        {
            switchController(
                node,
                {},
                { it->controller },
                timeout_s,
                SwitchController::Request::BEST_EFFORT);
        }
        else
        {
            // Whatever was displaced comes back in the same switch, so the joints are never
            // momentarily unowned, which here means unpowered. If it cannot come back the
            // switch is not made and this controller keeps them, still powered.
            swapArmController(node, it->displaces, it->controller, logger, timeout_s);
        }
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
