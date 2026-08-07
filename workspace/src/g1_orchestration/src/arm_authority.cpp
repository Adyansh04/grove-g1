#include "g1_orchestration/arm_authority.hpp"

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <string>
#include <vector>

namespace g1_orchestration
{

namespace
{

using SetHardwareComponentState = controller_manager_msgs::srv::SetHardwareComponentState;
using SwitchController          = controller_manager_msgs::srv::SwitchController;

constexpr const char* kComponentService = "/controller_manager/set_hardware_component_state";
constexpr const char* kSwitchService    = "/controller_manager/switch_controller";

// Shorter than the arm's budget, on purpose: an absent hand should be reported quickly rather
// than waited out twice. Mirrors activate_arm's HAND_ACTIVATE_TIMEOUT_S.
constexpr double kHandTimeoutS = 5.0;

/// A node used only for these calls. The executor already owns the tree's node, and
/// spin_until_future_complete on a node an executor holds throws instead of waiting, so this
/// borrows nothing -- it is created for the sequence and destroyed with it.
rclcpp::Node::SharedPtr makeClientNode()
{
    return std::make_shared<rclcpp::Node>("g1_arm_authority_client");
}

/// Blocking service call on a node nothing else is spinning.
template <typename ServiceT>
typename ServiceT::Response::SharedPtr callService(
    const rclcpp::Node::SharedPtr& node, const std::string& service,
    const typename ServiceT::Request::SharedPtr& request, double timeout_s)
{
    auto client = node->create_client<ServiceT>(service);
    if (!client->wait_for_service(std::chrono::duration<double>(timeout_s)))
    {
        RCLCPP_ERROR(node->get_logger(), "service '%s' never appeared", service.c_str());
        return nullptr;
    }
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::duration<double>(timeout_s)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "call to '%s' did not return", service.c_str());
        return nullptr;
    }
    return future.get();
}

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
    request->strictness             = SwitchController::Request::BEST_EFFORT;
    request->activate_asap          = true;
    request->timeout.sec            = static_cast<int>(timeout_s);

    const auto response = callService<SwitchController>(node, kSwitchService, request, timeout_s);
    return response != nullptr && response->ok;
}

/// The leaves are trivial wrappers, so they live here rather than in their own header.
class AcquireArm : public BT::SyncActionNode
{
public:
    AcquireArm(const std::string& name, const BT::NodeConfig& config, RosContext context)
      : BT::SyncActionNode(name, config)
      , node_(std::move(context.node))
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout_s", 15.0, "Per-step service budget.") };
    }

    BT::NodeStatus tick() override
    {
        return acquireArm(node_->get_logger(), getInput<double>("timeout_s").value_or(15.0)) ?
                   BT::NodeStatus::SUCCESS :
                   BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

class ReleaseArm : public BT::SyncActionNode
{
public:
    ReleaseArm(const std::string& name, const BT::NodeConfig& config, RosContext context)
      : BT::SyncActionNode(name, config)
      , node_(std::move(context.node))
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<double>("timeout_s", 15.0, "Per-step service budget.") };
    }

    /// Always SUCCESS. A release that reported failure would fail the tree it is cleaning up
    /// after, and there is nothing a tree can do about a controller that will not deactivate.
    /// The executor releases again on its way out regardless.
    BT::NodeStatus tick() override
    {
        releaseArm(node_->get_logger(), getInput<double>("timeout_s").value_or(15.0));
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

}  // namespace

const std::vector<ControlledPart>& controlledParts()
{
    // Duplicated from g1_bringup/scripts/activate_arm, which is the other implementation of
    // this sequence. test_authority_drift reads both and fails if they diverge.
    static const std::vector<ControlledPart> parts = {
        { "G1ArmSdkSystem", "arm_trajectory_controller" },
        { "G1Dex3SystemLeft", "left_hand_controller" },
        { "G1Dex3SystemRight", "right_hand_controller" },
    };
    return parts;
}

bool acquireArm(const rclcpp::Logger& logger, double timeout_s)
{
    const rclcpp::Node::SharedPtr      node  = makeClientNode();
    const std::vector<ControlledPart>& parts = controlledParts();

    // Component before controller, always. Humble ties command-interface availability to
    // hardware component state, so switching the controller in first can fail the switch or
    // strand a controller claiming interfaces that do not exist yet.
    const ControlledPart& arm = parts.front();
    RCLCPP_INFO(logger, "acquiring %s", arm.component.c_str());
    if (!setComponentState(
            node,
            arm.component,
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
            "active",
            timeout_s) ||
        !switchController(node, { arm.controller }, {}, timeout_s))
    {
        RCLCPP_ERROR(logger, "could not acquire the arm. Is the stack up and is /lowstate flowing?");
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
    return true;
}

void releaseArm(const rclcpp::Logger& logger, double timeout_s)
{
    // Reverse of acquire: controllers first, then components. Deactivating a component while
    // its controller still claims its interfaces is the failure this order avoids.
    const rclcpp::Node::SharedPtr      node  = makeClientNode();
    const std::vector<ControlledPart>& parts = controlledParts();
    for (auto it = parts.rbegin(); it != parts.rend(); ++it)
    {
        switchController(node, {}, { it->controller }, timeout_s);
        setComponentState(
            node,
            it->component,
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
            "inactive",
            timeout_s);
    }
    RCLCPP_INFO(logger, "arm and hands released");
}

void registerAuthorityNodes(BT::BehaviorTreeFactory& factory, const RosContext& context)
{
    factory.registerBuilder<AcquireArm>(
        "AcquireArm",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<AcquireArm>(name, config, context);
        });
    factory.registerBuilder<ReleaseArm>(
        "ReleaseArm",
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ReleaseArm>(name, config, context);
        });
}

}  // namespace g1_orchestration
