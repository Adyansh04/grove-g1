/**
 * @file mission_observer.cpp
 * @brief Turning tree transitions into events, the running step, and the mission's failure.
 */

#include "g1_orchestration/mission_observer.hpp"

#include <behaviortree_cpp/decorator_node.h>

#include <cctype>
#include <cstdint>
#include <format>
#include <utility>

#include "g1_orchestration/leaf_report.hpp"

namespace g1_orchestration
{

std::string stepOfPath(std::string_view path)
{
    for (std::size_t start = 0; start <= path.size();)
    {
        const std::size_t end = path.find('/', start);
        const std::string step =
            stepIdOf(path.substr(start, end == std::string_view::npos ? end : end - start));
        if (!step.empty())
        {
            return step;
        }
        if (end == std::string_view::npos)
        {
            break;
        }
        start = end + 1;
    }
    return {};
}

namespace
{

bool isStepNode(const BT::TreeNode& node)
{
    return node.registrationName() == "SubTree" && !stepIdOf(node.name()).empty();
}

/// The step a decorator wraps: a step's Timeout, Retry and ForceSuccess have no name of their own.
std::string stepBelow(const BT::TreeNode& node)
{
    const BT::TreeNode* below = &node;
    while (const auto* decorator = dynamic_cast<const BT::DecoratorNode*>(below))
    {
        below = decorator->child();
        if (below == nullptr)
        {
            return {};
        }
        if (isStepNode(*below))
        {
            return stepIdOf(below->name());
        }
    }
    return {};
}

}  // namespace

MissionObserver::MissionObserver(
    BT::TreeNode* root, rclcpp::Clock::SharedPtr clock, Level level,
    std::set<std::string> skill_leaves, Hooks hooks)
  : BT::StatusChangeLogger(root)
  , clock_(std::move(clock))
  , level_(level)
  , skill_leaves_(std::move(skill_leaves))
  , hooks_(std::move(hooks))
{}

bool MissionObserver::wanted(
    const BT::TreeNode& node, BT::NodeStatus prev, BT::NodeStatus status) const
{
    // A node going back to IDLE after it finished is a reset; only a halt (RUNNING to IDLE) is news.
    if (status == BT::NodeStatus::IDLE && prev != BT::NodeStatus::RUNNING)
    {
        return false;
    }
    return level_ == Level::kAll || isStepNode(node) ||
           skill_leaves_.contains(node.registrationName());
}

void MissionObserver::callback(
    BT::Duration /*timestamp*/, const BT::TreeNode& node, BT::NodeStatus prev, BT::NodeStatus status)
{
    const bool step_started = isStepNode(node) && status == BT::NodeStatus::RUNNING;
    const bool succeeded    = status == BT::NodeStatus::SUCCESS && prev != BT::NodeStatus::SUCCESS;
    const bool halted       = prev == BT::NodeStatus::RUNNING && status == BT::NodeStatus::IDLE;
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        const std::string&                path = node.fullPath();
        if (status == BT::NodeStatus::RUNNING)
        {
            running_.insert(path);
            halted_.erase(path);
        }
        else if (prev == BT::NodeStatus::RUNNING)
        {
            running_.erase(path);
            if (status == BT::NodeStatus::IDLE)
            {
                halted_.insert(path);
            }
        }
        if (step_started)
        {
            current_step_ = node.name();
        }
        // A leaf that failed, or a Timeout that ran out. The last of these before the tree fails
        // is what ended it: an earlier one may have been retried, or made optional. A Timeout
        // passing on its child's failure is not one.
        if (status == BT::NodeStatus::FAILURE &&
            (node.type() == BT::NodeType::ACTION || node.type() == BT::NodeType::CONDITION ||
             (node.registrationName() == "Timeout" && ranOut(node))))
        {
            failure_ = describe(node);
        }
        if (wanted(node, prev, status))
        {
            nervros_interfaces::msg::NodeEvent event;
            event.stamp           = clock_->now();
            event.uid             = node.UID();
            event.path            = path;
            event.name            = node.name();
            event.registration_id = node.registrationName();
            event.prev_status     = static_cast<std::uint8_t>(prev);
            event.status          = static_cast<std::uint8_t>(status);
            events_.push_back(std::move(event));
        }
    }
    // Outside the lock: the executor takes its own locks and publishes.
    if (step_started && hooks_.on_step)
    {
        hooks_.on_step(node.name());
    }
    if (succeeded && hooks_.on_success)
    {
        hooks_.on_success(node);
    }
    if (halted && hooks_.on_halt)
    {
        hooks_.on_halt(node);
    }
}

bool MissionObserver::ranOut(const BT::TreeNode& timeout) const
{
    const auto* decorator = dynamic_cast<const BT::DecoratorNode*>(&timeout);
    return decorator != nullptr && decorator->child() != nullptr &&
           halted_.contains(decorator->child()->fullPath());
}

Failure MissionObserver::describe(const BT::TreeNode& node) const
{
    Failure failure;
    failure.path = node.fullPath();
    failure.step = stepOfPath(failure.path);
    if (failure.step.empty())
    {
        failure.step = stepBelow(node);
    }
    if (failure.step.empty())
    {
        failure.step = stepIdOf(current_step_);
    }

    if (node.registrationName() == "Timeout")
    {
        failure.timed_out = true;
        const auto msec   = node.getInput<unsigned>("msec");
        failure.reason =
            msec ? std::format("timed out after {:g} s", msec.value() / 1000.0) : "timed out";
        return failure;
    }
    if (const auto* report = dynamic_cast<const LeafReport*>(&node);
        report != nullptr && !report->failureText().empty())
    {
        failure.reason = report->failureText();
        return failure;
    }
    failure.reason = std::format("{} failed", node.name());
    return failure;
}

std::vector<nervros_interfaces::msg::NodeEvent> MissionObserver::takeEvents()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return std::exchange(events_, {});
}

std::vector<std::string> MissionObserver::running() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return { running_.begin(), running_.end() };
}

std::optional<Failure> MissionObserver::failure() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return failure_;
}

std::string MissionObserver::currentStep() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return current_step_;
}

}  // namespace g1_orchestration
