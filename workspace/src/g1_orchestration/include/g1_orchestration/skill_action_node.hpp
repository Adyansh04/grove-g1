#ifndef G1_ORCHESTRATION__SKILL_ACTION_NODE_HPP_
#define G1_ORCHESTRATION__SKILL_ACTION_NODE_HPP_

/**
 * @file skill_action_node.hpp
 * @brief The base for leaves whose action reports `success` and `message`.
 *
 * Every skill server in this stack answers with those two fields, so judging the outcome is the
 * same code five times over. Deriving from here instead of RosActionNode leaves a leaf with
 * only fillGoal() to write.
 *
 * NavigateToPose is the exception and derives from RosActionNode directly: Nav2's result is
 * empty, so reaching the goal is reported by the result CODE alone.
 */

#include <behaviortree_cpp/basic_types.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>

#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Turns a skill result into a node status, logging the server's own reason.
 *
 * An ABORTED goal still carries its result, and that message is the only place the reason
 * exists; logging a bare "did not complete" throws it away.
 *
 * @tparam ResultT The action client's wrapped-result type.
 * @param logger Where the outcome is reported.
 * @param name Leaf name, used as the log prefix.
 * @param wrapped The completed goal's wrapped result.
 * @return SUCCESS only when the goal succeeded and the skill reported success.
 */
template <typename ResultT>
BT::NodeStatus
judgeSkillResult(const rclcpp::Logger& logger, const std::string& name, const ResultT& wrapped)
{
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        const std::string why =
            wrapped.result && !wrapped.result->message.empty() ? wrapped.result->message : "";
        RCLCPP_ERROR(
            logger,
            "[%s] did not complete%s%s",
            name.c_str(),
            why.empty() ? "" : ": ",
            why.c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!wrapped.result->success)
    {
        // The message names the phase that failed, which is the actionable part: "the pick
        // failed" is not, "grasp: the hand did not close" is.
        RCLCPP_ERROR(logger, "[%s] %s", name.c_str(), wrapped.result->message.c_str());
        return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(logger, "[%s] %s", name.c_str(), wrapped.result->message.c_str());
    return BT::NodeStatus::SUCCESS;
}

/**
 * @brief Base for leaves whose action result carries `success` and `message`.
 *
 * @tparam ActionT The ROS action this leaf drives.
 */
template <typename ActionT>
class SkillActionNode : public RosActionNode<ActionT>
{
public:
    using Base          = RosActionNode<ActionT>;
    using WrappedResult = typename Base::WrappedResult;

    SkillActionNode(
        const std::string& instance_name, const BT::NodeConfig& config, RosContext context,
        std::string action_name)
      : Base(instance_name, config, std::move(context), std::move(action_name))
    {}

protected:
    BT::NodeStatus judgeResult(const WrappedResult& result) override
    {
        return judgeSkillResult(this->node_->get_logger(), this->name(), result);
    }
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILL_ACTION_NODE_HPP_
