#ifndef G1_ORCHESTRATION__SKILLS__WAIT_HPP_
#define G1_ORCHESTRATION__SKILLS__WAIT_HPP_

/**
 * @file wait.hpp
 * @brief BT leaf that waits, as a gesture holds a pose or a skill gives a person time.
 */

#include <behaviortree_cpp/action_node.h>

#include <atomic>
#include <chrono>
#include <string>

#include "g1_orchestration/leaf_report.hpp"
#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Succeeds once `seconds` have gone by; fails at once when the mission winds down.
 *        Ticked like any leaf, so it never blocks the tree.
 */
class Wait : public BT::StatefulActionNode, public LeafReport
{
public:
    /// The longest a wait may be: longer is a pause, which the operator gives.
    static constexpr double kMaxSeconds = 60.0;

    Wait(const std::string& name, const BT::NodeConfig& config, const RosContext& context);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void           onHalted() override {}

private:
    using Clock = std::chrono::steady_clock;

    const std::atomic<bool>* stop_;
    Clock::time_point        until_{};
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__WAIT_HPP_
