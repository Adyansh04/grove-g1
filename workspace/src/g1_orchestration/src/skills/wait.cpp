/**
 * @file wait.cpp
 * @brief The Wait leaf.
 */

#include "g1_orchestration/skills/wait.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <string>

namespace g1_orchestration
{

Wait::Wait(const std::string& name, const BT::NodeConfig& config, const RosContext& context)
  : BT::StatefulActionNode(name, config)
  , stop_(context.stop)
  , pause_(context.pause)
{}

BT::PortsList Wait::providedPorts()
{
    return { BT::InputPort<double>("seconds", "How long, 0 to 60 s.") };
}

BT::NodeStatus Wait::onStart()
{
    setFailureText({});
    waiting_ = heldByPause(pause_, *this);
    return waiting_ ? BT::NodeStatus::RUNNING : begin();
}

BT::NodeStatus Wait::onRunning()
{
    if (windingDown(stop_))
    {
        setFailureText("stopped");
        return BT::NodeStatus::FAILURE;
    }
    if (waiting_)
    {
        waiting_ = heldByPause(pause_, *this);
        return waiting_ ? BT::NodeStatus::RUNNING : begin();
    }
    return Clock::now() >= until_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

BT::NodeStatus Wait::begin()
{
    if (windingDown(stop_))
    {
        setFailureText("stopped");
        return BT::NodeStatus::FAILURE;
    }
    const auto seconds = getInput<double>("seconds");
    if (!seconds || !std::isfinite(*seconds) || *seconds < 0.0 || *seconds > kMaxSeconds)
    {
        setFailureText(std::format("seconds must be from 0 to {:.0f}", kMaxSeconds));
        return BT::NodeStatus::FAILURE;
    }
    until_ = Clock::now() +
             std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(*seconds));
    return Clock::now() >= until_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

}  // namespace g1_orchestration
