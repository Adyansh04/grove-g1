#ifndef G1_ORCHESTRATION__SKILLS__LOOK_FOR_HPP_
#define G1_ORCHESTRATION__SKILLS__LOOK_FOR_HPP_

/**
 * @file look_for.hpp
 * @brief BT leaves that point the detector at what a task needs, and idle it again.
 */

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_orchestration/service_leaf.hpp"

namespace g1_orchestration
{

/**
 * @brief Writes the detector's `phrases`, then waits, without blocking the tick, until every
 *        named object is on /objects.
 *
 * Naming only what the task needs keeps each phrase to one track, and so keeps the bare-phrase
 * alias Pick and Place resolve by.
 */
class LookFor : public BT::StatefulActionNode
{
public:
    LookFor(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void           onHalted() override;

private:
    using Clock = std::chrono::steady_clock;

    enum class Write : std::uint8_t
    {
        kWaitingForService,
        kAwaitingReply,
        kDone,
    };

    /// Sends the phrase write once the service is up; past its budget, gives up on it.
    void pumpWrite();

    /// Ends the write and starts the wait for the objects.
    void finishWrite(bool taken);

    /// Drops the client node and everything it owns.
    void reset();

    rclcpp::Node::SharedPtr                                             node_;
    rclcpp::Node::SharedPtr                                             client_node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor>          executor_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr       set_phrases_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;

    std::string              detector_;
    std::vector<std::string> phrases_;  ///< What the detector is asked for.
    std::vector<std::string> wanted_;   ///< The /objects ids waited on.
    std::set<std::string>    seen_;     ///< Ids on the latest /objects.
    Write                    write_{ Write::kDone };
    double                   timeout_s_{ 0.0 };
    Clock::time_point        write_deadline_;
    Clock::time_point        deadline_;
};

/**
 * @brief Empties the detector's `phrases`, which idles it and frees the GPU.
 */
class StopLooking : public ServiceLeaf
{
public:
    StopLooking(const std::string& name, const BT::NodeConfig& config, RosContext context);
    static BT::PortsList providedPorts();
    BT::NodeStatus       tick() override;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SKILLS__LOOK_FOR_HPP_
