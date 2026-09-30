#ifndef G1_ORCHESTRATION__SERVER_FLEET_HPP_
#define G1_ORCHESTRATION__SERVER_FLEET_HPP_

/**
 * @file server_fleet.hpp
 * @brief A set of action servers, seen from outside: cancelled and counted whoever sent the goals.
 *
 * The mission's own leaves cancel their goals when the tree halts, but a stop has to reach goals
 * the executor did not send, and has to know when the servers have gone quiet. Cancelling needs no
 * goal handle (a request with a zero id cancels every goal a server has) and a server's status
 * topic lists every goal it holds, so neither needs to know the action's type at compile time.
 */

#include <action_msgs/msg/goal_status_array.hpp>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_generic_client.hpp>
#include <rclcpp_action/generic_client.hpp>
#include <string>
#include <vector>

namespace g1_orchestration
{

class ServerFleet
{
public:
    /**
     * @brief Follows each server named in @p specs.
     *
     * @param node Owns the clients and subscriptions.
     * @param specs Entries of the form "/action/name:package/action/Type".
     * @param group Callback group for their callbacks, which an executor serves. Mutually
     *        exclusive, so a server's status messages are handled one at a time and in order: the
     *        newest is the one that stays.
     * @throws std::invalid_argument If an entry is malformed or its type cannot be found.
     */
    ServerFleet(
        rclcpp::Node& node, const std::vector<std::string>& specs,
        const rclcpp::CallbackGroup::SharedPtr& group);

    /**
     * @brief Asks every server that is up to cancel every goal it holds. Returns at once.
     *
     * A server that cannot be asked is logged and skipped, so one bad client does not leave the
     * rest holding their goals.
     *
     * @return How many servers were asked.
     */
    std::size_t cancelAll();

    /// The servers with a goal in flight, each as "name (n goals)".
    [[nodiscard]] std::vector<std::string> busy() const;

    /**
     * @brief Waits until no server is busy.
     *
     * @param timeout Longest to wait.
     * @return What was still busy at the end; empty when all went quiet.
     */
    [[nodiscard]] std::vector<std::string> settle(std::chrono::duration<double> timeout) const;

    /// The action names followed, in the order given.
    [[nodiscard]] std::vector<std::string> names() const;

    /// How many servers are followed.
    [[nodiscard]] std::size_t size() const { return servers_.size(); }

private:
    struct Server
    {
        std::string                                                        name;
        rclcpp_action::GenericClient::SharedPtr                            client;
        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status;
        std::atomic<int>                                                   in_flight{ 0 };
    };

    rclcpp::Logger                       logger_;
    std::vector<std::unique_ptr<Server>> servers_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__SERVER_FLEET_HPP_
