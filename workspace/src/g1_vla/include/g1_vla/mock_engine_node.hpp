#ifndef G1_VLA__MOCK_ENGINE_NODE_HPP_
#define G1_VLA__MOCK_ENGINE_NODE_HPP_

/**
 * @file mock_engine_node.hpp
 * @brief A deterministic stand-in for a policy engine.
 *
 * Serves the same chunk service a real policy does, walking the named joints toward a fixed
 * target a bounded step at a time. It exists so the gate can be tested without a model: aim it
 * at free space and every chunk should execute, aim it into a surface and every chunk should be
 * rejected before the arm moves.
 */

#include <g1_msgs/srv/get_action_chunk.hpp>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

namespace g1_vla
{

class G1VlaMockEngine : public rclcpp::Node
{
public:
    explicit G1VlaMockEngine(const rclcpp::NodeOptions& options);

private:
    using GetActionChunk = g1_msgs::srv::GetActionChunk;

    /// Builds one chunk from the measured pose, or refuses if no joint state has arrived.
    void onGetActionChunk(
        const GetActionChunk::Request::SharedPtr&  request,
        const GetActionChunk::Response::SharedPtr& response);

    void onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Service<GetActionChunk>::SharedPtr                    chunk_service_;

    std::mutex                    joints_mutex_;
    std::map<std::string, double> measured_;

    std::vector<std::string> joint_names_;
    int                      steps_per_chunk_{ 8 };
    double                   action_dt_s_{ 0.1 };
};

}  // namespace g1_vla

#endif  // G1_VLA__MOCK_ENGINE_NODE_HPP_
