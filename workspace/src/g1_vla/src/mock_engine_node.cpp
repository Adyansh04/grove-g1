/**
 * @file mock_engine_node.cpp
 * @brief The stand-in policy engine: a bounded walk toward a fixed target.
 */

#include "g1_vla/mock_engine_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace g1_vla
{

G1VlaMockEngine::G1VlaMockEngine(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_vla_mock_engine", options)
{
    joint_names_ =
        declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});
    target_positions_ =
        declare_parameter<std::vector<double>>("target_positions", std::vector<double>{});
    steps_per_chunk_ = static_cast<int>(declare_parameter<int64_t>("steps_per_chunk", 8));
    action_dt_s_     = declare_parameter<double>("action_dt_s", 0.1);
    // Small enough that consecutive waypoints stay inside the gate's segment-step budget, which
    // is what makes per-waypoint collision checking meaningful.
    step_rad_ = declare_parameter<double>("step_rad", 0.05);

    if (joint_names_.size() != target_positions_.size())
    {
        throw std::runtime_error("joint_names and target_positions must be the same length");
    }

    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr& msg) { onJointStates(msg); });

    chunk_service_ = create_service<GetActionChunk>(
        "~/get_action_chunk",
        [this](
            const GetActionChunk::Request::SharedPtr&  request,
            const GetActionChunk::Response::SharedPtr& response) {
            onGetActionChunk(request, response);
        });

    RCLCPP_INFO(get_logger(), "mock engine serving %zu joint(s)", joint_names_.size());
}

void G1VlaMockEngine::onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
{
    const std::lock_guard<std::mutex> lock(joints_mutex_);
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
    {
        measured_[msg->name[i]] = msg->position[i];
    }
}

void G1VlaMockEngine::onGetActionChunk(
    const GetActionChunk::Request::SharedPtr&  request,
    const GetActionChunk::Response::SharedPtr& response)
{
    std::vector<double> current;
    current.reserve(joint_names_.size());
    {
        const std::lock_guard<std::mutex> lock(joints_mutex_);
        for (const std::string& name : joint_names_)
        {
            const auto it = measured_.find(name);
            if (it == measured_.end())
            {
                response->ok      = false;
                response->message = "no joint state yet for " + name;
                return;
            }
            current.push_back(it->second);
        }
    }

    response->chunk.joint_names = joint_names_;
    for (int step = 1; step <= steps_per_chunk_; ++step)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
            const double remaining = target_positions_[i] - current[i];
            current[i] += std::clamp(remaining, -step_rad_, step_rad_);
            point.positions.push_back(current[i]);
        }
        const double t                = action_dt_s_ * step;
        point.time_from_start.sec     = static_cast<int32_t>(t);
        point.time_from_start.nanosec = static_cast<uint32_t>(std::lround(std::fmod(t, 1.0) * 1e9));
        response->chunk.points.push_back(point);
    }

    response->ok = true;
    RCLCPP_DEBUG(get_logger(), "chunk for '%s'", request->instruction.c_str());
}

}  // namespace g1_vla
