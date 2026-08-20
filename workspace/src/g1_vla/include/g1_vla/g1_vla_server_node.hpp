#ifndef G1_VLA__G1_VLA_SERVER_NODE_HPP_
#define G1_VLA__G1_VLA_SERVER_NODE_HPP_

/**
 * @file g1_vla_server_node.hpp
 * @brief The grasp skill: a policy's action chunks, checked against the planning scene before
 *        any of them reach a controller.
 *
 * Adds no command path. Chunks go out through the same trajectory controllers MoveIt already
 * drives, so the one-writer rule is unaffected by this node existing, and the legs stay on the
 * balance policy throughout.
 *
 * Takes no control authority of its own: the arm and hands must already be acquired.
 */

#include <atomic>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <cstdint>
#include <g1_msgs/action/grasp.hpp>
#include <g1_msgs/srv/get_action_chunk.hpp>
#include <memory>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_vla/chunk_utils.hpp"

namespace g1_vla
{

/// One trajectory controller the gate can hand a slice of a chunk to.
struct ControllerTarget
{
    std::string              name;    ///< For failure messages; also the action namespace.
    std::vector<std::string> joints;  ///< What it owns, read from the robot model.
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client;
};

class G1VlaServer : public rclcpp::Node
{
public:
    explicit G1VlaServer(const rclcpp::NodeOptions& options);

    /// Waits for a goal still running on a detached thread, which outlives its clients otherwise.
    ~G1VlaServer() override;

    /**
     * @brief Loads the robot model and builds the controller clients.
     *
     * Separate from the constructor because the model arrives on this node's own callbacks, so
     * loading it before something spins the node deadlocks.
     */
    void initialize();

private:
    using Grasp                 = g1_msgs::action::Grasp;
    using GoalHandle            = rclcpp_action::ServerGoalHandle<Grasp>;
    using GetActionChunk        = g1_msgs::srv::GetActionChunk;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using JointTrajectory       = trajectory_msgs::msg::JointTrajectory;

    /// What one goal ended up doing. Its counters are the record the experiment reads.
    struct Outcome
    {
        bool        success{ false };
        std::string message;
        uint16_t    executed{ 0 };
        uint16_t    rejected{ 0 };
    };

    /// Claims the arm for one goal.
    bool acquire();

    /**
     * @brief Re-reads the tunables so a change takes effect on the next goal.
     *
     * Caching them at construction would mean every threshold needed a restart to move, which
     * is the wrong trade for numbers whose right value is found by watching the robot.
     */
    void refreshTunables();

    void executeGrasp(const std::shared_ptr<GoalHandle>& goal_handle);

    /**
     * @brief The query-validate-execute loop, between the collision exemption and its restore.
     *
     * Split out so every way this can end still passes through the one restore in its caller.
     */
    Outcome runGrasp(
        const std::shared_ptr<GoalHandle>& goal_handle, const std::string& side, double start_z);

    /// Asks the engine for the next chunk. @return nullopt with @p why set.
    std::optional<JointTrajectory> requestChunk(const std::string& instruction, std::string& why);

    /**
     * @brief Every reason this chunk must not be executed.
     *
     * @return Empty when the chunk may run. The kinematic checks come first because they are
     *         local; the planning-scene calls are the expensive part.
     */
    std::string rejectionReason(const JointTrajectory& chunk, const std::string& side);

    /// Asks move_group whether each waypoint is a legal state. Uses its live scene and matrix.
    std::string checkWaypoints(const JointTrajectory& chunk, const std::string& group);

    /// Splits the chunk across the controllers, sends it, and waits for all of them.
    bool executeChunk(const JointTrajectory& chunk, std::string& why);

    /// Cancels anything still running on the controllers.
    void cancelAll();

    /// Allows or restores contact between one hand and the octomap. @return false on failure.
    bool setHandContact(const std::string& side, bool allowed);

    JointMap measuredJoints();

    /// Height of @p object_id on /objects. @return nullopt if unknown or too old.
    std::optional<double> objectHeight(const std::string& object_id);

    void onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg);
    void onJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_states_sub_;

    std::mutex                         objects_mutex_;
    vision_msgs::msg::Detection3DArray objects_;
    std::mutex                         joints_mutex_;
    JointMap                           measured_;

    rclcpp::Client<GetActionChunk>::SharedPtr                       engine_;
    rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr   validity_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr   get_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
    rclcpp_action::Server<Grasp>::SharedPtr                         grasp_server_;

    robot_model_loader::RobotModelLoaderPtr model_loader_;
    moveit::core::RobotModelConstPtr        model_;
    std::vector<ControllerTarget>           controllers_;
    /// Velocity limits by joint, read from the model once.
    JointMap limits_;

    std::string engine_service_;
    double      engine_timeout_s_{ 10.0 };
    double      max_start_jump_rad_{ 0.15 };
    double      max_segment_step_rad_{ 0.20 };
    double      velocity_scaling_{ 0.5 };
    int         max_rejected_chunks_{ 5 };
    double      timeout_s_{ 90.0 };
    double      chunk_exec_timeout_s_{ 10.0 };
    double      success_lift_m_{ 0.05 };
    double      object_timeout_s_{ 1.0 };

    /// One goal at a time: two would drive overlapping joints through the same controllers.
    std::atomic<bool> busy_{ false };
    std::atomic<int>  goals_running_{ 0 };
};

}  // namespace g1_vla

#endif  // G1_VLA__G1_VLA_SERVER_NODE_HPP_
