#ifndef G1_BRINGUP__WALK_POLICY_SIM_NODE_HPP_
#define G1_BRINGUP__WALK_POLICY_SIM_NODE_HPP_

/**
 * @file walk_policy_sim_node.hpp
 * @brief Sim-only node running Unitree's pretrained G1 walking policy against unitree_mujoco.
 */

#include <torch/script.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "g1_bringup/blend_math.hpp"
#include "g1_bringup/walk_policy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_bringup
{

/**
 * @brief SIM-ONLY balance/walking stand-in built on `unitree_rl_gym`'s pretrained G1 policy.
 *
 * unitree_mujoco emulates only the low-level device: it runs no balance
 * controller, so a floating-base G1 topples within about a second and a half
 * unless something actively balances it. This node is that something, so the
 * simulated robot can stand and walk while the locomotion channel is exercised.
 *
 * It publishes leg targets on `/sim/walk_policy_cmd` and never touches
 * `/lowcmd`: `motion_service_sim` remains the single writer of that channel, and
 * consumes this as one of its inputs. Only motor slots 0-11 are ever written
 * here, and the consumer reads only those slots -- two independent guarantees
 * that a leg policy can never disturb the arms owned by `rt/arm_sdk`.
 *
 * @warning SIM-ONLY TESTBED SCAFFOLDING. The real G1 walks exclusively through
 * LocoClient and the vendor's onboard motion service via the real SDK; that path
 * is unchanged by this node's existence. This policy's gait, limits and recovery
 * differ from the vendor's, and it was trained with the arms near their default
 * posture, so it is a testbed for our own control plumbing rather than a model
 * of the robot's real locomotion. It must never run near hardware, and it is
 * launched exclusively by `sim.launch.py`.
 */
class WalkPolicySim : public rclcpp::Node
{
public:
    explicit WalkPolicySim(const rclcpp::NodeOptions& options);

private:
    void lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg);
    void walkCmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);
    void inferenceTick();
    void publishLegTargets(const std::array<double, kNumPolicyJoints>& targets);

    /**
     * @brief Startup phases: wait for state feedback, then run.
     *
     * Deliberately no posture ramp. The vendor's MuJoCo deployment enters its
     * policy loop immediately with the default posture as the initial target,
     * and matching that matters: easing a spawned, upright, unbalanced robot
     * into the crouched default posture instead makes it squat and topple
     * before the policy runs at all (measured).
     */
    enum class Phase : std::uint8_t
    {
        kWaitingForState,
        kRunning,
    };

    WalkPolicyConfig           config_{};
    torch::jit::script::Module policy_;

    double publish_rate_hz_{};
    /// Per-joint gains the policy expects; from the vendor's `kps`/`kds` for the legs.
    std::array<double, kNumPolicyJoints> kps_{};
    std::array<double, kNumPolicyJoints> kds_{};

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr walk_cmd_sub_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr      policy_cmd_pub_;
    rclcpp::TimerBase::SharedPtr                               inference_timer_;

    /// Latest measured leg state, refreshed from /lowstate; only slots 0-11 are read.
    std::array<double, kNumPolicyJoints> joint_position_{};
    std::array<double, kNumPolicyJoints> joint_velocity_{};
    std::array<double, 3>                base_angular_velocity_{};
    std::array<double, 4>                base_quat_wxyz_{ 1.0, 0.0, 0.0, 0.0 };
    bool                                 lowstate_received_{ false };

    /// Fed back into the next observation -- the policy is recurrent through its own action.
    std::array<double, kNumPolicyJoints> previous_action_{};
    std::array<double, 3>                command_mps_{};

    Phase                                 phase_{ Phase::kWaitingForState };
    std::chrono::steady_clock::time_point phase_start_{};
    /// Gait clock starts when inference does, not at construction.
    std::chrono::steady_clock::time_point run_start_{};

    /// Reused across ticks so the 50 Hz path does no per-tick allocation.
    std::vector<float> obs_buffer_;
};

}  // namespace g1_bringup

#endif  // G1_BRINGUP__WALK_POLICY_SIM_NODE_HPP_
