#ifndef G1_BRINGUP__ARM_SDK_SIM_BRIDGE_NODE_HPP_
#define G1_BRINGUP__ARM_SDK_SIM_BRIDGE_NODE_HPP_

#include <array>
#include <chrono>

#include "g1_bringup/blend_math.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_bringup
{

// SIM-ONLY stand-in for the onboard motion service -- see the package
// README's safety banner. unitree_mujoco emulates only the low-level device
// (subscribes rt/lowcmd, publishes rt/lowstate); nothing in the sim services
// rt/arm_sdk, and with nothing commanding the legs the sim robot collapses.
// This node closes that gap kinematically: it subscribes /arm_sdk (what
// g1_hardware_interface's G1ArmSdkSystem publishes) and /lowstate (what
// unitree_mujoco publishes), and is the ONLY thing in this stack allowed to
// publish /lowcmd, and only when launched by sim.launch.py. Never run this
// near real hardware -- on the real robot the onboard motion service already
// owns /lowcmd entirely, and two publishers on that channel is exactly the
// dual-writer hazard that must never occur on a shared control channel.
//
// Plain node, not lifecycle-managed: this is sim test scaffolding emulating
// an always-on vendor service that has no activate/deactivate concept of its
// own on the real robot either -- there is no meaningful inactive state for
// it to sit in.
class ArmSdkSimBridge : public rclcpp::Node
{
public:
    explicit ArmSdkSimBridge(const rclcpp::NodeOptions& options);

private:
    void lowstateCallback(unitree_hg::msg::LowState::SharedPtr msg);
    void armSdkCallback(unitree_hg::msg::LowCmd::SharedPtr msg);
    void publishTick();

    // Parameters (config/arm_sdk_sim_bridge.yaml) -- see README for meaning
    // and provenance of the defaults.
    double publish_rate_hz_{};
    double leg_kp_{};
    double leg_kd_{};
    double waist_kp_{};
    double waist_kd_{};
    double arm_hold_kp_{};
    double arm_hold_kd_{};
    double arm_sdk_timeout_s_{};
    double timeout_ramp_down_s_{};

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Subscription<unitree_hg::msg::LowCmd>::SharedPtr   arm_sdk_sub_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr      lowcmd_pub_;
    rclcpp::TimerBase::SharedPtr                               publish_timer_;

    // Set once, from the first /lowstate sample, and never touched again --
    // the frozen reference every subsequent tick holds legs/waist against
    // and blends arms toward at weight 0. All callbacks and the publish
    // timer run on the same single-threaded executor (this node's main()
    // does a plain rclcpp::spin), so there is no cross-thread contention to
    // guard here beyond this one publish-once latch.
    bool                               hold_pose_captured_{ false };
    std::array<double, kNumBodyMotors> hold_q_{};

    // Latest /arm_sdk command (arm slots + the weight slot) and its arrival
    // time.
    std::array<double, kNumArmMotors>     arm_cmd_q_{};
    std::array<double, kNumArmMotors>     arm_cmd_kp_{};
    std::array<double, kNumArmMotors>     arm_cmd_kd_{};
    double                                arm_cmd_weight_{ 0.0 };
    bool                                  arm_sdk_received_{ false };
    std::chrono::steady_clock::time_point arm_sdk_arrival_{};

    // Persistent across ticks: stepEffectiveWeight() needs the previous
    // value to slew from, and last_tick_ gives it a real dt even though this
    // node runs off a wall timer rather than a control-loop period.
    double                                effective_weight_{ 0.0 };
    std::chrono::steady_clock::time_point last_tick_{};
};

}  // namespace g1_bringup

#endif  // G1_BRINGUP__ARM_SDK_SIM_BRIDGE_NODE_HPP_
