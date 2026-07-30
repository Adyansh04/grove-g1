/**
 * @file arm_sdk_sim_bridge_node.cpp
 * @brief Sim-only bridge that turns /arm_sdk weighted commands into /lowcmd for unitree_mujoco.
 */

#include "g1_bringup/arm_sdk_sim_bridge_node.hpp"

#include <stdexcept>

#include "g1_bringup/blend_math.hpp"
#include "g1_hardware_interface/motor_crc_hg.hpp"

namespace g1_bringup
{

/**
 * @brief Declare parameters, wire the /lowstate, /arm_sdk, and /lowcmd topics, and start the
 * publish timer for this sim-only bridge.
 *
 * @param options Node options forwarded to rclcpp::Node.
 * @throws std::invalid_argument If publish_rate_hz, arm_sdk_timeout_ms, or timeout_ramp_down_s
 *         resolve to a non-positive value.
 */
ArmSdkSimBridge::ArmSdkSimBridge(const rclcpp::NodeOptions& options)
  : rclcpp::Node("arm_sdk_sim_bridge", options)
{
    publish_rate_hz_                = declare_parameter("publish_rate_hz", 500.0);
    leg_kp_                         = declare_parameter("leg_kp", 100.0);
    leg_kd_                         = declare_parameter("leg_kd", 1.0);
    waist_kp_                       = declare_parameter("waist_kp", 50.0);
    waist_kd_                       = declare_parameter("waist_kd", 1.0);
    arm_hold_kp_                    = declare_parameter("arm_hold_kp", 40.0);
    arm_hold_kd_                    = declare_parameter("arm_hold_kd", 1.0);
    const double arm_sdk_timeout_ms = declare_parameter("arm_sdk_timeout_ms", 500.0);
    arm_sdk_timeout_s_              = arm_sdk_timeout_ms / 1000.0;
    timeout_ramp_down_s_            = declare_parameter("timeout_ramp_down_s", 1.0);

    /*
     * Fail fast on a nonsensical rate/duration tunable, mirroring
     * g1_hardware_interface::G1ArmSdkSystem::on_init's strictly-positive
     * gate. publish_rate_hz <= 0 makes the wall-timer period's
     * duration_cast undefined and breaks the first tick's dt;
     * arm_sdk_timeout_s/timeout_ramp_down_s <= 0 turns the advertised
     * no-snap staleness decay into an instantaneous snap (or an upward
     * ramp) via stepEffectiveWeight's max_step. Gains (leg/waist/arm_hold
     * kp/kd) are left unchecked here, same as G1ArmSdkSystem::on_init
     * leaves its own per-joint kp/kd unchecked -- only the rate/duration
     * tunables are load-bearing for avoiding UB/snap-on-misconfiguration.
     */
    if (publish_rate_hz_ <= 0.0 || arm_sdk_timeout_s_ <= 0.0 || timeout_ramp_down_s_ <= 0.0)
    {
        RCLCPP_FATAL(
            get_logger(),
            "publish_rate_hz (%f), arm_sdk_timeout_ms (%f s), and timeout_ramp_down_s (%f) must "
            "all be strictly positive",
            publish_rate_hz_,
            arm_sdk_timeout_s_,
            timeout_ramp_down_s_);
        throw std::invalid_argument(
            "arm_sdk_sim_bridge: publish_rate_hz/arm_sdk_timeout_ms/timeout_ramp_down_s must be "
            "strictly positive");
    }

    /*
     * Best-effort: matches unitree_mujoco's own rt/lowstate publisher QoS
     * family (best-effort-compatible; verified RELIABLE in the milestone-1
     * spike, which a best-effort request is still compatible with) and only
     * the newest of the ~900 Hz stream ever matters.
     */
    const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowstate_sub_           = create_subscription<unitree_hg::msg::LowState>(
        "/lowstate",
        lowstate_qos,
        [this](unitree_hg::msg::LowState::SharedPtr msg) { lowstateCallback(msg); });

    /*
     * Reliable: matches g1_hardware_interface's G1ArmSdkSystem, the sole
     * /arm_sdk publisher in this stack.
     */
    const auto arm_sdk_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    arm_sdk_sub_           = create_subscription<unitree_hg::msg::LowCmd>(
        "/arm_sdk",
        arm_sdk_qos,
        [this](unitree_hg::msg::LowCmd::SharedPtr msg) { armSdkCallback(msg); });

    /*
     * Best-effort: matches unitree_mujoco's own rt/lowcmd subscription
     * exactly (verified in the milestone-1 spike). This bridge is the sim's
     * only /lowcmd publisher, so there's no reliability contention to
     * protect against, and only the newest tick ever matters at
     * publish_rate_hz_.
     */
    const auto lowcmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowcmd_pub_           = create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", lowcmd_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {
            publishTick();
        });

    RCLCPP_WARN(
        get_logger(),
        "arm_sdk_sim_bridge is SIM-ONLY: it owns /lowcmd in this process and must never run "
        "against real hardware (see README.md).");
}

void ArmSdkSimBridge::lowstateCallback(unitree_hg::msg::LowState::SharedPtr msg)
{
    if (hold_pose_captured_)
    {
        return;
    }
    for (int i = 0; i < kNumBodyMotors; ++i)
    {
        hold_q_[static_cast<std::size_t>(i)] = msg->motor_state[static_cast<std::size_t>(i)].q;
    }
    hold_pose_captured_ = true;
}

void ArmSdkSimBridge::armSdkCallback(unitree_hg::msg::LowCmd::SharedPtr msg)
{
    for (int i = 0; i < kNumArmMotors; ++i)
    {
        const auto& motor = msg->motor_cmd[static_cast<std::size_t>(kFirstArmMotor + i)];
        arm_cmd_q_[static_cast<std::size_t>(i)]  = motor.q;
        arm_cmd_kp_[static_cast<std::size_t>(i)] = motor.kp;
        arm_cmd_kd_[static_cast<std::size_t>(i)] = motor.kd;
    }
    arm_cmd_weight_   = msg->motor_cmd[kWeightMotorIndex].q;
    arm_sdk_received_ = true;
    arm_sdk_arrival_  = std::chrono::steady_clock::now();
}

void ArmSdkSimBridge::publishTick()
{
    if (!hold_pose_captured_)
    {
        return;  // nothing to hold yet -- no /lowstate sample has arrived
    }

    const auto   now = std::chrono::steady_clock::now();
    const double dt  = last_tick_.time_since_epoch().count() == 0 ?
                           1.0 / publish_rate_hz_ :
                           std::chrono::duration<double>(now - last_tick_).count();
    last_tick_       = now;

    /*
     * No /arm_sdk received yet is treated the same as stale: the effective
     * weight target is 0 either way, so arms simply hold at hold_q_ until
     * the first real command shows up.
     */
    bool stale = true;
    if (arm_sdk_received_)
    {
        const auto age_s = std::chrono::duration<double>(now - arm_sdk_arrival_).count();
        stale            = age_s > arm_sdk_timeout_s_;
    }
    effective_weight_ =
        stepEffectiveWeight(effective_weight_, arm_cmd_weight_, stale, timeout_ramp_down_s_, dt);

    /*
     * assembleSimLowCmd() (blend_math) does the leg/waist stiff-hold,
     * arm blend, and weight-slot echo -- unit-tested directly (see
     * test/test_assemble_sim_low_cmd.cpp) without a live node or DDS.
     */
    unitree_hg::msg::LowCmd cmd = assembleSimLowCmd(
        hold_q_,
        arm_cmd_q_,
        arm_cmd_kp_,
        arm_cmd_kd_,
        effective_weight_,
        leg_kp_,
        leg_kd_,
        waist_kp_,
        waist_kd_,
        arm_hold_kp_,
        arm_hold_kd_);

    /*
     * mode/mode_pr/mode_machine are deliberately never set: read directly
     * from unitree_mujoco's own source
     * (simulate/src/unitree_sdk2_bridge.h, RobotBridge::run()), its actuator
     * torque is computed purely as
     * `tau_ff + kp * (q_des - q_meas) + kd * (dq_des - dq_meas)` per motor
     * slot -- the incoming LowCmd's mode fields are never read for
     * actuation. This is a sim-specific finding; the real onboard motion
     * service's use of these fields (if any) is unverified and stays a
     * hardware re-validation item.
     */
    g1_hardware_interface::vendored::computeLowCmdCrc(cmd);
    lowcmd_pub_->publish(cmd);
}

}  // namespace g1_bringup
