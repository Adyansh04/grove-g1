#include "g1_bringup/arm_sdk_sim_bridge_node.hpp"

#include "g1_bringup/blend_math.hpp"
#include "g1_hardware_interface/motor_crc_hg.hpp"

namespace g1_bringup
{

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

    // Best-effort: matches unitree_mujoco's own rt/lowstate publisher QoS
    // family (best-effort-compatible; verified RELIABLE in the milestone-1
    // spike, which a best-effort request is still compatible with) and only
    // the newest of the ~900 Hz stream ever matters.
    const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowstate_sub_           = create_subscription<unitree_hg::msg::LowState>(
        "/lowstate",
        lowstate_qos,
        [this](unitree_hg::msg::LowState::SharedPtr msg) { lowstateCallback(msg); });

    // Reliable: matches g1_hardware_interface's G1ArmSdkSystem, the sole
    // /arm_sdk publisher in this stack.
    const auto arm_sdk_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    arm_sdk_sub_           = create_subscription<unitree_hg::msg::LowCmd>(
        "/arm_sdk",
        arm_sdk_qos,
        [this](unitree_hg::msg::LowCmd::SharedPtr msg) { armSdkCallback(msg); });

    // Best-effort: matches unitree_mujoco's own rt/lowcmd subscription
    // exactly (verified in the milestone-1 spike). This bridge is the sim's
    // only /lowcmd publisher, so there's no reliability contention to
    // protect against, and only the newest tick ever matters at
    // publish_rate_hz_.
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
    for (int i = 0; i < kNumArmMotors; ++i)
    {
        latest_arm_measured_[static_cast<std::size_t>(i)] =
            msg->motor_state[static_cast<std::size_t>(kFirstArmMotor + i)].q;
    }

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

    // No /arm_sdk received yet is treated the same as stale: the effective
    // weight target is 0 either way, so arms simply hold at hold_q_ until
    // the first real command shows up.
    bool stale = true;
    if (arm_sdk_received_)
    {
        const auto age_s = std::chrono::duration<double>(now - arm_sdk_arrival_).count();
        stale            = age_s > arm_sdk_timeout_s_;
    }
    effective_weight_ =
        stepEffectiveWeight(effective_weight_, arm_cmd_weight_, stale, timeout_ramp_down_s_, dt);

    unitree_hg::msg::LowCmd cmd;  // rosidl-generated: zero-initialized, including reserved slots
                                  // and mode/mode_pr/mode_machine -- see the comment below on why
                                  // those are deliberately left untouched.

    for (int i = 0; i < kNumLegMotors; ++i)
    {
        auto& motor = cmd.motor_cmd[static_cast<std::size_t>(i)];
        motor.q     = static_cast<float>(hold_q_[static_cast<std::size_t>(i)]);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = static_cast<float>(leg_kp_);
        motor.kd    = static_cast<float>(leg_kd_);
    }
    for (int i = kNumLegMotors; i < kFirstArmMotor; ++i)
    {
        auto& motor = cmd.motor_cmd[static_cast<std::size_t>(i)];
        motor.q     = static_cast<float>(hold_q_[static_cast<std::size_t>(i)]);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = static_cast<float>(waist_kp_);
        motor.kd    = static_cast<float>(waist_kd_);
    }
    for (int i = 0; i < kNumArmMotors; ++i)
    {
        const auto idx         = static_cast<std::size_t>(i);
        const int  motor_index = kFirstArmMotor + i;
        auto&      motor       = cmd.motor_cmd[static_cast<std::size_t>(motor_index)];
        motor.q                = static_cast<float>(blend(
            hold_q_[static_cast<std::size_t>(motor_index)],
            arm_cmd_q_[idx],
            effective_weight_));
        motor.dq               = 0.0F;
        motor.tau              = 0.0F;
        motor.kp = static_cast<float>(blend(arm_hold_kp_, arm_cmd_kp_[idx], effective_weight_));
        motor.kd = static_cast<float>(blend(arm_hold_kd_, arm_cmd_kd_[idx], effective_weight_));
    }
    cmd.motor_cmd[kWeightMotorIndex].q = static_cast<float>(effective_weight_);

    // mode/mode_pr/mode_machine are deliberately never set: read directly
    // from unitree_mujoco's own source
    // (simulate/src/unitree_sdk2_bridge.h, RobotBridge::run()), its actuator
    // torque is computed purely as
    // `tau_ff + kp * (q_des - q_meas) + kd * (dq_des - dq_meas)` per motor
    // slot -- the incoming LowCmd's mode fields are never read for
    // actuation. This is a sim-specific finding; the real onboard motion
    // service's use of these fields (if any) is unverified and stays a
    // hardware re-validation item.
    g1_hardware_interface::vendored::computeLowCmdCrc(cmd);
    lowcmd_pub_->publish(cmd);
}

}  // namespace g1_bringup
