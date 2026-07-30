#ifndef G1_BRINGUP__MOTION_SERVICE_SIM_NODE_HPP_
#define G1_BRINGUP__MOTION_SERVICE_SIM_NODE_HPP_

/**
 * @file motion_service_sim_node.hpp
 * @brief Sim-only node that synthesizes /lowcmd from /arm_sdk and /lowstate for MuJoCo bring-up,
 * and answers the LocoClient wire protocol (/api/sport/request, /api/sport/response) without
 * ever driving a leg.
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

#include "g1_bringup/blend_math.hpp"
#include "g1_bringup/loco_fsm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_bringup
{

/**
 * @brief SIM-ONLY stand-in for the onboard motion service -- see the package README's safety
 * banner.
 *
 * unitree_mujoco emulates only the low-level device (subscribes rt/lowcmd, publishes
 * rt/lowstate); nothing in the sim services rt/arm_sdk or the LocoClient wire protocol
 * (/api/sport/request, /api/sport/response), and with nothing commanding the legs the sim robot
 * collapses. This node closes both gaps in one place, mirroring the real motion service's own
 * scope (one service, not two): it subscribes /arm_sdk (what g1_hardware_interface's
 * G1ArmSdkSystem publishes) and /lowstate (what unitree_mujoco publishes), and is the ONLY thing
 * in this stack allowed to publish /lowcmd, and only when launched by sim.launch.py. Its
 * LocoClient responder half is **protocol-only**: it tracks an FSM state and applies the same
 * SET_FSM_ID/SET_VELOCITY acceptance rules a real onboard controller would (see loco_fsm.hpp),
 * but never actuates a leg -- nothing in dispatchSportRequest() touches /lowcmd, hold_q_, or any
 * arm-blend state, and the robot stays pelvis-welded regardless of FSM state or velocity
 * requests (see the README's "LocoClient wire responder" section for why walking-in-sim is out
 * of scope this milestone).
 *
 * Plain node, not lifecycle-managed: this is sim test scaffolding emulating an always-on
 * vendor service that has no activate/deactivate concept of its own on the real robot either
 * -- there is no meaningful inactive state for it to sit in.
 *
 * @warning Never run this near real hardware -- on the real robot the onboard motion service
 * already owns /lowcmd entirely, and two publishers on that channel is exactly the dual-writer
 * hazard that must never occur on a shared control channel.
 */
class MotionServiceSim : public rclcpp::Node
{
public:
    explicit MotionServiceSim(const rclcpp::NodeOptions& options);

private:
    void lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg);
    void armSdkCallback(const unitree_hg::msg::LowCmd::ConstSharedPtr& msg);
    void publishTick();

    void sportRequestCallback(const unitree_api::msg::Request::ConstSharedPtr& msg);

    /**
     * @brief Dispatches one /api/sport/request by api_id -- the responder's entire behavior
     * (see the README's dispatch table). The only side effect is loco_fsm_state_, and only for a
     * successful SET_FSM_ID; nothing here ever touches /lowcmd or any arm/leg state.
     * @param api_id         The request's header.identity.api_id.
     * @param parameter      The request's JSON parameter string.
     * @param response_data  Out param: the response's `data` field (only GET_FSM_ID populates it).
     * @return The response's status code.
     */
    std::int32_t dispatchSportRequest(
        std::int64_t api_id, const std::string& parameter, std::string& response_data);

    /// Parameters (config/motion_service_sim.yaml) -- see README for meaning
    /// and provenance of the defaults.
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

    rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr sport_request_sub_;
    rclcpp::Publisher<unitree_api::msg::Response>::SharedPtr   sport_response_pub_;
    /// LocoClient FSM state this responder tracks -- starts at Damp, matching the real robot's
    /// own boot state (see loco_fsm.hpp's legality table for why SET_VELOCITY is illegal here).
    int loco_fsm_state_{ kFsmDamp };

    /// Set once, from the first /lowstate sample, and never touched again --
    /// the frozen reference every subsequent tick holds legs/waist against
    /// and blends arms toward at weight 0. All callbacks and the publish
    /// timer run on the same single-threaded executor (this node's main()
    /// does a plain rclcpp::spin), so there is no cross-thread contention to
    /// guard here beyond this one publish-once latch.
    bool                               hold_pose_captured_{ false };
    std::array<double, kNumBodyMotors> hold_q_{};

    /// Latest /arm_sdk command (arm slots + the weight slot) and its arrival
    /// time.
    std::array<double, kNumArmMotors>     arm_cmd_q_{};
    std::array<double, kNumArmMotors>     arm_cmd_kp_{};
    std::array<double, kNumArmMotors>     arm_cmd_kd_{};
    double                                arm_cmd_weight_{ 0.0 };
    bool                                  arm_sdk_received_{ false };
    std::chrono::steady_clock::time_point arm_sdk_arrival_{};

    /// Persistent across ticks: stepEffectiveWeight() needs the previous
    /// value to slew from, and last_tick_ gives it a real dt even though this
    /// node runs off a wall timer rather than a control-loop period.
    double                                effective_weight_{ 0.0 };
    std::chrono::steady_clock::time_point last_tick_{};
};

}  // namespace g1_bringup

#endif  // G1_BRINGUP__MOTION_SERVICE_SIM_NODE_HPP_
