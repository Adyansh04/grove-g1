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
#include <memory>
#include <optional>
#include <string>

#include "g1_bringup/blend_math.hpp"
#include "g1_bringup/loco_fsm.hpp"
#include "g1_bringup/walk_policy.hpp"
#include "g1_bringup/walk_policy_session.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_bringup
{

/**
 * @brief Sim-only stand-in for the onboard motion service.
 *
 * Synthesizes /lowcmd from /arm_sdk and /lowstate for MuJoCo simulation,
 * and handles the LocoClient wire protocol (/api/sport/request, /api/sport/response).
 *
 * @warning Never run this near real hardware — the onboard motion service already owns /lowcmd.
 */
class MotionServiceSim : public rclcpp::Node
{
public:
    explicit MotionServiceSim(const rclcpp::NodeOptions& options);

private:
    void lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg);
    void armSdkCallback(const unitree_hg::msg::LowCmd::ConstSharedPtr& msg);
    void sportModeStateCallback(const unitree_go::msg::SportModeState::ConstSharedPtr& msg);
    void publishTick();

    /// Reads config/walk_policy.yaml and loads the ONNX session. Returns false (with the policy
    /// left disabled) if the joint order or model path is wrong, so a misconfigured policy
    /// degrades to the stiff hold rather than driving the legs from a bad mapping.
    bool setUpWalkPolicy();

    /// One decimated inference: assembles the observation from the latest state, runs the policy,
    /// and stores the lower-body targets publishTick() consumes.
    void walkPolicyTick();

    void sportRequestCallback(const unitree_api::msg::Request::ConstSharedPtr& msg);

    /**
     * @brief Dispatches one /api/sport/request by api_id -- the responder's entire behavior
     * (see the README's dispatch table). Side effects are loco_fsm_state_ and, for an ACCEPTED
     * SET_VELOCITY, the latched command the walking policy consumes -- so this path does reach
     * motors 0-14. It still never touches the arm slots or publishes anything itself.
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
    /// Lower-body joint states. joint_state_broadcaster only publishes the arms,
    /// because the hardware interface is arm-only; without these
    /// robot_state_publisher cannot connect pelvis to torso_link and every frame
    /// above the waist, sensors included, is stranded in its own TF tree.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr lower_joint_pub_;
    /// Names filled once; only the numbers change per sample.
    sensor_msgs::msg::JointState lower_joint_msg_;
    /// /lowstate arrives at ~1 kHz. Publishing joint states that fast is pure
    /// waste next to joint_state_broadcaster's ~200 Hz, and it measurably
    /// disturbed bring-up, so it is decimated.
    int                          lower_joint_decimate_ = 0;
    bool                         publish_lower_joints_ = false;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    /// Base linear velocity for the policy observation. /lowstate carries no such field -- the
    /// sim's own bridge publishes it here from the pelvis `frame_vel` sensor. This is precisely
    /// why the walking policy is structurally sim-only: on the real robot this topic is served by
    /// the onboard motion service, which is off whenever this stack owns the low-level channel.
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportmodestate_sub_;

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

    /// Walking policy state. The 50 Hz policy timer and the 500 Hz publish timer both run on this
    /// node's single-threaded executor, so these need no synchronisation -- the same contract the
    /// arm-blend members above rely on.
    bool                               walk_policy_enabled_{ false };
    double                             walk_policy_staleness_timeout_s_{ 0.1 };
    PolicyConfig                       walk_policy_config_{};
    std::unique_ptr<WalkPolicySession> walk_policy_session_;

    /// Wall timer for policy evaluation.
    rclcpp::TimerBase::SharedPtr walk_policy_timer_;

    std::array<float, kActionDim>         walk_last_action_{};
    std::array<double, kNumLowerMotors>   walk_target_q_{};
    bool                                  walk_target_valid_{ false };
    std::chrono::steady_clock::time_point walk_target_stamp_{};

    /// Latest base state feeding the observation, refreshed by the /lowstate and
    /// /sportmodestate callbacks.
    PolicyInputs walk_inputs_{};
    bool         sportmodestate_received_{ false };

    /// Velocity latched by an ACCEPTED SET_VELOCITY, carrying the request's own duration as its
    /// dead-man. Cleared whenever a successful SET_FSM_ID leaves Start, so releasing locomotion
    /// authority stops the robot instead of leaving the last command in force.
    std::optional<VelocityCommand> walk_velocity_{};
};

}  // namespace g1_bringup

#endif  // G1_BRINGUP__MOTION_SERVICE_SIM_NODE_HPP_
