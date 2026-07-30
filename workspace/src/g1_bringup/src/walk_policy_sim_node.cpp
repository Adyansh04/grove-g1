/**
 * @file walk_policy_sim_node.cpp
 * @brief Sim-only node running Unitree's pretrained G1 walking policy against unitree_mujoco.
 */

#include "g1_bringup/walk_policy_sim_node.hpp"

#include <ATen/Parallel.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace g1_bringup
{

namespace
{
/// Vendor leg gains (`kps`/`kds` in unitree_rl_gym's g1.yaml) -- the policy's actions
/// are only meaningful against the stiffness it was trained with.
constexpr std::array<double, kNumPolicyJoints> kDefaultKps{ 100, 100, 100, 150, 40, 40,
                                                            100, 100, 100, 150, 40, 40 };
constexpr std::array<double, kNumPolicyJoints> kDefaultKds{ 2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2 };
}  // namespace

WalkPolicySim::WalkPolicySim(const rclcpp::NodeOptions& options)
  : rclcpp::Node("walk_policy_sim", options)
{
    const std::string policy_path = declare_parameter(
        "policy_path",
        "/opt/unitree_robotics/unitree_rl_gym/deploy/pre_train/g1/motion.pt");
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);  // vendor control_dt = 0.02

    config_.ang_vel_scale = declare_parameter("ang_vel_scale", config_.ang_vel_scale);
    config_.dof_pos_scale = declare_parameter("dof_pos_scale", config_.dof_pos_scale);
    config_.dof_vel_scale = declare_parameter("dof_vel_scale", config_.dof_vel_scale);
    config_.action_scale  = declare_parameter("action_scale", config_.action_scale);
    config_.gait_period_s = declare_parameter("gait_period_s", config_.gait_period_s);

    const auto to_array3 = [](const std::vector<double>& v, std::array<double, 3> fallback) {
        return v.size() == 3 ? std::array<double, 3>{ v[0], v[1], v[2] } : fallback;
    };
    config_.cmd_scale = to_array3(
        declare_parameter("cmd_scale", std::vector<double>{ 2.0, 2.0, 0.25 }),
        config_.cmd_scale);
    config_.max_cmd = to_array3(
        declare_parameter("max_cmd", std::vector<double>{ 0.8, 0.5, 1.57 }),
        config_.max_cmd);

    const auto to_array12 = [](const std::vector<double>&                  v,
                               const std::array<double, kNumPolicyJoints>& fallback) {
        std::array<double, kNumPolicyJoints> out = fallback;
        if (v.size() == kNumPolicyJoints)
        {
            std::copy(v.begin(), v.end(), out.begin());
        }
        return out;
    };
    config_.default_angles = to_array12(
        declare_parameter(
            "default_angles",
            std::vector<double>(config_.default_angles.begin(), config_.default_angles.end())),
        config_.default_angles);
    kps_ = to_array12(
        declare_parameter("kps", std::vector<double>(kDefaultKps.begin(), kDefaultKps.end())),
        kDefaultKps);
    kds_ = to_array12(
        declare_parameter("kds", std::vector<double>(kDefaultKds.begin(), kDefaultKds.end())),
        kDefaultKds);

    if (publish_rate_hz_ <= 0.0)
    {
        RCLCPP_FATAL(
            get_logger(),
            "publish_rate_hz (%f) must be strictly positive",
            publish_rate_hz_);
        throw std::invalid_argument("walk_policy_sim: publish_rate_hz must be strictly positive");
    }

    try
    {
        policy_ = torch::jit::load(policy_path);
        policy_.eval();
    }
    catch (const c10::Error& e)
    {
        RCLCPP_FATAL(get_logger(), "failed to load policy '%s': %s", policy_path.c_str(), e.what());
        throw std::runtime_error("walk_policy_sim: could not load the policy checkpoint");
    }
    /*
     * One inference thread. The default pool would spawn a worker per core and
     * fight a 20 ms wall timer for the same cores, which shows up as jitter in
     * a control loop rather than as speed.
     */
    at::set_num_threads(1);
    obs_buffer_.resize(kNumPolicyObs);

    // Matches unitree_mujoco's own /lowstate publisher; only the newest sample matters.
    const auto lowstate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    lowstate_sub_           = create_subscription<unitree_hg::msg::LowState>(
        "/lowstate",
        lowstate_qos,
        [this](const unitree_hg::msg::LowState::ConstSharedPtr& msg) { lowstateCallback(msg); });

    // Sim-internal command from the LocoClient responder; reliable so a stop is never dropped.
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    walk_cmd_sub_      = create_subscription<geometry_msgs::msg::Twist>(
        "/sim/walk_cmd",
        cmd_qos,
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { walkCmdCallback(msg); });

    // 50 Hz stream where only the newest tick matters, and the consumer has its own
    // staleness fallback -- same reasoning as /lowstate.
    const auto policy_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    policy_cmd_pub_ = create_publisher<unitree_hg::msg::LowCmd>("/sim/walk_policy_cmd", policy_qos);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    inference_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {
            inferenceTick();
        });

    RCLCPP_WARN(
        get_logger(),
        "walk_policy_sim is SIM-ONLY testbed scaffolding: it runs unitree_rl_gym's pretrained "
        "policy so the simulated G1 can stand and walk. The real robot walks only through "
        "LocoClient and the onboard motion service -- never run this near hardware (see README).");
}

void WalkPolicySim::lowstateCallback(const unitree_hg::msg::LowState::ConstSharedPtr& msg)
{
    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        joint_position_[i] = msg->motor_state[i].q;
        joint_velocity_[i] = msg->motor_state[i].dq;
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        base_angular_velocity_[i] = msg->imu_state.gyroscope[i];
    }
    for (std::size_t i = 0; i < 4; ++i)
    {
        base_quat_wxyz_[i] = msg->imu_state.quaternion[i];
    }
    lowstate_received_ = true;
}

void WalkPolicySim::walkCmdCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
    command_mps_ = { msg->linear.x, msg->linear.y, msg->angular.z };
}

void WalkPolicySim::publishLegTargets(const std::array<double, kNumPolicyJoints>& targets)
{
    /*
     * Only slots 0-11 are written. The consumer reads only those slots too, so
     * the arms owned by rt/arm_sdk stay untouchable from here even if this node
     * misbehaves.
     */
    unitree_hg::msg::LowCmd cmd;
    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        auto& motor = cmd.motor_cmd[i];
        motor.q     = static_cast<float>(targets[i]);
        motor.dq    = 0.0F;
        motor.tau   = 0.0F;
        motor.kp    = static_cast<float>(kps_[i]);
        motor.kd    = static_cast<float>(kds_[i]);
    }
    policy_cmd_pub_->publish(cmd);
}

void WalkPolicySim::inferenceTick()
{
    if (!lowstate_received_)
    {
        return;  // nothing to observe yet
    }

    const auto now = std::chrono::steady_clock::now();

    if (phase_ == Phase::kWaitingForState)
    {
        /*
         * Straight to inference on the first state sample, matching the
         * vendor's own MuJoCo deployment: it sets the default posture as the
         * initial target and enters the policy loop immediately.
         *
         * Its other entry point, deploy_real.py, first eases from wherever the
         * limp robot is lying into that posture over two seconds -- that is for
         * a real robot with an operator present, and it is actively wrong here.
         * Measured: ramping a spawned, upright, unbalanced robot into the
         * crouched default posture makes it squat and topple before the policy
         * ever runs, and no locomotion policy recovers from a fall.
         *
         * previous_action_ starts zeroed, so the first target this publishes is
         * exactly the default posture -- the vendor's pre-loop initial value.
         */
        phase_     = Phase::kRunning;
        run_start_ = now;
        RCLCPP_INFO(get_logger(), "state feedback acquired; running the walking policy");
    }

    const double elapsed_s = std::chrono::duration<double>(now - run_start_).count();
    const auto   obs       = assembleObservation(
        config_,
        base_angular_velocity_,
        base_quat_wxyz_,
        joint_position_,
        joint_velocity_,
        previous_action_,
        command_mps_,
        gaitPhase(config_, elapsed_s));

    for (std::size_t i = 0; i < kNumPolicyObs; ++i)
    {
        obs_buffer_[i] = static_cast<float>(obs[i]);
    }

    torch::NoGradGuard no_grad;
    const auto         input = torch::from_blob(
                           obs_buffer_.data(),
                           { 1, static_cast<long>(kNumPolicyObs) },
                           torch::kFloat32)
                           .clone();
    const auto  output = policy_.forward({ input }).toTensor().contiguous();
    const auto* action = output.data_ptr<float>();
    for (std::size_t i = 0; i < kNumPolicyJoints; ++i)
    {
        previous_action_[i] = action[i];
    }

    publishLegTargets(actionToJointTargets(config_, previous_action_));
}

}  // namespace g1_bringup
