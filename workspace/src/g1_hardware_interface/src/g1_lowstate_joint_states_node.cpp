/**
 * @file g1_lowstate_joint_states_node.cpp
 * @brief Publishes the legs and waist to /joint_states from /lowstate. HARDWARE ONLY.
 *
 * Motors 0-14 belong to the onboard controller, so no ros2_control component exports them and
 * joint_state_broadcaster never sees them. robot_state_publisher emits no transform at all for
 * a joint it has never received, which strands every frame above the waist -- including the
 * Mid360 the LiDAR-inertial odometry depends on. The package README has the full account.
 *
 * Simulation gets the same 15 motors from g1_motion_service_sim, so it does not need this.
 */

#include <memory>
#include <stdexcept>

#include "g1_hardware_interface/lowstate_joint_states.hpp"
#include "rclcpp/rclcpp.hpp"

namespace g1_hardware_interface
{

class LowStateJointStates : public rclcpp::Node
{
public:
    LowStateJointStates()
      : rclcpp::Node("g1_lowstate_joint_states")
    {
        const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
        if (publish_rate_hz <= 0.0)
        {
            throw std::runtime_error("publish_rate_hz must be positive");
        }
        // Throttled on elapsed time rather than by counting messages: /lowstate is ~500 Hz on
        // the robot and nearer 1 kHz in simulation, and a decimation factor would have to know
        // which.
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz);

        initLowerBodyJointState(msg_);
        // Latest-only, and reliable: robot_state_publisher merges by joint name, so this sits
        // alongside joint_state_broadcaster's arm publication rather than competing with it.
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::QoS(rclcpp::KeepLast(1)));
        // Best-effort, depth 1, matching the robot's own /lowstate publisher.
        lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
            "/lowstate",
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
            [this](const unitree_hg::msg::LowState::ConstSharedPtr& msg) { onLowState(*msg); });

        // Silence here looks exactly like the broken TF tree this node exists to prevent, so it
        // says so itself rather than leaving the symptom to surface three packages downstream.
        idle_timer_ = create_wall_timer(std::chrono::seconds(5), [this] {
            if (!received_)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "no /lowstate yet -- the legs and waist are absent from /joint_states, so "
                    "TF above the waist will not resolve and LiDAR odometry cannot start.");
            }
        });
    }

private:
    void onLowState(const unitree_hg::msg::LowState& state)
    {
        received_                = true;
        const rclcpp::Time stamp = now();
        if (have_published_ && stamp - last_publish_ < publish_period_)
        {
            return;
        }
        last_publish_   = stamp;
        have_published_ = true;

        fillLowerBodyJointState(state, msg_);
        msg_.header.stamp = stamp;
        joint_pub_->publish(msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::TimerBase::SharedPtr                               idle_timer_;
    sensor_msgs::msg::JointState                               msg_;
    rclcpp::Duration                                           publish_period_{ 0, 0 };
    rclcpp::Time                                               last_publish_;
    bool                                                       have_published_ = false;
    bool                                                       received_       = false;
};

}  // namespace g1_hardware_interface

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<g1_hardware_interface::LowStateJointStates>());
    }
    catch (const std::exception& e)
    {
        // An uncaught throw exits -6 through std::terminate, which reads as a crash rather than
        // as a rejected parameter. Same handling as g1_gait_shaper_node.
        RCLCPP_FATAL(rclcpp::get_logger("g1_lowstate_joint_states"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
