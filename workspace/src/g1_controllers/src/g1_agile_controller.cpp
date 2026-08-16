/**
 * @file g1_agile_controller.cpp
 * @brief Runs the AGILE velocity policy over the rt/lowcmd component's interfaces.
 */

#include "g1_controllers/g1_agile_controller.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>

#include "g1_controllers/interface_naming.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace g1_controllers
{
namespace
{

/// IMU interfaces the component exports, in the order imu_state_indices_ stores them. Orientation
/// is x,y,z,w here and w,x,y,z in the policy, so the two are not interchangeable.
constexpr std::array<const char*, 7> kImuInterfaces{ "orientation.w",      "orientation.x",
                                                     "orientation.y",      "orientation.z",
                                                     "angular_velocity.x", "angular_velocity.y",
                                                     "angular_velocity.z" };

double clampMagnitude(double value, double limit)
{
    return limit > 0.0 ? std::clamp(value, -limit, limit) : value;
}

}  // namespace

controller_interface::CallbackReturn G1AgileController::on_init()
{
    auto_declare<std::string>("model_path", "");
    auto_declare<std::string>("cmd_vel_topic", "/cmd_vel");
    auto_declare<std::string>("imu_sensor_name", "imu");
    auto_declare<std::string>("command_prefix", "");
    auto_declare<std::string>("command_suffix", "");
    auto_declare<int>("decimation", kPolicyDecimation);
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<double>("max_linear_speed", 0.0);
    auto_declare<double>("max_angular_speed", 0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1AgileController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    const auto logger = get_node()->get_logger();

    model_path_        = get_node()->get_parameter("model_path").as_string();
    cmd_vel_topic_     = get_node()->get_parameter("cmd_vel_topic").as_string();
    imu_sensor_name_   = get_node()->get_parameter("imu_sensor_name").as_string();
    command_prefix_    = get_node()->get_parameter("command_prefix").as_string();
    command_suffix_    = get_node()->get_parameter("command_suffix").as_string();
    decimation_        = static_cast<int>(get_node()->get_parameter("decimation").as_int());
    cmd_vel_timeout_s_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
    max_linear_speed_  = get_node()->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_node()->get_parameter("max_angular_speed").as_double();

    if (model_path_.empty())
    {
        // Default to the policy shipped with this package, so only a retrained one needs a path.
        model_path_ = ament_index_cpp::get_package_share_directory("g1_controllers") +
                      "/policy/unitree_g1_velocity_e2e.onnx";
    }
    if (decimation_ < 1)
    {
        RCLCPP_ERROR(logger, "decimation must be >= 1");
        return controller_interface::CallbackReturn::ERROR;
    }

    const auto update_rate = static_cast<double>(get_update_rate());
    if (update_rate > 0.0)
    {
        const double policy_hz = update_rate / static_cast<double>(decimation_);
        if (std::abs(policy_hz - kPolicyRateHz) > 1.0)
        {
            // The policy's history spacing is baked in at training, so an off-rate loop silently
            // changes what it thinks a timestep is.
            RCLCPP_WARN(
                logger,
                "policy would run at %.1f Hz, not the %.0f Hz it was trained at",
                policy_hz,
                kPolicyRateHz);
        }
    }

    try
    {
        policy_ = std::make_unique<AgilePolicy>(model_path_);
    }
    catch (const std::exception& error)
    {
        RCLCPP_ERROR(logger, "could not load '%s': %s", model_path_.c_str(), error.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    cmd_vel_buffer_.writeFromNonRT(geometry_msgs::msg::Twist{});
    cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::SystemDefaultsQoS(),
        // ConstSharedPtr by reference: the only const-ref callback signature rclcpp accepts, and
        // it avoids a refcount bump per message.
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr& message) {
            cmd_vel_buffer_.writeFromNonRT(*message);
            last_cmd_vel_seconds_.store(get_node()->now().seconds());
        });

    inferring_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
        "~/inferring",
        rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(logger, "loaded '%s'", model_path_.c_str());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration G1AgileController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve((kNumObsJoints * 2) + kImuInterfaces.size());

    for (const auto& joint : kAgileObsJointNames)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    for (const auto& joint : kAgileObsJointNames)
    {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    for (const auto* interface : kImuInterfaces)
    {
        config.names.push_back(imu_sensor_name_ + "/" + interface);
    }
    return config;
}

std::vector<std::string> G1AgileController::commandNamesFor(std::string_view type) const
{
    std::vector<std::string> names;
    names.reserve(kNumActJoints);
    for (const auto& joint : kAgileActionJointNames)
    {
        std::string name;
        if (!command_prefix_.empty())
        {
            name += command_prefix_;
            name += '/';
        }
        name += joint;
        name += '/';
        name += type;
        name += command_suffix_;
        names.push_back(std::move(name));
    }
    return names;
}

controller_interface::InterfaceConfiguration
G1AgileController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(kNumActJoints * kInterfacesPerJoint);

    for (const std::string_view type : { std::string_view{ hardware_interface::HW_IF_POSITION },
                                         std::string_view{ hardware_interface::HW_IF_VELOCITY },
                                         std::string_view{ hardware_interface::HW_IF_EFFORT },
                                         kHwIfKp,
                                         kHwIfKd })
    {
        const auto names = commandNamesFor(type);
        config.names.insert(config.names.end(), names.begin(), names.end());
    }
    return config;
}

bool G1AgileController::resolveInterfaces()
{
    const auto logger = get_node()->get_logger();

    std::vector<std::string> joints(kAgileObsJointNames.begin(), kAgileObsJointNames.end());
    if (!indexInterfaces(
            logger,
            suffixed(joints, hardware_interface::HW_IF_POSITION),
            state_interfaces_,
            position_state_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joints, hardware_interface::HW_IF_VELOCITY),
            state_interfaces_,
            velocity_state_indices_))
    {
        return false;
    }

    std::vector<std::string> imu_names;
    imu_names.reserve(kImuInterfaces.size());
    for (const auto* interface : kImuInterfaces)
    {
        imu_names.push_back(imu_sensor_name_ + "/" + interface);
    }
    if (!indexInterfaces(logger, imu_names, state_interfaces_, imu_state_indices_))
    {
        return false;
    }

    return indexInterfaces(
               logger,
               commandNamesFor(hardware_interface::HW_IF_POSITION),
               command_interfaces_,
               position_command_indices_) &&
           indexInterfaces(
               logger,
               commandNamesFor(hardware_interface::HW_IF_VELOCITY),
               command_interfaces_,
               velocity_command_indices_) &&
           indexInterfaces(
               logger,
               commandNamesFor(hardware_interface::HW_IF_EFFORT),
               command_interfaces_,
               effort_command_indices_) &&
           indexInterfaces(
               logger,
               commandNamesFor(kHwIfKp),
               command_interfaces_,
               kp_command_indices_) &&
           indexInterfaces(
               logger,
               commandNamesFor(kHwIfKd),
               command_interfaces_,
               kd_command_indices_);
}

controller_interface::CallbackReturn
G1AgileController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (!resolveInterfaces())
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    policy_->reset();
    decimation_count_ = 0;
    inferring_        = false;
    cmd_vel_buffer_.writeFromNonRT(geometry_msgs::msg::Twist{});
    last_cmd_vel_seconds_.store(0.0);

    std_msgs::msg::Bool message;
    message.data = false;
    inferring_publisher_->publish(message);

    RCLCPP_INFO(
        get_node()->get_logger(),
        "commanding %zu joints every %d ticks",
        kNumActJoints,
        decimation_);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1AgileController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    position_state_indices_.clear();
    velocity_state_indices_.clear();
    imu_state_indices_.clear();
    position_command_indices_.clear();
    velocity_command_indices_.clear();
    effort_command_indices_.clear();
    kp_command_indices_.clear();
    kd_command_indices_.clear();
    inferring_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
}

void G1AgileController::packObservation()
{
    for (std::size_t i = 0; i < kNumObsJoints; ++i)
    {
        observation_.joint_position.at(i) = static_cast<float>(
            state_interfaces_[position_state_indices_[i]].get_optional().value_or(0.0));
        observation_.joint_velocity.at(i) = static_cast<float>(
            state_interfaces_[velocity_state_indices_[i]].get_optional().value_or(0.0));
    }

    for (std::size_t i = 0; i < 4; ++i)
    {
        observation_.root_quat_wxyz.at(i) = static_cast<float>(
            state_interfaces_[imu_state_indices_[i]].get_optional().value_or(0.0));
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        observation_.root_ang_vel_b.at(i) = static_cast<float>(
            state_interfaces_[imu_state_indices_[4 + i]].get_optional().value_or(0.0));
    }
}

controller_interface::return_type
G1AgileController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    if (++decimation_count_ < decimation_)
    {
        // Command interfaces hold their last value, so the policy's targets stand between ticks.
        return controller_interface::return_type::OK;
    }
    decimation_count_ = 0;

    const auto&  command = *cmd_vel_buffer_.readFromRT();
    const double age     = time.seconds() - last_cmd_vel_seconds_.load();
    const bool   stale   = last_cmd_vel_seconds_.load() <= 0.0 ||
                       (cmd_vel_timeout_s_ > 0.0 && age > cmd_vel_timeout_s_);

    observation_.velocity_command = {
        stale ? 0.0F : static_cast<float>(clampMagnitude(command.linear.x, max_linear_speed_)),
        stale ? 0.0F : static_cast<float>(clampMagnitude(command.linear.y, max_linear_speed_)),
        stale ? 0.0F : static_cast<float>(clampMagnitude(command.angular.z, max_angular_speed_)),
    };

    packObservation();

    if (!policy_->run(observation_, action_))
    {
        RCLCPP_ERROR(get_node()->get_logger(), "inference failed -- the policy is not commanding");
        return controller_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < kNumActJoints; ++i)
    {
        // Absolute radians, and the gains the policy expects them to be held with. Velocity and
        // effort targets are zero, matching the actuator model the policy was trained against.
        (void)command_interfaces_[position_command_indices_[i]].set_value(
            static_cast<double>(action_.joint_position.at(i)));
        (void)command_interfaces_[velocity_command_indices_[i]].set_value(0.0);
        (void)command_interfaces_[effort_command_indices_[i]].set_value(0.0);
        (void)command_interfaces_[kp_command_indices_[i]].set_value(
            static_cast<double>(action_.kp.at(i)));
        (void)command_interfaces_[kd_command_indices_[i]].set_value(
            static_cast<double>(action_.kd.at(i)));
    }

    if (!inferring_)
    {
        inferring_ = true;
        std_msgs::msg::Bool message;
        message.data = true;
        inferring_publisher_->publish(message);
    }
    return controller_interface::return_type::OK;
}

}  // namespace g1_controllers

PLUGINLIB_EXPORT_CLASS(g1_controllers::G1AgileController, controller_interface::ControllerInterface)
