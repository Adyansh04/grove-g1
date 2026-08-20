/**
 * @file g1_freeze_controller.cpp
 * @brief Capture-and-hold controller for joints on the rt/lowcmd component.
 */

#include "g1_controllers/g1_freeze_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <string_view>

#include "g1_controllers/interface_naming.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace g1_controllers
{

controller_interface::CallbackReturn G1FreezeController::on_init()
{
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<double>("kp", 0.0);
    auto_declare<double>("kd", 0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1FreezeController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    kp_          = get_node()->get_parameter("kp").as_double();
    kd_          = get_node()->get_parameter("kd").as_double();

    if (joint_names_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "no joints given, nothing to hold");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (kp_ <= 0.0 || kd_ <= 0.0)
    {
        // A freeze with no stiffness is not a freeze, it is a disable with extra steps.
        RCLCPP_ERROR(get_node()->get_logger(), "kp and kd must both be > 0");
        return controller_interface::CallbackReturn::ERROR;
    }

    frozen_positions_.assign(joint_names_.size(), 0.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
G1FreezeController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size() * 5);
    for (const std::string_view type : { std::string_view{ hardware_interface::HW_IF_POSITION },
                                         std::string_view{ hardware_interface::HW_IF_VELOCITY },
                                         std::string_view{ hardware_interface::HW_IF_EFFORT },
                                         kHwIfKp,
                                         kHwIfKd })
    {
        const auto names = suffixed(joint_names_, type);
        config.names.insert(config.names.end(), names.begin(), names.end());
    }
    return config;
}

controller_interface::InterfaceConfiguration
G1FreezeController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type  = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names = suffixed(joint_names_, hardware_interface::HW_IF_POSITION);
    return config;
}

controller_interface::CallbackReturn
G1FreezeController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    const auto logger = get_node()->get_logger();
    if (!indexInterfaces(
            logger,
            suffixed(joint_names_, hardware_interface::HW_IF_POSITION),
            state_interfaces_,
            position_state_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joint_names_, hardware_interface::HW_IF_POSITION),
            command_interfaces_,
            position_command_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joint_names_, hardware_interface::HW_IF_VELOCITY),
            command_interfaces_,
            velocity_command_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joint_names_, hardware_interface::HW_IF_EFFORT),
            command_interfaces_,
            effort_command_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joint_names_, kHwIfKp),
            command_interfaces_,
            kp_command_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joint_names_, kHwIfKd),
            command_interfaces_,
            kd_command_indices_))
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        const auto position = state_interfaces_[position_state_indices_[i]].get_optional();
        if (!position.has_value())
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "joint '%s' has no position to freeze at",
                joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        frozen_positions_[i] = position.value();
    }

    RCLCPP_INFO(get_node()->get_logger(), "holding %zu joints", joint_names_.size());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1FreezeController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    position_state_indices_.clear();
    position_command_indices_.clear();
    velocity_command_indices_.clear();
    effort_command_indices_.clear();
    kp_command_indices_.clear();
    kd_command_indices_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
G1FreezeController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        (void)command_interfaces_[position_command_indices_[i]].set_value(frozen_positions_[i]);
        (void)command_interfaces_[velocity_command_indices_[i]].set_value(0.0);
        (void)command_interfaces_[effort_command_indices_[i]].set_value(0.0);
        (void)command_interfaces_[kp_command_indices_[i]].set_value(kp_);
        (void)command_interfaces_[kd_command_indices_[i]].set_value(kd_);
    }
    return controller_interface::return_type::OK;
}

}  // namespace g1_controllers

PLUGINLIB_EXPORT_CLASS(g1_controllers::G1FreezeController, controller_interface::ControllerInterface)
