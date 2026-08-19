/**
 * @file g1_safety_controller.cpp
 * @brief Chainable blend and slew stage between a policy and the rt/lowcmd component.
 */

#include "g1_controllers/g1_safety_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

#include "g1_controllers/interface_naming.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace g1_controllers
{
namespace
{

/// Reference interfaces start as NaN, meaning no upstream controller wrote them this tick. Also
/// true for an infinity, which is not a value we can use either, and which would otherwise
/// reach the integrator, where `inf - inf` makes integrated_position_ NaN for the rest of the
/// session. Note blend_ratio 0 is no protection: `0.0 * inf` is NaN, not zero.
bool unwritten(double value) { return !std::isfinite(value); }

/// Expands a single-value or per-joint parameter to one entry per joint.
bool expandToJoints(const std::vector<double>& values, std::size_t joints, std::vector<double>& out)
{
    if (values.size() == 1)
    {
        out.assign(joints, values.front());
        return true;
    }
    if (values.size() == joints)
    {
        out = values;
        return true;
    }
    return false;
}

}  // namespace

double blendAndSlew(
    double activation, double commanded, double blend_ratio, double integrated, double max_velocity,
    double dt) noexcept
{
    const double blended = activation + (blend_ratio * (commanded - activation));
    const double bound =
        max_velocity > 0.0 ? max_velocity * dt : std::numeric_limits<double>::infinity();
    return integrated + std::clamp(blended - integrated, -bound, bound);
}

controller_interface::CallbackReturn G1SafetyController::on_init()
{
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("kp", { 100.0 });
    auto_declare<std::vector<double>>("kd", { 2.0 });
    auto_declare<std::vector<double>>("max_velocity", { -1.0 });
    auto_declare<double>("blend_ratio", 0.0);
    auto_declare<double>("max_blend_ratio_speed", 1.0);
    auto_declare<double>("mean_velocity_limit", 10.0);
    auto_declare<double>("max_velocity_limit", 35.0);
    auto_declare<std::string>("emergency_controller", "");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1SafetyController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    const auto logger = get_node()->get_logger();
    joint_names_      = get_node()->get_parameter("joints").as_string_array();
    if (joint_names_.empty())
    {
        RCLCPP_ERROR(logger, "no joints given, nothing to blend");
        return controller_interface::CallbackReturn::ERROR;
    }

    const auto n = joint_names_.size();
    if (!expandToJoints(get_node()->get_parameter("kp").as_double_array(), n, fallback_kp_) ||
        !expandToJoints(get_node()->get_parameter("kd").as_double_array(), n, fallback_kd_) ||
        !expandToJoints(
            get_node()->get_parameter("max_velocity").as_double_array(),
            n,
            max_velocity_))
    {
        RCLCPP_ERROR(logger, "kp, kd and max_velocity must have 1 or %zu entries", n);
        return controller_interface::CallbackReturn::ERROR;
    }

    max_blend_ratio_speed_ = get_node()->get_parameter("max_blend_ratio_speed").as_double();
    if (max_blend_ratio_speed_ <= 0.0)
    {
        RCLCPP_ERROR(logger, "max_blend_ratio_speed must be > 0, else the blend never arrives");
        return controller_interface::CallbackReturn::ERROR;
    }
    target_blend_ratio_.store(
        std::clamp(get_node()->get_parameter("blend_ratio").as_double(), 0.0, 1.0));

    mean_velocity_limit_  = get_node()->get_parameter("mean_velocity_limit").as_double();
    max_velocity_limit_   = get_node()->get_parameter("max_velocity_limit").as_double();
    emergency_controller_ = get_node()->get_parameter("emergency_controller").as_string();

    activation_position_.assign(n, 0.0);
    integrated_position_.assign(n, 0.0);

    parameter_callback_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto& parameter : parameters)
            {
                if (parameter.get_name() != "blend_ratio")
                {
                    continue;
                }
                const double value = parameter.as_double();
                if (value < 0.0 || value > 1.0)
                {
                    result.successful = false;
                    result.reason     = "blend_ratio must be within [0, 1]";
                    continue;
                }
                target_blend_ratio_.store(value);
            }
            return result;
        });

    if (!emergency_controller_.empty())
    {
        switch_client_ = get_node()->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
G1SafetyController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size() * kInterfacesPerJoint);
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
G1SafetyController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type           = controller_interface::interface_configuration_type::INDIVIDUAL;
    const auto positions  = suffixed(joint_names_, hardware_interface::HW_IF_POSITION);
    const auto velocities = suffixed(joint_names_, hardware_interface::HW_IF_VELOCITY);
    config.names.reserve(positions.size() + velocities.size());
    config.names.insert(config.names.end(), positions.begin(), positions.end());
    config.names.insert(config.names.end(), velocities.begin(), velocities.end());
    return config;
}

std::vector<hardware_interface::CommandInterface>
G1SafetyController::on_export_reference_interfaces()
{
    // Sized here rather than in on_configure: the handles below hold pointers into this vector,
    // so any later resize would leave them dangling.
    reference_interfaces_.assign(
        joint_names_.size() * kInterfacesPerJoint,
        std::numeric_limits<double>::quiet_NaN());

    // Names follow the upstream `<controller>/<joint>/<type>_raw`, so a third-party controller
    // chains onto this unchanged.
    std::vector<hardware_interface::CommandInterface> refs;
    refs.reserve(joint_names_.size() * kInterfacesPerJoint);

    const std::string name = get_node()->get_name();
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        const std::size_t base = i * kInterfacesPerJoint;
        refs.emplace_back(
            name,
            joint_names_[i] + "/position_raw",
            &reference_interfaces_[base + kPosition]);
        refs.emplace_back(
            name,
            joint_names_[i] + "/velocity_raw",
            &reference_interfaces_[base + kVelocity]);
        refs.emplace_back(
            name,
            joint_names_[i] + "/effort_raw",
            &reference_interfaces_[base + kEffort]);
        refs.emplace_back(name, joint_names_[i] + "/kp_raw", &reference_interfaces_[base + kKp]);
        refs.emplace_back(name, joint_names_[i] + "/kd_raw", &reference_interfaces_[base + kKd]);
    }
    return refs;
}

controller_interface::CallbackReturn
G1SafetyController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    const auto logger = get_node()->get_logger();
    if (!indexInterfaces(
            logger,
            suffixed(joint_names_, hardware_interface::HW_IF_POSITION),
            state_interfaces_,
            position_state_indices_) ||
        !indexInterfaces(
            logger,
            suffixed(joint_names_, hardware_interface::HW_IF_VELOCITY),
            state_interfaces_,
            velocity_state_indices_) ||
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

    // Both ramps start at the measured pose, so activating never steps a joint.
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        const auto position = state_interfaces_[position_state_indices_[i]].get_optional();
        if (!position.has_value())
        {
            RCLCPP_ERROR(logger, "joint '%s' has no position", joint_names_[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        activation_position_[i] = position.value();
        integrated_position_[i] = position.value();
    }

    std::fill(
        reference_interfaces_.begin(),
        reference_interfaces_.end(),
        std::numeric_limits<double>::quiet_NaN());
    blend_ratio_           = 0.0;
    emergency_switch_sent_ = false;
    emergency_latched_.store(false);

    if (switch_client_)
    {
        emergency_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(100), [this]() {
            requestEmergencySwitch();
        });
    }

    RCLCPP_INFO(
        get_node()->get_logger(),
        "blending %zu joints in at %.2f/s toward ratio %.2f",
        joint_names_.size(),
        max_blend_ratio_speed_,
        target_blend_ratio_.load());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
G1SafetyController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    emergency_timer_.reset();
    position_state_indices_.clear();
    velocity_state_indices_.clear();
    position_command_indices_.clear();
    velocity_command_indices_.clear();
    effort_command_indices_.clear();
    kp_command_indices_.clear();
    kd_command_indices_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type G1SafetyController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // References arrive only through the exported interfaces; there is no topic path.
    return controller_interface::return_type::OK;
}

bool G1SafetyController::outOfDomain() const
{
    if (mean_velocity_limit_ <= 0.0 && max_velocity_limit_ <= 0.0)
    {
        return false;
    }

    double      sum       = 0.0;
    double      peak      = 0.0;
    std::size_t contribed = 0;
    for (const auto index : velocity_state_indices_)
    {
        const auto velocity = state_interfaces_[index].get_optional();
        if (!velocity.has_value())
        {
            continue;
        }
        // Every comparison below is false against NaN, since std::max returns the accumulator
        // and NaN > limit is false, so without this a robot whose velocities have gone non-finite
        // reads as in-domain. This check fails closed on exactly the input it exists to catch.
        if (!std::isfinite(velocity.value()))
        {
            return true;
        }
        const double magnitude = std::abs(velocity.value());
        sum += magnitude;
        peak = std::max(peak, magnitude);
        ++contribed;
    }
    // Divided by the joints that actually contributed. Counting skipped ones biases the mean low,
    // which weakens the guard precisely when interface access is already degraded.
    if (contribed == 0)
    {
        return true;
    }
    const double mean = sum / static_cast<double>(contribed);

    return (max_velocity_limit_ > 0.0 && peak > max_velocity_limit_) ||
           (mean_velocity_limit_ > 0.0 && mean > mean_velocity_limit_);
}

void G1SafetyController::latchEmergency(const char* reason)
{
    if (emergency_latched_.exchange(true))
    {
        return;
    }
    // const char* rather than std::string: this runs on the 200 Hz path, and building a string
    // from the literal would put a heap allocation in the tick where the policy has just gone
    // out of range.
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "%s -- holding the last safe pose and switching to '%s'",
        reason,
        emergency_controller_.c_str());
}

void G1SafetyController::requestEmergencySwitch()
{
    if (!emergency_latched_.load() || emergency_switch_sent_ || !switch_client_ ||
        !switch_client_->service_is_ready())
    {
        return;
    }
    emergency_switch_sent_ = true;

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers   = { emergency_controller_ };
    request->deactivate_controllers = { get_node()->get_name() };
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_client_->async_send_request(request);
}

controller_interface::return_type G1SafetyController::update_and_write_commands(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
    const double dt = period.seconds();

    if (!emergency_latched_.load() && outOfDomain())
    {
        latchEmergency("joint velocity left the policy's trained range");
    }

    // A latched emergency stops the blend advancing but keeps commanding the last integrated
    // pose, so authority is held until the emergency controller takes over.
    if (!emergency_latched_.load())
    {
        const double target = std::clamp(target_blend_ratio_.load(), 0.0, 1.0);
        const double step   = max_blend_ratio_speed_ * dt;
        blend_ratio_        = std::clamp(target, blend_ratio_ - step, blend_ratio_ + step);
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
        const std::size_t base        = i * kInterfacesPerJoint;
        const double      commanded   = reference_interfaces_[base + kPosition];
        const double      position_in = unwritten(commanded) ? activation_position_[i] : commanded;

        // A latched emergency means the policy is no longer trusted, so nothing it writes is
        // used: the pose stops advancing AND the gains revert to the freeze values. Reading its
        // stiffness here would hold the last safe pose with a diverging policy's gains, updated
        // at 50 Hz, for as long as the switch takes to land.
        if (emergency_latched_.load())
        {
            (void)command_interfaces_[position_command_indices_[i]].set_value(
                integrated_position_[i]);
            (void)command_interfaces_[velocity_command_indices_[i]].set_value(0.0);
            (void)command_interfaces_[effort_command_indices_[i]].set_value(0.0);
            (void)command_interfaces_[kp_command_indices_[i]].set_value(fallback_kp_[i]);
            (void)command_interfaces_[kd_command_indices_[i]].set_value(fallback_kd_[i]);
            continue;
        }

        integrated_position_[i] = blendAndSlew(
            activation_position_[i],
            position_in,
            blend_ratio_,
            integrated_position_[i],
            max_velocity_[i],
            dt);

        const double velocity_in = reference_interfaces_[base + kVelocity];
        const double effort_in   = reference_interfaces_[base + kEffort];
        const double kp_in       = reference_interfaces_[base + kKp];
        const double kd_in       = reference_interfaces_[base + kKd];

        // Velocity and effort scale with the blend; gains pass through, since the policy's own
        // stiffness is what its targets assume.
        (void)command_interfaces_[position_command_indices_[i]].set_value(integrated_position_[i]);
        (void)command_interfaces_[velocity_command_indices_[i]].set_value(
            unwritten(velocity_in) ? 0.0 : blend_ratio_ * velocity_in);
        (void)command_interfaces_[effort_command_indices_[i]].set_value(
            unwritten(effort_in) ? 0.0 : blend_ratio_ * effort_in);
        (void)command_interfaces_[kp_command_indices_[i]].set_value(
            unwritten(kp_in) ? fallback_kp_[i] : kp_in);
        (void)command_interfaces_[kd_command_indices_[i]].set_value(
            unwritten(kd_in) ? fallback_kd_[i] : kd_in);
    }

    return controller_interface::return_type::OK;
}

}  // namespace g1_controllers

PLUGINLIB_EXPORT_CLASS(
    g1_controllers::G1SafetyController, controller_interface::ChainableControllerInterface)
