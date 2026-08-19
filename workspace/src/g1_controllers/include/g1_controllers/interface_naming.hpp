#ifndef G1_CONTROLLERS__INTERFACE_NAMING_HPP_
#define G1_CONTROLLERS__INTERFACE_NAMING_HPP_

/**
 * @file interface_naming.hpp
 * @brief Interface-name construction and lookup shared by the controllers in this package.
 */

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"

namespace g1_controllers
{

/// Gain interface names the lowcmd component exports. Must match g1_hardware_interface's
/// kHwIfKp/kHwIfKd. The spelling is upstream's, so a third-party controller binds unchanged.
inline constexpr std::string_view kHwIfKp{ "kp" };
inline constexpr std::string_view kHwIfKd{ "kd" };

/// The five command interfaces one joint needs for the component's impedance branch.
inline constexpr std::size_t kInterfacesPerJoint = 5;

/**
 * @brief Builds `<joint>/<type>` for every joint, preserving order.
 *
 * @param joints Joint names.
 * @param type   Interface type, e.g. "position".
 * @return One interface name per joint.
 */
[[nodiscard]] inline std::vector<std::string>
suffixed(const std::vector<std::string>& joints, std::string_view type)
{
    std::vector<std::string> names;
    names.reserve(joints.size());
    for (const auto& joint : joints)
    {
        std::string name = joint;
        name += '/';
        name += type;
        names.push_back(std::move(name));
    }
    return names;
}

/**
 * @brief Locates `names` within `interfaces`, preserving the order of `names`.
 *
 * @param logger     Logger the missing name is reported on.
 * @param names      Interface names to find.
 * @param interfaces Loaned interfaces to search.
 * @param out        Filled with one index per name.
 * @return false if any name is absent, leaving `out` unusable.
 */
template <typename InterfaceT>
[[nodiscard]] bool indexInterfaces(
    const rclcpp::Logger& logger, const std::vector<std::string>& names,
    const std::vector<InterfaceT>& interfaces, std::vector<std::size_t>& out)
{
    out.clear();
    out.reserve(names.size());
    for (const auto& name : names)
    {
        const auto it =
            std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto& iface) {
                return iface.get_name() == name;
            });
        if (it == interfaces.end())
        {
            RCLCPP_ERROR(logger, "interface '%s' was not claimed", name.c_str());
            return false;
        }
        out.push_back(static_cast<std::size_t>(std::distance(interfaces.begin(), it)));
    }
    return true;
}

}  // namespace g1_controllers

#endif  // G1_CONTROLLERS__INTERFACE_NAMING_HPP_
