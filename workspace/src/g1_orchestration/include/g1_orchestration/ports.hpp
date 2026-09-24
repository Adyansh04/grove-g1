#ifndef G1_ORCHESTRATION__PORTS_HPP_
#define G1_ORCHESTRATION__PORTS_HPP_

/**
 * @file ports.hpp
 * @brief Ports shared by more than one leaf, declared once so Groot2 shows one description.
 */

#include <behaviortree_cpp/basic_types.h>

#include <string>
#include <utility>

namespace g1_orchestration::ports
{

/// A port's name paired with the type information BT needs for it.
using Port = std::pair<std::string, BT::PortInfo>;

/**
 * @brief The `arm` port: which arm a skill should use.
 */
inline Port arm() { return BT::InputPort<std::string>("arm", "right", "'left' or 'right'."); }

/**
 * @brief The `object_id` port: which detected object a skill should act on.
 */
inline Port objectId()
{
    return BT::InputPort<std::string>("object_id", "Must match a class_id published on /objects.");
}

/**
 * @brief The `timeout_s` port sent in the goal, bounding the skill on its server.
 */
inline Port goalTimeout()
{
    return BT::InputPort<double>("timeout_s", 0.0, "0 uses the server's own default.");
}

/**
 * @brief The `timeout_s` port, bounding a blocking service call the leaf makes itself.
 *
 * @param default_s Used when the tree leaves the port unset.
 * @param description Tooltip Groot2 shows for this leaf's timeout.
 */
inline Port serviceTimeout(double default_s, const char* description)
{
    return BT::InputPort<double>("timeout_s", default_s, description);
}

}  // namespace g1_orchestration::ports

#endif  // G1_ORCHESTRATION__PORTS_HPP_
