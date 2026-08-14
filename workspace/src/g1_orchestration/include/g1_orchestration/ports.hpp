#ifndef G1_ORCHESTRATION__PORTS_HPP_
#define G1_ORCHESTRATION__PORTS_HPP_

/**
 * @file ports.hpp
 * @brief Ports shared by more than one leaf, declared once.
 *
 * These were repeated character-for-character across four leaves. Declaring them here keeps
 * their descriptions identical too, which matters more than it sounds: the descriptions are
 * what Groot2 shows a tree author as tooltips.
 */

#include <behaviortree_cpp/basic_types.h>

#include <string>
#include <utility>

namespace g1_orchestration::ports
{

using Port = std::pair<std::string, BT::PortInfo>;

inline Port arm() { return BT::InputPort<std::string>("arm", "right", "'left' or 'right'."); }

inline Port objectId()
{
    return BT::InputPort<std::string>("object_id", "Must match a class_id published on /objects.");
}

/// Sent to the SERVER as part of the goal. Distinct from serviceTimeout(): this one bounds the
/// skill, that one bounds a local call, named apart so a tree author does not confuse the two.
inline Port goalTimeout()
{
    return BT::InputPort<double>("timeout_s", 0.0, "0 uses the server's own default.");
}

/// Bounds a blocking service call the leaf makes itself.
inline Port serviceTimeout(double default_s, const char* description)
{
    return BT::InputPort<double>("timeout_s", default_s, description);
}

}  // namespace g1_orchestration::ports

#endif  // G1_ORCHESTRATION__PORTS_HPP_
