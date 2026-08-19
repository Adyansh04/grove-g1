#ifndef G1_ORCHESTRATION__PORT_TYPES_HPP_
#define G1_ORCHESTRATION__PORT_TYPES_HPP_

/**
 * @file port_types.hpp
 * @brief Custom port types a tree can write as a string, and their parsers.
 */

#include <behaviortree_cpp/basic_types.h>

namespace g1_orchestration
{

/**
 * @brief A base goal written into the tree XML as "x;y;yaw", in metres and radians.
 */
struct Station
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

/**
 * @brief A point written into the tree XML as "x;y;z", in metres.
 *
 * Distinct from Station rather than reusing its third field: a place target has a height, not
 * a heading.
 */
struct Point3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

}  // namespace g1_orchestration

// Declared in BT's namespace because that is where the library looks the conversion up.
namespace BT
{
/**
 * @brief Parses "x;y;yaw" into a Station.
 */
template <>
g1_orchestration::Station convertFromString(StringView str);

/**
 * @brief Parses "x;y;z" into a Point3.
 */
template <>
g1_orchestration::Point3 convertFromString(StringView str);
}  // namespace BT

#endif  // G1_ORCHESTRATION__PORT_TYPES_HPP_
