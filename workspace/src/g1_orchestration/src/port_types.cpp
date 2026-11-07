/**
 * @file port_types.cpp
 * @brief Parsers for the custom port types a tree writes as strings.
 */

#include "g1_orchestration/port_types.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace
{

/// @p text as a number, refused unless finite: a NaN or an infinity would reach a goal.
double finite(BT::StringView text, const char* what)
{
    const double value = BT::convertFromString<double>(text);
    if (!std::isfinite(value))
    {
        throw BT::RuntimeError(
            std::string(what) + " takes finite numbers, got: ",
            std::string(text));
    }
    return value;
}

}  // namespace

namespace BT
{

template <>
g1_orchestration::Station convertFromString(StringView str)
{
    const std::vector<StringView> parts = splitString(str, ';');
    if (parts.size() != 3)
    {
        throw RuntimeError("a station is 'x;y;yaw', got: ", std::string(str));
    }
    g1_orchestration::Station station;
    station.x   = finite(parts[0], "a station");
    station.y   = finite(parts[1], "a station");
    station.yaw = finite(parts[2], "a station");
    return station;
}

template <>
g1_orchestration::Point3 convertFromString(StringView str)
{
    const std::vector<StringView> parts = splitString(str, ';');
    if (parts.size() != 3)
    {
        throw RuntimeError("a point is 'x;y;z', got: ", std::string(str));
    }
    g1_orchestration::Point3 point;
    point.x = finite(parts[0], "a point");
    point.y = finite(parts[1], "a point");
    point.z = finite(parts[2], "a point");
    return point;
}

}  // namespace BT
