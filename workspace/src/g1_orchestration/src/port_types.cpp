#include "g1_orchestration/port_types.hpp"

#include <string>
#include <vector>

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
    station.x   = convertFromString<double>(parts[0]);
    station.y   = convertFromString<double>(parts[1]);
    station.yaw = convertFromString<double>(parts[2]);
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
    point.x = convertFromString<double>(parts[0]);
    point.y = convertFromString<double>(parts[1]);
    point.z = convertFromString<double>(parts[2]);
    return point;
}

}  // namespace BT
