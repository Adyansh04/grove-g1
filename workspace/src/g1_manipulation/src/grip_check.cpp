/**
 * @file grip_check.cpp
 * @brief Reading a grip off the finger joints.
 */

#include "g1_manipulation/grip_check.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <set>

namespace g1_manipulation
{
namespace
{

/// Which finger a joint belongs to, or empty for one that says nothing about a grip.
std::string fingerOf(const std::string& joint)
{
    // thumb_0 rolls the thumb across the palm instead of closing it, so it can stall against its
    // own travel with nothing in the hand.
    if (joint.find("thumb_0") != std::string::npos)
    {
        return {};
    }
    for (const char* finger : { "thumb", "index", "middle" })
    {
        if (joint.find(finger) != std::string::npos)
        {
            return finger;
        }
    }
    return {};
}

bool isLoaded(const JointGrip& joint, double min_error_rad, double min_effort_nm)
{
    if (!std::isfinite(joint.target) || !std::isfinite(joint.position) ||
        !std::isfinite(joint.effort))
    {
        return false;
    }
    return std::abs(joint.target - joint.position) >= min_error_rad &&
           std::abs(joint.effort) >= min_effort_nm;
}

}  // namespace

GripVerdict
verifyGrip(const std::vector<JointGrip>& joints, double min_error_rad, double min_effort_nm)
{
    std::set<std::string> pressing;
    double                worst   = 0.0;
    double                hardest = 0.0;
    for (const JointGrip& joint : joints)
    {
        const std::string finger = fingerOf(joint.name);
        if (finger.empty() || !isLoaded(joint, min_error_rad, min_effort_nm))
        {
            continue;
        }
        pressing.insert(finger);
        worst   = std::max(worst, std::abs(joint.target - joint.position));
        hardest = std::max(hardest, std::abs(joint.effort));
    }

    GripVerdict verdict;
    verdict.holding = pressing.size() >= 2;
    if (verdict.holding)
    {
        verdict.why = std::format(
            "{} fingers are pressing ({:.2f} rad short, {:.2f} Nm)",
            pressing.size(),
            worst,
            hardest);
    }
    else if (pressing.size() == 1)
    {
        verdict.why = "only the " + *pressing.begin() + " is pressing; nothing opposes it";
    }
    else
    {
        verdict.why = "every finger reached its target unloaded";
    }
    return verdict;
}

}  // namespace g1_manipulation
