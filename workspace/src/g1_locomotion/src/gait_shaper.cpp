/**
 * @file gait_shaper.cpp
 * @brief Reduction of a planner's Twist onto the gait's achievable motions.
 */
#include "g1_locomotion/gait_shaper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_locomotion
{

GaitShaper::GaitShaper(const Config& config)
  : config_(config)
{
    // Checked here rather than in the node that reads the YAML, because shape() is what relies
    // on it: std::clamp below is undefined when lo > hi, which a negative yaw_clamp produces.
    // Enforcing it in one caller left the class accepting a config its own body cannot handle.
    if (config_.fwd_engage < 0.0 || config_.rev_engage <= 0.0 || config_.yaw_engage <= 0.0 ||
        config_.yaw_clamp < 0.0)
    {
        throw std::invalid_argument(
            "GaitShaper: fwd_engage and yaw_clamp must be >= 0 and yaw_engage > 0. A negative "
            "yaw_clamp inverts the clamp bounds, and a negative fwd_engage lets reverse "
            "through -- removing the backstop the navigation trees rely on.");
    }
    // Not merely a bad tuning value: it makes the third motion this class exists to produce
    // unreachable. Every turn that clears yaw_engage is then clamped back under it, so the
    // shaper emits a turn the gait cannot step for.
    if (config_.yaw_clamp < config_.yaw_engage)
    {
        throw std::invalid_argument(
            "GaitShaper: yaw_clamp must be >= yaw_engage, or every turn this class accepts is "
            "clamped back below the threshold that accepted it and the robot never rotates.");
    }
    if (config_.lat_engage <= 0.0 || config_.lat_clamp < 0.0)
    {
        throw std::invalid_argument("GaitShaper: lat_engage must be > 0 and lat_clamp >= 0.");
    }
    if (config_.lat_clamp < config_.lat_engage)
    {
        throw std::invalid_argument(
            "GaitShaper: lat_clamp must be >= lat_engage, for the same reason yaw_clamp must "
            "clear yaw_engage: otherwise every strafe this class accepts is clamped back under "
            "the threshold that accepted it and the robot never steps sideways.");
    }
}

GaitShaper::Command GaitShaper::shape(const Command& in) const
{
    if (std::abs(in.vyaw) >= config_.yaw_engage)
    {
        // std::clamp, not copysign(min(abs)): the constructor guarantees yaw_clamp >= 0, so the
        // bounds are ordered and this says what it means.
        return Command{ 0.0, 0.0, std::clamp(in.vyaw, -config_.yaw_clamp, config_.yaw_clamp) };
    }
    if (in.vx >= config_.fwd_engage)
    {
        return Command{ in.vx, 0.0, 0.0 };
    }
    // Reverse, at its own higher threshold. Everything between -rev_engage and fwd_engage is
    // still a stop, which is where a planner's backup speeds live.
    //
    // Finiteness is checked HERE and not on the forward branch because an infinity fails
    // `>= fwd_engage` on its own only in the positive direction: -inf satisfies that comparison
    // perfectly well and would otherwise reach the gait as a real reverse command.
    if (std::isfinite(in.vx) && in.vx <= -config_.rev_engage)
    {
        return Command{ in.vx, 0.0, 0.0 };
    }
    // Last, so a command carrying anything else never becomes a strafe. The gait's measured
    // response to mixed commands is bad enough that the primitives have to stay exclusive.
    if (std::abs(in.vy) >= config_.lat_engage)
    {
        return Command{ 0.0, std::clamp(in.vy, -config_.lat_clamp, config_.lat_clamp), 0.0 };
    }
    return Command{ 0.0, 0.0, 0.0 };
}

}  // namespace g1_locomotion
