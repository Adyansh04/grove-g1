/**
 * @file gait_shaper.cpp
 * @brief Reduction of a planner's Twist onto the gait's achievable motions.
 */
#include "g1_locomotion/gait_shaper.hpp"

#include <algorithm>
#include <cmath>

namespace g1_locomotion
{

GaitShaper::GaitShaper(const Config& config)
  : config_(config)
{}

GaitShaper::Command GaitShaper::shape(const Command& in) const
{
    if (std::abs(in.vyaw) >= config_.yaw_engage)
    {
        return Command{ 0.0,
                        0.0,
                        std::copysign(std::min(std::abs(in.vyaw), config_.yaw_clamp), in.vyaw) };
    }
    if (in.vx >= config_.fwd_engage)
    {
        return Command{ in.vx, 0.0, 0.0 };
    }
    return Command{ 0.0, 0.0, 0.0 };
}

}  // namespace g1_locomotion
