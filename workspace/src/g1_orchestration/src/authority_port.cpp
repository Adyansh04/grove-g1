/**
 * @file authority_port.cpp
 * @brief The controller_manager implementation of AuthorityPort.
 */

#include "g1_orchestration/authority_port.hpp"

#include <exception>
#include <utility>

#include "g1_orchestration/arm_authority.hpp"

namespace g1_orchestration
{

ControllerManagerAuthority::ControllerManagerAuthority(rclcpp::Logger logger, double timeout_s)
  : logger_(std::move(logger))
  , timeout_s_(timeout_s)
{}

Acquired ControllerManagerAuthority::acquire() { return acquireArm(logger_, timeout_s_); }

void ControllerManagerAuthority::release()
{
    // Called on the way out of a mission, so an escaping exception would skip what follows.
    try
    {
        releaseArm(logger_, timeout_s_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "the arms may still be acquired: release threw: %s", e.what());
    }
}

}  // namespace g1_orchestration
