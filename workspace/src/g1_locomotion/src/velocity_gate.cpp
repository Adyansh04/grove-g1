/**
 * @file velocity_gate.cpp
 * @brief Velocity re-issue and locomotion-authority state machine.
 */
#include "g1_locomotion/velocity_gate.hpp"

namespace g1_locomotion
{

VelocityGate::VelocityGate(const Config& config)
  : config_(config)
{}

void VelocityGate::setCommand(
    double vx, double vy, double vyaw, std::chrono::steady_clock::time_point now)
{
    // Latched regardless; tick() is what decides whether it is ever acted on. Counting the
    // ones that will be thrown away is the only externally visible sign that a publisher is
    // talking to a gate that holds no authority. Zero commands do not count -- Nav2 and teleop
    // both idle at zero, and an idle publisher has not had an intent dropped.
    if (authority_ != LocoAuthority::kHeld && (vx != 0.0 || vy != 0.0 || vyaw != 0.0))
    {
        ++ignored_command_count_;
    }

    vx_               = vx;
    vy_               = vy;
    vyaw_             = vyaw;
    last_cmd_arrival_ = now;
}

void VelocityGate::beginAcquire()
{
    authority_      = LocoAuthority::kAcquiring;
    failure_streak_ = 0;
}

void VelocityGate::onAcquireResult(bool success)
{
    authority_ = success ? LocoAuthority::kHeld : LocoAuthority::kReleased;
    // Fresh hold: judge whatever cmd_vel is currently latched on its own merits (fresh vs. stale,
    // zero vs. not) instead of treating it as already-stopped-once from a previous hold.
    stopped_once_ = false;
}

void VelocityGate::beginRelease() { authority_ = LocoAuthority::kReleasing; }

void VelocityGate::onReleaseResult() { authority_ = LocoAuthority::kReleased; }

void VelocityGate::forceRelease() { authority_ = LocoAuthority::kReleased; }

std::optional<VelocityGate::Intent> VelocityGate::tick(std::chrono::steady_clock::time_point now)
{
    if (authority_ != LocoAuthority::kHeld)
    {
        return std::nullopt;
    }

    const bool never_received = last_cmd_arrival_ == std::chrono::steady_clock::time_point::min();
    const auto timeout        = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config_.cmd_vel_timeout_s));
    const bool stale = never_received || (now - last_cmd_arrival_) > timeout;
    const bool zero  = (vx_ == 0.0) && (vy_ == 0.0) && (vyaw_ == 0.0);

    if (stale || zero)
    {
        if (stopped_once_)
        {
            return std::nullopt;
        }
        stopped_once_ = true;
        return Intent{ 0.0, 0.0, 0.0 };
    }

    stopped_once_ = false;
    return Intent{ vx_, vy_, vyaw_ };
}

void VelocityGate::onVelocityResult(std::int32_t error_code)
{
    last_error_code_ = error_code;
    if (error_code == 0)
    {
        failure_streak_ = 0;
        return;
    }
    ++failure_streak_;
    if (failure_streak_ >= config_.failure_streak_limit && authority_ == LocoAuthority::kHeld)
    {
        authority_ = LocoAuthority::kReleased;
    }
}

}  // namespace g1_locomotion
