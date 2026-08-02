#ifndef G1_LOCOMOTION__VELOCITY_GATE_HPP_
#define G1_LOCOMOTION__VELOCITY_GATE_HPP_

/**
 * @file velocity_gate.hpp
 * @brief Pure, ROS-free velocity re-issue and locomotion-authority state machine.
 */

#include <chrono>
#include <cstdint>
#include <optional>

namespace g1_locomotion
{

/**
 * @brief Locomotion (velocity-command) authority held by this bridge.
 *
 * Values match g1_msgs::msg::LocoStatus's authority constants exactly (RELEASED=0,
 * ACQUIRING=1, HELD=2, RELEASING=3) so the node can static_cast directly when publishing status.
 */
enum class LocoAuthority : std::uint8_t
{
    kReleased  = 0,
    kAcquiring = 1,
    kHeld      = 2,
    kReleasing = 3,
};

/**
 * @brief Velocity re-issue and locomotion-authority state engine.
 */
class VelocityGate
{
public:
    /**
     * @brief Tunables -- see the package README's param table for defaults/provenance.
     */
    struct Config
    {
        double cmd_vel_timeout_s{ 0.5 };
        int    failure_streak_limit{ 3 };
    };

    /**
     * @brief One tick's commanded velocity, returned by tick() when it should be sent.
     */
    struct Intent
    {
        double vx;
        double vy;
        double vyaw;
    };

    explicit VelocityGate(const Config& config);

    /**
     * @brief Latches the newest commanded velocity -- called from the cmd_vel subscription
     * callback. Stored unconditionally regardless of authority state; only tick() decides
     * whether it's ever acted on (cmd_vel is honoured only in kHeld).
     * @param vx    Forward velocity, m/s (already clamped/signed by the caller).
     * @param vy    Lateral velocity, m/s (already clamped/signed by the caller).
     * @param vyaw  Yaw rate, rad/s (already clamped/signed by the caller).
     * @param now   Arrival time, used later to judge staleness.
     */
    void setCommand(double vx, double vy, double vyaw, std::chrono::steady_clock::time_point now);

    /// Call when a SetLocoMode(START) goal's SET_FSM_ID request is about to be sent.
    void beginAcquire();
    /// Call with that request's outcome -- kHeld on success, kReleased on failure. Either way
    /// this is a defined terminal state, never left stuck in kAcquiring.
    void onAcquireResult(bool success);
    /// Call when a SetLocoMode(DAMP) goal's SET_FSM_ID request is about to be sent (only
    /// meaningful from kHeld).
    void beginRelease();
    /// Call with that request's outcome -- always ends at kReleased regardless of success or
    /// failure: a release that failed to confirm on the wire still isn't a state worth
    /// continuing to command velocity from.
    void onReleaseResult();
    /// Unconditional, immediate release -- used by the rogue-publisher advisory guard and by
    /// on_deactivate. Does not send anything itself; the caller is responsible for any
    /// defensive stop request.
    void forceRelease();

    /**
     * @brief One re-issue-timer tick.
     *
     * Returns nothing outside kHeld. Otherwise: a stale (older than cmd_vel_timeout_s, or never
     * received) or all-zero command sends exactly one (0, 0, 0) intent, then nothing further
     * until a fresh non-zero command arrives (duration alone can't be a dead-man once re-issuing
     * itself has stopped); a live non-zero command is returned every tick, for as long as tick()
     * keeps being called -- continuous re-issue is what keeps duration's short window safe.
     * @param now  Current time.
     * @return The velocity to send this tick, or nullopt if nothing should be sent.
     */
    std::optional<Intent> tick(std::chrono::steady_clock::time_point now);

    /**
     * @brief Feeds back the outcome of the SetVelocity request tick() most recently produced.
     *
     * A zero code resets the consecutive-failure streak; a non-zero code advances it and, at
     * failure_streak_limit, releases authority (kHeld -> kReleased).
     * @param error_code  The correlator's reported outcome (0, a LocoClient wire error, or a
     *   sweep() timeout code).
     */
    void onVelocityResult(std::int32_t error_code);

    LocoAuthority authority() const noexcept { return authority_; }
    int           failureStreak() const noexcept { return failure_streak_; }
    std::int32_t  lastErrorCode() const noexcept { return last_error_code_; }

private:
    Config        config_;
    LocoAuthority authority_{ LocoAuthority::kReleased };

    double                                vx_{ 0.0 };
    double                                vy_{ 0.0 };
    double                                vyaw_{ 0.0 };
    std::chrono::steady_clock::time_point last_cmd_arrival_{
        std::chrono::steady_clock::time_point::min()
    };
    bool stopped_once_{ false };

    int          failure_streak_{ 0 };
    std::int32_t last_error_code_{ 0 };
};

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__VELOCITY_GATE_HPP_
