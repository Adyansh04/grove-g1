#ifndef G1_BRINGUP__LOCO_FSM_HPP_
#define G1_BRINGUP__LOCO_FSM_HPP_

/**
 * @file loco_fsm.hpp
 * @brief Sim-only LocoClient FSM legality table -- the SET_FSM_ID/SET_VELOCITY acceptance rules
 * motion_service_sim's /api/sport/request responder enforces (see that node's README section).
 */

#include <cstdint>

namespace g1_bringup
{

/**
 * @brief FSM ids this table has an opinion about.
 *
 * Squat(2)/Sit(3)/ZeroTorque(0) have no named constant here, deliberately: no edge in the
 * legality table below ever names them as a legal target, so nothing in this file can reference
 * them by accident (mirrors why g1_msgs's SetLocoMode.action leaves them unnamed too).
 */
inline constexpr int kFsmDamp    = 1;
inline constexpr int kFsmStandUp = 4;
inline constexpr int kFsmStart   = 500;

/// LocoClient wire status codes this table produces.
inline constexpr std::int32_t kLocoFsmCodeSuccess = 0;
/// LocoState not available -- SET_VELOCITY requested outside Start.
inline constexpr std::int32_t kLocoFsmCodeLocoStateNotAvailable = 7301;
/// Invalid fsm id -- SET_FSM_ID requested an edge not in the legality table below.
inline constexpr std::int32_t kLocoFsmCodeInvalidFsmId = 7302;

/**
 * @brief Outcome of a SET_FSM_ID(requested) request applied against `current`.
 */
struct SetFsmIdResult
{
    int          new_state;   ///< `current` unchanged on rejection, `requested` on acceptance.
    std::int32_t error_code;  ///< kLocoFsmCodeSuccess or kLocoFsmCodeInvalidFsmId.
};

/**
 * @brief Applies a SET_FSM_ID(requested) transition against `current`, per this sim responder's
 * own legality table:
 *
 *     Damp(1)    -> StandUp(4)
 *     StandUp(4) -> Start(500)
 *     Start(500) -> StandUp(4)
 *     Start(500) -> Damp(1)
 *     StandUp(4) -> Damp(1)
 *
 * Every other edge -- into Squat(2)/Sit(3)/ZeroTorque(0), a same-state no-op, or any pair not
 * listed above -- is rejected.
 *
 * @note The vendor wire contract has no status code specific to "illegal transition" (only 7301
 * "LocoState not available" and 7302 "invalid fsm id"). Reusing 7302 here is this responder's own
 * approximation, not a verified vendor behavior, and stays a hardware re-validation item.
 *
 * @param current    Current FSM state.
 * @param requested  Requested FSM state.
 * @return The resulting state (`current`, unchanged, if rejected) and the wire status code.
 */
SetFsmIdResult applySetFsmId(int current, int requested);

/**
 * @brief SET_VELOCITY's own state gate: legal only from Start(500) -- every other state has no
 * locomotion context for the onboard controller to command a velocity against.
 * @param current  Current FSM state.
 * @return kLocoFsmCodeSuccess in Start, kLocoFsmCodeLocoStateNotAvailable (7301) otherwise.
 */
std::int32_t checkVelocityAllowed(int current);

}  // namespace g1_bringup

#endif  // G1_BRINGUP__LOCO_FSM_HPP_
