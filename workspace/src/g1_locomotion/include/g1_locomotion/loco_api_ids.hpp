#ifndef G1_LOCOMOTION__LOCO_API_IDS_HPP_
#define G1_LOCOMOTION__LOCO_API_IDS_HPP_

/**
 * @file loco_api_ids.hpp
 * @brief LocoClient wire API ids and status codes this bridge sends/interprets.
 */

#include <cstdint>

namespace g1_locomotion
{

/**
 * @brief GET_FSM_ID -- no parameter; response `data` is `{"data": <fsm_id>}`.
 */
inline constexpr std::int64_t kApiIdGetFsmId = 7001;
/**
 * @brief SET_FSM_ID -- parameter `{"data": <fsm_id>}`.
 */
inline constexpr std::int64_t kApiIdSetFsmId = 7101;
/**
 * @brief SET_VELOCITY -- parameter `{"velocity":[vx,vy,vyaw],"duration":d}`.
 */
inline constexpr std::int64_t kApiIdSetVelocity = 7105;

// SET_ARM_TASK (7106) deliberately has no constant here: it crosses into rt/arm_sdk's control
// authority (WaveHand/ShakeHand make the onboard controller move the arms, fighting whatever
// blend weight our rt/arm_sdk publisher currently holds), and no arbitration rule between the two
// paths exists. Leaving it undefined means nothing here can send it by accident.

inline constexpr std::int32_t kCodeSuccess = 0;
/// LocoState not available -- the onboard controller isn't in a state that can service the call.
inline constexpr std::int32_t kCodeLocoStateNotAvailable = 7301;
/// Invalid fsm id (e.g. rejected by the onboard controller's own transition table).
inline constexpr std::int32_t kCodeInvalidFsmId = 7302;

// 7303 (invalid task id) deliberately has no constant here, same reason 7106 SET_ARM_TASK has
// none: this bridge never sends a task-id-bearing request, so it can never receive that specific
// rejection.

/// UT_ROBOT_TASK_TIMEOUT -- sweep()'s own timeout code, matching the vendored BaseClient's value.
inline constexpr std::int32_t kCodeTaskTimeout = -1;
/// UT_ROBOT_TASK_UNKNOWN_ERROR.
inline constexpr std::int32_t kCodeTaskUnknownError = -2;

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__LOCO_API_IDS_HPP_
