#ifndef G1_LOCOMOTION__LOCO_PAYLOADS_HPP_
#define G1_LOCOMOTION__LOCO_PAYLOADS_HPP_

/**
 * @file loco_payloads.hpp
 * @brief JSON (de)serialization for the LocoClient wire contract, via nlohmann -- the same
 * library the vendor SDK uses, so our output is byte-identical to BaseClient's own js.dump().
 */

#include <optional>
#include <string>

namespace g1_locomotion
{

/**
 * @brief Fixed duration (seconds) on every outgoing SET_VELOCITY request.
 *
 * The vendor's own Move() latches 864000.0 (10 days) for a "continuous" gait so a caller doesn't
 * have to keep re-issuing -- but that defeats duration's purpose as a dead-man switch: a
 * publisher that dies would leave the robot walking for up to 10 days. This bridge always
 * re-issues instead (see VelocityGate), so every request -- including the stop request -- can
 * safely use the same short duration, and nothing in this package ever needs a longer one.
 */
inline constexpr float kVelocityDurationS = 1.0F;

/**
 * @brief Builds the exact `{"data":<fsm_id>}` JSON body for a 7101 SET_FSM_ID request.
 * @param fsm_id  Target FSM id.
 * @return The JSON-encoded request parameter.
 */
std::string buildSetFsmIdPayload(int fsm_id);

/**
 * @brief Builds the exact JSON body for a 7105 SET_VELOCITY request: `duration` fixed at
 * kVelocityDurationS, `velocity` as `[vx, vy, vyaw]`. No `duration` parameter on purpose: the
 * only way to keep the vendor's 864000 s "continuous move" literal out of this source tree
 * entirely is to never accept a caller-supplied duration in the first place.
 * @param vx    Forward velocity, m/s.
 * @param vy    Lateral velocity, m/s.
 * @param vyaw  Yaw rate, rad/s.
 * @return The JSON-encoded request parameter.
 */
std::string buildSetVelocityPayload(float vx, float vy, float vyaw);

/**
 * @brief Parses the `{"data":<fsm_id>}` body of a 7001 GET_FSM_ID response.
 * @param response_data  The response's `data` field.
 * @return The parsed fsm_id, or nullopt if `response_data` isn't valid JSON or has no integer
 *   `data` field.
 */
std::optional<int> parseFsmIdResponse(const std::string& response_data);

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__LOCO_PAYLOADS_HPP_
