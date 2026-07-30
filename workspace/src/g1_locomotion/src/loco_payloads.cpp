/**
 * @file loco_payloads.cpp
 * @brief JSON (de)serialization for the LocoClient wire contract.
 */
#include "g1_locomotion/loco_payloads.hpp"

#include <nlohmann/json.hpp>

namespace g1_locomotion
{

std::string buildSetFsmIdPayload(int fsm_id)
{
    nlohmann::json js;
    js["data"] = fsm_id;
    return js.dump();
}

std::string buildSetVelocityPayload(float vx, float vy, float vyaw)
{
    nlohmann::json js;
    js["velocity"] = { vx, vy, vyaw };
    js["duration"] = kVelocityDurationS;
    return js.dump();
}

std::optional<int> parseFsmIdResponse(const std::string& response_data)
{
    try
    {
        const auto js = nlohmann::json::parse(response_data);
        if (!js.contains("data") || !js["data"].is_number_integer())
        {
            return std::nullopt;
        }
        return js["data"].get<int>();
    }
    catch (const nlohmann::json::parse_error&)
    {
        return std::nullopt;
    }
}

}  // namespace g1_locomotion
