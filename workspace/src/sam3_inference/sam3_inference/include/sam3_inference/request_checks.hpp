#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace sam3_inference
{

/**
 * @brief Why a request's prompts cannot be segmented.
 * @param prompts The request's phrases.
 * @return The reason when the list is empty or a prompt is empty or blank; nullopt when all are
 * usable.
 */
[[nodiscard]] std::optional<std::string> checkPrompts(std::span<const std::string> prompts);

/**
 * @brief Age of an image from its header stamp.
 * @param stamp_sec Seconds of the stamp.
 * @param stamp_nanosec Nanoseconds of the stamp.
 * @param now_s The node clock's current time, seconds.
 * @return Seconds since the stamp (negative for a stamp in the future), or nullopt for a stamp
 * before time zero, which no camera produces.
 */
[[nodiscard]] std::optional<double>
imageAgeS(std::int32_t stamp_sec, std::uint32_t stamp_nanosec, double now_s) noexcept;

}  // namespace sam3_inference
