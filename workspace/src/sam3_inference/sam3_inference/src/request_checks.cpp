#include "sam3_inference/request_checks.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>

namespace sam3_inference
{

std::optional<std::string> checkPrompts(std::span<const std::string> prompts)
{
    if (prompts.empty())
    {
        return "no prompts";
    }
    for (std::size_t i = 0; i < prompts.size(); ++i)
    {
        // SAM 3 runs an empty text too and then scores arbitrary objects.
        const bool blank = std::ranges::all_of(prompts[i], [](char c) {
            return std::isspace(static_cast<unsigned char>(c)) != 0;
        });
        if (blank)
        {
            return std::format("prompt {} is empty", i);
        }
    }
    return std::nullopt;
}

std::optional<double>
imageAgeS(std::int32_t stamp_sec, std::uint32_t stamp_nanosec, double now_s) noexcept
{
    if (stamp_sec < 0)
    {
        return std::nullopt;
    }
    return now_s - (static_cast<double>(stamp_sec) + (static_cast<double>(stamp_nanosec) * 1e-9));
}

}  // namespace sam3_inference
