#ifndef G1_ORCHESTRATION__TEXT_VALUES_HPP_
#define G1_ORCHESTRATION__TEXT_VALUES_HPP_

/**
 * @file text_values.hpp
 * @brief Strict number parsing and word lists, shared by the mission and taught-skill checks.
 */

#include <charconv>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace g1_orchestration
{

/// The fewest and most tries a RetryUntilSuccessful may be given in a mission or a taught skill.
inline constexpr int kMinRetryAttempts = 1;
inline constexpr int kMaxRetryAttempts = 3;

/**
 * @brief Reads all of @p text as a whole number.
 *
 * @return The number, or nothing when @p text has anything else in it (spaces and a leading plus
 *         included).
 */
inline std::optional<std::int64_t> wholeNumber(std::string_view text)
{
    std::int64_t value      = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc() || end != text.data() + text.size())
    {
        return std::nullopt;
    }
    return value;
}

/**
 * @brief Reads all of @p text as a finite real number.
 *
 * @return The number, or nothing when @p text is anything else.
 */
inline std::optional<double> realNumber(std::string_view text)
{
    double value            = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc() || end != text.data() + text.size() || !std::isfinite(value))
    {
        return std::nullopt;
    }
    return value;
}

/// The words of @p words, joined by ", ".
inline std::string join(const std::vector<std::string>& words)
{
    std::string out;
    for (const std::string& word : words)
    {
        out += out.empty() ? "" : ", ";
        out += word;
    }
    return out;
}

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__TEXT_VALUES_HPP_
