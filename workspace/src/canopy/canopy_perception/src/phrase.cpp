/**
 * @file phrase.cpp
 * @brief Detector phrases as ids.
 */

#include "canopy_perception/phrase.hpp"

#include <cctype>

namespace canopy_perception
{

std::string slugify(std::string_view phrase)
{
    std::string out;
    out.reserve(phrase.size());
    for (const char character : phrase)
    {
        const auto raw = static_cast<unsigned char>(character);
        if (std::isalnum(raw) != 0)
        {
            out.push_back(static_cast<char>(std::tolower(raw)));
        }
        else if (!out.empty() && out.back() != '_')
        {
            out.push_back('_');
        }
    }
    while (!out.empty() && out.back() == '_')
    {
        out.pop_back();
    }
    return out;
}

}  // namespace canopy_perception
