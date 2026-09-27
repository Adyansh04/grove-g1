#ifndef CANOPY_PERCEPTION__PHRASE_HPP_
#define CANOPY_PERCEPTION__PHRASE_HPP_

/**
 * @file phrase.hpp
 * @brief Detector phrases as ids.
 */

#include <string>
#include <string_view>

namespace canopy_perception
{

/// Turns a noun phrase into the id it is published under: "Red Block" is red_block.
[[nodiscard]] std::string slugify(std::string_view phrase);

}  // namespace canopy_perception

#endif  // CANOPY_PERCEPTION__PHRASE_HPP_
