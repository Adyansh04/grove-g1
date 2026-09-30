#ifndef G1_ORCHESTRATION__XML_CANONICAL_HPP_
#define G1_ORCHESTRATION__XML_CANONICAL_HPP_

/**
 * @file xml_canonical.hpp
 * @brief One text form for an XML element, so two copies of a macro can be compared.
 */

#include <tinyxml2.h>

#include <string>
#include <vector>

namespace g1_orchestration
{

/**
 * @brief Prints @p element compactly, with attributes sorted and comments dropped.
 *
 * Whitespace and attribute order carry no meaning in a behavior tree, and a model that copies a
 * macro in may reformat it; the words are what must match.
 *
 * @param element The element to print, with everything under it.
 * @return The canonical text.
 */
std::string canonicalXml(const tinyxml2::XMLElement& element);

/**
 * @brief The tag of @p element and of everything under it, in document order.
 *
 * @param element The element to walk; its own tag comes first.
 * @return The tags, repeated as often as they occur.
 */
std::vector<std::string> tagsUnder(const tinyxml2::XMLElement& element);

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__XML_CANONICAL_HPP_
