/**
 * @file xml_canonical.cpp
 * @brief Canonical text for a tinyxml2 element.
 */

#include "g1_orchestration/xml_canonical.hpp"

#include <algorithm>
#include <cctype>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace g1_orchestration
{

namespace
{

void escapeInto(std::string& out, std::string_view text)
{
    for (const char c : text)
    {
        switch (c)
        {
            case '&':
                out += "&amp;";
                break;
            case '<':
                out += "&lt;";
                break;
            case '>':
                out += "&gt;";
                break;
            case '"':
                out += "&quot;";
                break;
            default:
                out += c;
        }
    }
}

std::string_view trimmed(std::string_view text)
{
    const auto blank = [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; };
    while (!text.empty() && blank(text.front()))
    {
        text.remove_prefix(1);
    }
    while (!text.empty() && blank(text.back()))
    {
        text.remove_suffix(1);
    }
    return text;
}

// Depth is bounded by tinyxml2's own nesting limit, so the recursion cannot run away.
// NOLINTNEXTLINE(misc-no-recursion)
void printInto(std::string& out, const tinyxml2::XMLElement& element)
{
    out += '<';
    out += element.Name();
    std::map<std::string, std::string> attributes;
    for (const tinyxml2::XMLAttribute* a = element.FirstAttribute(); a != nullptr; a = a->Next())
    {
        attributes[a->Name()] = a->Value();
    }
    for (const auto& [name, value] : attributes)
    {
        out += ' ';
        out += name;
        out += "=\"";
        escapeInto(out, value);
        out += '"';
    }
    out += '>';
    for (const tinyxml2::XMLNode* child = element.FirstChild(); child != nullptr;
         child                          = child->NextSibling())
    {
        if (const tinyxml2::XMLElement* nested = child->ToElement())
        {
            printInto(out, *nested);
        }
        else if (const tinyxml2::XMLText* text = child->ToText())
        {
            escapeInto(out, trimmed(text->Value()));
        }
    }
    out += "</";
    out += element.Name();
    out += '>';
}

// NOLINTNEXTLINE(misc-no-recursion)
void collectTags(std::vector<std::string>& tags, const tinyxml2::XMLElement& element)
{
    tags.emplace_back(element.Name());
    for (const tinyxml2::XMLElement* child = element.FirstChildElement(); child != nullptr;
         child                             = child->NextSiblingElement())
    {
        collectTags(tags, *child);
    }
}

}  // namespace

std::string canonicalXml(const tinyxml2::XMLElement& element)
{
    std::string out;
    printInto(out, element);
    return out;
}

std::vector<std::string> tagsUnder(const tinyxml2::XMLElement& element)
{
    std::vector<std::string> tags;
    collectTags(tags, element);
    return tags;
}

}  // namespace g1_orchestration
