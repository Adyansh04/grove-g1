/**
 * @file catalog.cpp
 * @brief Loading catalog.yaml and serving it as JSON.
 */

#include "g1_orchestration/catalog.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <behaviortree_cpp/contrib/json.hpp>
#include <cmath>
#include <cstdint>
#include <format>
#include <fstream>
#include <iterator>
#include <ranges>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/sha256.hpp"

namespace g1_orchestration
{

namespace
{

constexpr std::size_t kVersionDigits = 12;

const std::set<std::string, std::less<>>& knownRisks()
{
    static const std::set<std::string, std::less<>> risks{
        "observe",
        "world_edit",
        "motion",
        "manipulation",
    };
    return risks;
}

const std::set<std::string, std::less<>>& knownResources()
{
    static const std::set<std::string, std::less<>> resources{
        kBaseResource,
        kLeftArmResource,
        kRightArmResource,
    };
    return resources;
}

const std::set<std::string, std::less<>>& knownFormats()
{
    static const std::set<std::string, std::less<>> formats{ "",
                                                             "id",
                                                             "phrase",
                                                             "station",
                                                             "number" };
    return formats;
}

const std::set<std::string, std::less<>>& knownTypes()
{
    static const std::set<std::string, std::less<>> types{ "string", "world_id" };
    return types;
}

[[noreturn]] void fail(const std::string& where, const std::string& what)
{
    throw std::runtime_error(std::format("catalog: {}: {}", where, what));
}

YAML::Node required(const YAML::Node& parent, const char* key, const std::string& where)
{
    const YAML::Node node = parent[key];
    if (!node.IsDefined() || node.IsNull())
    {
        fail(where, std::format("missing '{}'", key));
    }
    return node;
}

std::string text(const YAML::Node& parent, const char* key, const std::string& where)
{
    const YAML::Node node = required(parent, key, where);
    if (!node.IsScalar())
    {
        fail(where, std::format("'{}' must be text", key));
    }
    return node.as<std::string>();
}

double number(const YAML::Node& parent, const char* key, const std::string& where)
{
    const YAML::Node node  = required(parent, key, where);
    double           value = 0.0;
    if (!node.IsScalar() || !YAML::convert<double>::decode(node, value) || !std::isfinite(value))
    {
        fail(where, std::format("'{}' must be a number", key));
    }
    return value;
}

std::vector<std::string>
words(const YAML::Node& parent, const char* key, const std::string& where, bool must_exist)
{
    std::vector<std::string> out;
    const YAML::Node         node = parent[key];
    if (!node.IsDefined() || node.IsNull())
    {
        if (must_exist)
        {
            fail(where, std::format("missing '{}'", key));
        }
        return out;
    }
    if (!node.IsSequence())
    {
        fail(where, std::format("'{}' must be a list", key));
    }
    for (const YAML::Node& item : node)
    {
        if (!item.IsScalar())
        {
            fail(where, std::format("'{}' must list text", key));
        }
        out.push_back(item.as<std::string>());
    }
    return out;
}

SkillArg parseArg(const YAML::Node& node, const std::string& skill)
{
    const std::string where = std::format("skill {}", skill);
    if (!node.IsMap())
    {
        fail(where, "every argument must be a mapping");
    }
    SkillArg arg;
    arg.name         = text(node, "name", where);
    arg.description  = text(node, "description", where + " arg " + arg.name);
    arg.type         = node["type"].IsDefined() ? text(node, "type", where) : "string";
    arg.format       = node["format"].IsDefined() ? text(node, "format", where) : "";
    arg.allowed      = words(node, "enum", where + " arg " + arg.name, false);
    arg.default_from = node["default_from"].IsDefined() ? text(node, "default_from", where) : "";
    if (!knownFormats().contains(arg.format))
    {
        fail(where + " arg " + arg.name, "unknown format '" + arg.format + "'");
    }
    if (!knownTypes().contains(arg.type))
    {
        fail(where + " arg " + arg.name, "unknown type '" + arg.type + "'");
    }
    if (arg.format == "number")
    {
        arg.min = number(node, "min", where + " arg " + arg.name);
        arg.max = number(node, "max", where + " arg " + arg.name);
        if (!(arg.min < arg.max))
        {
            fail(where + " arg " + arg.name, "min must be less than max");
        }
    }
    else if (node["min"].IsDefined() || node["max"].IsDefined())
    {
        fail(where + " arg " + arg.name, "only a number takes min and max");
    }
    return arg;
}

Skill parseSkill(const YAML::Node& node)
{
    if (!node.IsMap())
    {
        fail("skills", "every skill must be a mapping");
    }
    Skill skill;
    skill.name              = text(node, "name", "a skill");
    const std::string where = "skill " + skill.name;
    skill.description       = text(node, "description", where);
    skill.macro             = text(node, "template", where);
    skill.risk              = text(node, "risk", where);
    skill.idempotent        = required(node, "idempotent", where).as<bool>();
    skill.max_duration_s    = required(node, "max_duration_s", where).as<double>();
    skill.preconditions     = words(node, "requires", where, true);
    skill.effects           = words(node, "effects", where, true);
    skill.resources         = words(node, "resources", where, true);

    const YAML::Node args = required(node, "args", where);
    if (!args.IsSequence())
    {
        fail(where, "'args' must be a list");
    }
    for (const YAML::Node& arg : args)
    {
        skill.args.push_back(parseArg(arg, skill.name));
    }

    if (!knownRisks().contains(skill.risk))
    {
        fail(where, "unknown risk '" + skill.risk + "'");
    }
    for (const std::string& resource : skill.resources)
    {
        if (!knownResources().contains(resource))
        {
            fail(where, "unknown resource '" + resource + "'");
        }
    }
    if (!std::isfinite(skill.max_duration_s) || skill.max_duration_s <= 0.0)
    {
        fail(where, "max_duration_s must be positive");
    }
    return skill;
}

nlohmann::ordered_json toJson(const SkillArg& arg)
{
    nlohmann::ordered_json json;
    json["name"]        = arg.name;
    json["type"]        = arg.type;
    json["description"] = arg.description;
    json["enum"]        = arg.allowed;
    if (arg.format == "number")
    {
        json["format"] = arg.format;
        json["min"]    = arg.min;
        json["max"]    = arg.max;
    }
    if (!arg.default_from.empty())
    {
        json["default_from"] = arg.default_from;
    }
    return json;
}

nlohmann::ordered_json toJson(const Skill& skill)
{
    nlohmann::ordered_json json;
    json["name"]        = skill.name;
    json["description"] = skill.description;
    json["args"]        = nlohmann::ordered_json::array();
    for (const SkillArg& arg : skill.args)
    {
        json["args"].push_back(toJson(arg));
    }
    json["requires"]   = skill.preconditions;
    json["effects"]    = skill.effects;
    json["resources"]  = skill.resources;
    json["idempotent"] = skill.idempotent;
    json["risk"]       = skill.risk;
    // A whole number of seconds reads as one, not as 420.0.
    if (std::floor(skill.max_duration_s) == skill.max_duration_s)
    {
        json["max_duration_s"] = static_cast<std::int64_t>(skill.max_duration_s);
    }
    else
    {
        json["max_duration_s"] = skill.max_duration_s;
    }
    json["template"] = skill.macro;
    return json;
}

}  // namespace

bool Skill::needsArms() const
{
    return std::ranges::any_of(resources, [](const std::string& resource) {
        return resource == kLeftArmResource || resource == kRightArmResource;
    });
}

Catalog Catalog::load(const std::filesystem::path& file)
{
    std::ifstream in(file);
    if (!in)
    {
        throw std::runtime_error("catalog: cannot read " + file.string());
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    return parse(buffer.str());
}

Catalog Catalog::parse(std::string_view yaml_text)
{
    YAML::Node root;
    try
    {
        root = YAML::Load(std::string(yaml_text));
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error(std::string("catalog: not valid YAML: ") + e.what());
    }
    const YAML::Node skills = root["skills"];
    if (!skills.IsDefined() || !skills.IsSequence() || skills.size() == 0)
    {
        fail("file", "needs a non-empty 'skills' list");
    }

    Catalog catalog;
    catalog.text_sha256_ = sha256Hex(yaml_text);
    catalog.version_     = catalog.text_sha256_.substr(0, kVersionDigits);
    std::set<std::string> names;
    std::set<std::string> macros;
    for (const YAML::Node& node : skills)
    {
        Skill skill = parseSkill(node);
        if (!names.insert(skill.name).second)
        {
            fail("skill " + skill.name, "listed twice");
        }
        if (!macros.insert(skill.macro).second)
        {
            fail("skill " + skill.name, "shares macro '" + skill.macro + "' with another skill");
        }
        catalog.skills_.push_back(std::move(skill));
    }
    return catalog;
}

void Catalog::restrict(std::string_view choice)
{
    const auto equals = choice.find('=');
    const auto dot    = choice.substr(0, equals).find('.');
    if (equals == std::string_view::npos || dot == std::string_view::npos)
    {
        fail("arg_choices", std::format("'{}' is not Skill.arg=a|b", choice));
    }
    const std::string_view   skill_name = choice.substr(0, dot);
    const std::string_view   arg_name   = choice.substr(dot + 1, equals - dot - 1);
    std::vector<std::string> values;
    for (const auto part : std::views::split(choice.substr(equals + 1), '|'))
    {
        if (!std::ranges::empty(part))
        {
            values.emplace_back(std::ranges::data(part), std::ranges::size(part));
        }
    }
    if (values.empty())
    {
        fail("arg_choices", std::format("'{}' names no values", choice));
    }
    for (Skill& skill : skills_)
    {
        const auto arg = std::ranges::find(skill.args, arg_name, &SkillArg::name);
        if (skill.name == skill_name && arg != skill.args.end())
        {
            arg->allowed = std::move(values);
            text_sha256_ = sha256Hex(text_sha256_ + "\n" + std::string(choice));
            return;
        }
    }
    fail("arg_choices", std::format("no argument {}.{}", skill_name, arg_name));
}

void Catalog::bindPalette(std::string_view palette_xml)
{
    version_ = sha256Hex(text_sha256_ + "\n" + std::string(palette_xml)).substr(0, kVersionDigits);
}

const Skill* Catalog::findByMacro(std::string_view macro) const
{
    for (const Skill& skill : skills_)
    {
        if (skill.macro == macro)
        {
            return &skill;
        }
    }
    return nullptr;
}

std::string Catalog::json() const
{
    nlohmann::ordered_json json;
    json["catalog_version"] = version_;
    json["skills"]          = nlohmann::ordered_json::array();
    for (const Skill& skill : skills_)
    {
        json["skills"].push_back(toJson(skill));
    }
    return json.dump();
}

}  // namespace g1_orchestration
