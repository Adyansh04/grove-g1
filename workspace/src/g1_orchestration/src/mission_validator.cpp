/**
 * @file mission_validator.cpp
 * @brief The allowlist walk, macro comparison and worst-case arithmetic behind mission checks.
 */

#include "g1_orchestration/mission_validator.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <behaviortree_cpp/contrib/json.hpp>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "g1_orchestration/sha256.hpp"
#include "g1_orchestration/xml_canonical.hpp"

namespace g1_orchestration
{

namespace
{

constexpr std::string_view kMissionTree = "Mission";

constexpr int         kMinAttempts   = 1;
constexpr int         kMaxAttempts   = 3;
constexpr std::size_t kMaxNameLength = 64;
constexpr std::size_t kMaxArgLength  = 200;
constexpr std::size_t kMaxPhrase     = 80;
constexpr double      kMaxStationM   = 1000.0;
/// A little over a full turn, for a yaw written as an angle rather than wrapped.
constexpr double kMaxStationYaw = 2.0 * std::numbers::pi + 1.0;

/// The nodes a Mission tree may use besides SubTree.
bool isControl(std::string_view tag)
{
    return tag == "Sequence" || tag == "Fallback" || tag == "RetryUntilSuccessful" ||
           tag == "Timeout" || tag == "ForceSuccess";
}

/// What to tell a planner about a node it must not write, when it is a likely thing to try.
const char* whyForbidden(std::string_view tag)
{
    if (tag == "Parallel" || tag == "ParallelAll")
    {
        return "Parallel is not allowed: a mission does one step at a time. Use Sequence.";
    }
    if (tag == "ReactiveSequence" || tag == "ReactiveFallback")
    {
        return "Reactive nodes are not allowed: they restart their children every tick, which "
               "would restart a motion half-way. Use Sequence or Fallback.";
    }
    if (tag == "Repeat" || tag == "KeepRunningUntilFailure")
    {
        return "Unbounded loops are not allowed. List the steps, or wrap one in "
               "RetryUntilSuccessful with num_attempts from 1 to 3.";
    }
    if (tag == "Script" || tag == "SetBlackboard")
    {
        return "Scripts and blackboard writes are not allowed in a mission. Pass values to a "
               "skill as its arguments.";
    }
    if (tag == "AcquireArm" || tag == "ReleaseArm")
    {
        return "The executor takes and gives back the arms itself; a mission never does.";
    }
    return nullptr;
}

bool validName(std::string_view name)
{
    return !name.empty() && name.size() <= kMaxNameLength && std::ranges::all_of(name, [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
    });
}

std::string join(const std::vector<std::string>& words)
{
    std::string out;
    for (const std::string& word : words)
    {
        out += out.empty() ? "" : ", ";
        out += word;
    }
    return out;
}

/// The whole of @p text as an integer, or nothing.
std::optional<std::int64_t> wholeNumber(std::string_view text)
{
    std::int64_t value      = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc() || end != text.data() + text.size())
    {
        return std::nullopt;
    }
    return value;
}

std::optional<double> realNumber(std::string_view text)
{
    double value            = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc() || end != text.data() + text.size() || !std::isfinite(value))
    {
        return std::nullopt;
    }
    return value;
}

/// Whether @p value is "x;y;yaw": three finite numbers, the position within the map's reach.
bool isStation(std::string_view value)
{
    const std::size_t first  = value.find(';');
    const std::size_t second = first == std::string_view::npos ? first : value.find(';', first + 1);
    if (second == std::string_view::npos || value.find(';', second + 1) != std::string_view::npos)
    {
        return false;
    }
    const auto x   = realNumber(value.substr(0, first));
    const auto y   = realNumber(value.substr(first + 1, second - first - 1));
    const auto yaw = realNumber(value.substr(second + 1));
    return x && y && yaw && std::abs(*x) <= kMaxStationM && std::abs(*y) <= kMaxStationM &&
           std::abs(*yaw) <= kMaxStationYaw;
}

std::string_view label(const tinyxml2::XMLElement& element)
{
    const char* name = element.Attribute("name");
    return name != nullptr ? name : element.Name();
}

std::vector<const tinyxml2::XMLElement*> childrenOf(const tinyxml2::XMLElement& element)
{
    std::vector<const tinyxml2::XMLElement*> children;
    for (const tinyxml2::XMLElement* child = element.FirstChildElement(); child != nullptr;
         child                             = child->NextSiblingElement())
    {
        children.push_back(child);
    }
    return children;
}

/// Why @p value cannot be this argument, or empty when it can.
std::string problemWith(const SkillArg& arg, std::string_view value)
{
    if (value.empty())
    {
        return "is empty";
    }
    if (value.size() > kMaxArgLength)
    {
        return "is too long";
    }
    if (value.find_first_of("{}") != std::string_view::npos)
    {
        return "must be a plain value, not a reference to something else";
    }
    if (!arg.allowed.empty() && std::ranges::find(arg.allowed, value) == arg.allowed.end())
    {
        return "must be one of: " + join(arg.allowed);
    }
    if (arg.type == "world_id")
    {
        const bool shaped = value.size() >= 2 && value.size() <= kMaxNameLength &&
                            std::isupper(static_cast<unsigned char>(value.front())) != 0 &&
                            std::ranges::all_of(value.substr(1), [](char c) {
                                return std::isdigit(static_cast<unsigned char>(c)) != 0;
                            });
        if (!shaped)
        {
            return "must be an id from the world model: a capital letter and a number, such as "
                   "R2 for a room or O17 for an object";
        }
    }
    else if (arg.format == "id")
    {
        const bool plain = value.size() <= kMaxNameLength && std::ranges::all_of(value, [](char c) {
                               return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
                           });
        if (!plain)
        {
            return "must be an id: letters, digits and underscores, such as O17 or red_block";
        }
    }
    else if (arg.format == "phrase")
    {
        const bool plain = value.size() <= kMaxPhrase && std::ranges::all_of(value, [](char c) {
                               return std::isalnum(static_cast<unsigned char>(c)) != 0 ||
                                      c == ' ' || c == '-' || c == '_' || c == '\'';
                           });
        if (!plain)
        {
            return "must be plain words (letters, digits, spaces, hyphens), at most 80 "
                   "characters, with no commas, equals signs or semicolons";
        }
    }
    else if (arg.format == "station" && !isStation(value))
    {
        return "must be x;y;yaw in the map frame, metres and radians, for example "
               "4.30;-5.60;1.5708";
    }
    else if (arg.format == "number")
    {
        const auto number = realNumber(value);
        if (!number || *number < arg.min || *number > arg.max)
        {
            return std::format("must be a number from {} to {}", arg.min, arg.max);
        }
    }
    return {};
}

/**
 * @brief One validation run: collects diagnostics and the facts the executor needs.
 */
class Checker
{
public:
    Checker(
        const Catalog* catalog, const MacroLibrary* library, const Limits* limits,
        const std::set<std::string, std::less<>>* internal_tags, Validation* out)
      : catalog_(catalog)
      , library_(library)
      , limits_(limits)
      , internal_tags_(internal_tags)
      , out_(out)
    {}

    void run(std::string_view xml, std::optional<std::string_view> expected_sha256)
    {
        out_->sha256 = sha256Hex(xml);
        if (xml.size() > limits_->max_bytes)
        {
            add(code::kTooLarge,
                0,
                "",
                std::format(
                    "The mission is {} bytes; the most allowed is {}.",
                    xml.size(),
                    limits_->max_bytes));
            return;
        }
        if (expected_sha256 && *expected_sha256 != out_->sha256)
        {
            add(code::kHashMismatch,
                0,
                "",
                "tree_sha256 is not the lowercase hex SHA-256 of these exact bytes, so this is "
                "not the mission that was approved. Hash the text as sent, byte for byte.");
        }
        if (xml.find('\0') != std::string_view::npos)
        {
            add(code::kXmlSyntax, 0, "", "The text contains a NUL byte.");
            return;
        }
        if (const std::size_t at = findInclude(xml); at != std::string_view::npos)
        {
            add(code::kIncludeForbidden,
                lineOf(xml, at),
                "include",
                "<include> is not allowed: a mission must carry everything it needs in this one "
                "document.");
        }

        tinyxml2::XMLDocument doc;
        if (doc.Parse(xml.data(), xml.size()) != tinyxml2::XML_SUCCESS)
        {
            add(code::kXmlSyntax,
                doc.ErrorLineNum(),
                "",
                std::string("Not valid XML: ") + doc.ErrorStr());
            return;
        }
        const tinyxml2::XMLElement* root = doc.RootElement();
        if (root == nullptr || !checkRoot(*root))
        {
            return;
        }
        collectTrees(*root);
        if (mission_ == nullptr)
        {
            add(code::kNoMissionTree,
                root->GetLineNum(),
                "root",
                "There is no <BehaviorTree ID=\"Mission\">. The mission to run must be that "
                "tree.");
        }
        else
        {
            checkMission(*mission_);
            if (worst_case_s_ > limits_->max_duration_s)
            {
                add(code::kTooLong,
                    *mission_,
                    std::format(
                        "In the worst case this mission takes {:.0f} s, and the robot allows "
                        "{:.0f} s for one mission. Send it as more than one mission, or take out "
                        "steps or retries.",
                        worst_case_s_,
                        limits_->max_duration_s));
            }
        }
        checkModels();
        if (out_->diagnostics.empty())
        {
            out_->worst_case_s = worst_case_s_;
        }
    }

private:
    void add(const char* diagnostic_code, int line, std::string_view node, std::string message)
    {
        if (static_cast<int>(out_->diagnostics.size()) >= limits_->max_diagnostics)
        {
            if (!truncated_)
            {
                truncated_ = true;
                out_->diagnostics.push_back(
                    { code::kTooManyProblems,
                      0,
                      "",
                      "There are more problems than are listed here. Fix these first." });
            }
            return;
        }
        out_->diagnostics.push_back(
            { diagnostic_code, line, std::string(node), std::move(message) });
    }

    void add(const char* diagnostic_code, const tinyxml2::XMLElement& at, std::string message)
    {
        add(diagnostic_code, at.GetLineNum(), label(at), std::move(message));
    }

    static std::size_t findInclude(std::string_view xml)
    {
        constexpr std::string_view kNeedle = "<include";
        const auto                 found   = std::ranges::search(xml, kNeedle, [](char a, char b) {
            return std::tolower(static_cast<unsigned char>(a)) == b;
        });
        return found.empty() ? std::string_view::npos :
                               static_cast<std::size_t>(found.begin() - xml.begin());
    }

    static int lineOf(std::string_view xml, std::size_t offset)
    {
        return 1 + static_cast<int>(std::count(xml.begin(), xml.begin() + offset, '\n'));
    }

    /// False when there is nothing worth checking further.
    bool checkRoot(const tinyxml2::XMLElement& root)
    {
        if (std::string_view(root.Name()) != "root")
        {
            add(code::kBadRoot,
                root,
                "The document element must be <root BTCPP_format=\"4\" "
                "main_tree_to_execute=\"Mission\">.");
            return false;
        }
        const char* format = root.Attribute("BTCPP_format");
        if (format == nullptr || std::string_view(format) != "4")
        {
            add(code::kBadRoot, root, "<root> needs BTCPP_format=\"4\".");
        }
        const char* main = root.Attribute("main_tree_to_execute");
        if (main == nullptr || std::string_view(main) != kMissionTree)
        {
            add(code::kBadRoot, root, "<root> needs main_tree_to_execute=\"Mission\".");
        }
        for (const tinyxml2::XMLAttribute* a = root.FirstAttribute(); a != nullptr; a = a->Next())
        {
            const std::string_view name = a->Name();
            if (name != "BTCPP_format" && name != "main_tree_to_execute")
            {
                add(code::kUnknownAttribute,
                    root,
                    std::format(
                        "<root> takes only BTCPP_format and main_tree_to_execute, not {}.",
                        name));
            }
        }
        if (root.NextSiblingElement() != nullptr)
        {
            add(code::kBadRoot, *root.NextSiblingElement(), "There must be only one <root>.");
        }
        return true;
    }

    void collectTrees(const tinyxml2::XMLElement& root)
    {
        std::set<std::string> ids;
        for (const tinyxml2::XMLElement* child = root.FirstChildElement(); child != nullptr;
             child                             = child->NextSiblingElement())
        {
            const std::string_view tag = child->Name();
            if (tag == "TreeNodesModel")
            {
                models_.push_back(child);
            }
            else if (tag == "BehaviorTree")
            {
                const char* id = child->Attribute("ID");
                if (id == nullptr)
                {
                    add(code::kMissingAttribute, *child, "<BehaviorTree> needs an ID.");
                }
                else if (!ids.insert(id).second)
                {
                    add(code::kDuplicateTree,
                        *child,
                        std::format("There are two <BehaviorTree ID=\"{}\">.", id));
                }
                else if (id == kMissionTree)
                {
                    mission_ = child;
                }
                else
                {
                    // BehaviorTree.CPP would let a document redefine a registered macro, so this
                    // is the one place a planner could put a leaf of its own.
                    add(code::kExtraTree,
                        child->GetLineNum(),
                        id,
                        std::format(
                            "The document defines <BehaviorTree ID=\"{}\">. A mission holds only "
                            "the Mission tree: the robot supplies the skills itself, so use "
                            "<SubTree ID=\"...\"> for a skill and do not copy it in. The skills "
                            "are: {}.",
                            id,
                            skillNames()));
                }
            }
            else
            {
                add(code::kBadStructure,
                    *child,
                    std::format(
                        "<{}> is not allowed inside <root>. Only <BehaviorTree> and "
                        "<TreeNodesModel> are.",
                        tag));
            }
        }
    }

    void checkMission(const tinyxml2::XMLElement& mission)
    {
        for (const tinyxml2::XMLAttribute* a = mission.FirstAttribute(); a != nullptr; a = a->Next())
        {
            if (std::string_view(a->Name()) != "ID")
            {
                add(code::kUnknownAttribute,
                    mission,
                    std::format("<BehaviorTree> takes only an ID, not {}.", a->Name()));
            }
        }
        const auto children = childrenOf(mission);
        if (children.size() != 1)
        {
            add(code::kBadChildren,
                mission,
                "The Mission tree must hold exactly one node, usually a Sequence of the steps.");
            return;
        }
        worst_case_s_ = walk(*children.front(), 1);
    }

    /// Complains about attributes a node may not carry: `name`, and the one its own tag adds.
    void
    checkAttributes(const tinyxml2::XMLElement& node, std::string_view tag, std::string_view own)
    {
        for (const tinyxml2::XMLAttribute* a = node.FirstAttribute(); a != nullptr; a = a->Next())
        {
            const std::string_view name = a->Name();
            if (name.starts_with('_'))
            {
                add(code::kForbiddenAttribute,
                    node,
                    std::format(
                        "The attribute {} is not allowed: pre and post condition attributes "
                        "(_skipIf, _successIf, _failureIf, _while, _onSuccess, _onFailure, "
                        "_onHalted, _post) and _autoremap have no place in a mission.",
                        name));
            }
            else if (name != "name" && name != own)
            {
                add(code::kUnknownAttribute,
                    node,
                    std::format("{} does not take the attribute {}.", tag, name));
            }
        }
        checkName(node);
    }

    void checkName(const tinyxml2::XMLElement& node)
    {
        const char* name = node.Attribute("name");
        if (name != nullptr && !validName(name))
        {
            add(code::kBadName,
                node,
                "A name is 1 to 64 letters, digits or underscores, such as s1_PickObject.");
        }
    }

    /// The seconds a node can take at most, checking it and everything under it on the way.
    // Depth is capped by Limits::max_depth, so the recursion is bounded.
    // NOLINTNEXTLINE(misc-no-recursion)
    double walk(const tinyxml2::XMLElement& node, int depth)
    {
        if (++nodes_ > limits_->max_nodes)
        {
            if (!too_many_)
            {
                too_many_ = true;
                add(code::kTooManyNodes,
                    node,
                    std::format(
                        "The Mission tree has more than {} nodes. Split the work into fewer "
                        "steps.",
                        limits_->max_nodes));
            }
            return 0.0;
        }
        if (depth > limits_->max_depth)
        {
            if (!too_deep_)
            {
                too_deep_ = true;
                add(code::kTooDeep,
                    node,
                    std::format("The Mission tree nests deeper than {} levels.", limits_->max_depth));
            }
            return 0.0;
        }

        const std::string_view tag = node.Name();
        if (tag == "SubTree")
        {
            return checkSubTree(node);
        }
        const auto children = childrenOf(node);
        if (!isControl(tag))
        {
            rejectTag(node, tag);
        }

        double                multiplier = 1.0;
        std::optional<double> cap_s;
        if (isControl(tag))
        {
            std::string_view own;
            if (tag == "RetryUntilSuccessful")
            {
                own = "num_attempts";
            }
            else if (tag == "Timeout")
            {
                own = "msec";
            }
            checkAttributes(node, tag, own);
            const bool decorator = tag != "Sequence" && tag != "Fallback";
            if (decorator ? children.size() != 1 : children.empty())
            {
                add(code::kBadChildren,
                    node,
                    decorator ? std::format("{} takes exactly one child.", tag) :
                                std::format("{} needs at least one child.", tag));
            }
            if (tag == "RetryUntilSuccessful")
            {
                multiplier = attempts(node);
            }
            else if (tag == "Timeout")
            {
                cap_s = timeoutSeconds(node);
            }
        }

        // A Fallback may try every child and fail through to the last, so its worst case is the
        // sum, not the slowest one: a watchdog set from the smaller figure would fire on a
        // mission that is still working.
        double below = 0.0;
        for (const tinyxml2::XMLElement* child : children)
        {
            below += walk(*child, depth + 1);
        }
        const double cost = multiplier * below;
        return cap_s ? std::min(*cap_s, cost) : cost;
    }

    /// The attempts a Retry allows, or 1 (with a diagnostic) when the attribute is unusable.
    double attempts(const tinyxml2::XMLElement& node)
    {
        const char* text = node.Attribute("num_attempts");
        const auto  n    = text != nullptr ? wholeNumber(text) : std::nullopt;
        if (n && *n >= kMinAttempts && *n <= kMaxAttempts)
        {
            return static_cast<double>(*n);
        }
        add(code::kBadRetries,
            node,
            std::format(
                "num_attempts must be a whole number from {} to {}{}.",
                kMinAttempts,
                kMaxAttempts,
                text == nullptr ? " (it is missing)" : ""));
        return 1.0;
    }

    std::optional<double> timeoutSeconds(const tinyxml2::XMLElement& node)
    {
        const char* text = node.Attribute("msec");
        const auto  msec = text != nullptr ? wholeNumber(text) : std::nullopt;
        if (msec && *msec > 0 && *msec <= std::numeric_limits<int>::max())
        {
            return static_cast<double>(*msec) / 1000.0;
        }
        add(code::kBadTimeout,
            node,
            std::format(
                "msec must be a positive whole number of milliseconds{}.",
                text == nullptr ? " (it is missing)" : ""));
        return std::nullopt;
    }

    void rejectTag(const tinyxml2::XMLElement& node, std::string_view tag)
    {
        if (const char* why = whyForbidden(tag); why != nullptr)
        {
            add(code::kForbiddenNode, node, why);
        }
        else if (internal_tags_->contains(tag))
        {
            add(code::kInternalLeaf,
                node,
                std::format(
                    "{} is an internal node that only the macros may use. Use a skill from the "
                    "catalog as a SubTree.",
                    tag));
        }
        else
        {
            add(code::kUnknownNode,
                node,
                std::format(
                    "{} is not a mission node. A mission is made of Sequence, Fallback, "
                    "RetryUntilSuccessful, Timeout and ForceSuccess around SubTrees of these "
                    "skills: {}.",
                    tag,
                    skillNames()));
        }
    }

    std::string skillNames() const
    {
        std::vector<std::string> names;
        for (const Skill& skill : catalog_->skills())
        {
            names.push_back(skill.macro);
        }
        return join(names);
    }

    double checkSubTree(const tinyxml2::XMLElement& node)
    {
        if (++steps_ > limits_->max_steps && !too_many_steps_)
        {
            too_many_steps_ = true;
            add(code::kTooManySteps,
                node,
                std::format(
                    "The Mission tree has more than {} steps (SubTrees). Send it as more than one "
                    "mission.",
                    limits_->max_steps));
        }
        const char* id = node.Attribute("ID");
        if (id == nullptr)
        {
            add(code::kMissingAttribute, node, "SubTree needs an ID: the skill to run.");
            return 0.0;
        }
        const Skill* skill = catalog_->findByMacro(id);
        if (skill == nullptr)
        {
            add(code::kUnknownMacro,
                node,
                std::format("{} is not a skill. The skills are: {}.", id, skillNames()));
            return 0.0;
        }
        if (node.FirstChildElement() != nullptr)
        {
            add(code::kBadChildren,
                node,
                "A SubTree has no children; give it its arguments as attributes.");
        }
        out_->macros.insert(id);
        const std::string instance =
            node.Attribute("name") != nullptr ? node.Attribute("name") : id;
        if (!step_names_.insert(instance).second)
        {
            add(code::kDuplicateName,
                node,
                std::format("Two steps are named {}. Give each step its own name.", instance));
        }
        out_->steps.emplace(instance, id);
        checkName(node);

        for (const tinyxml2::XMLAttribute* a = node.FirstAttribute(); a != nullptr; a = a->Next())
        {
            const std::string_view name = a->Name();
            if (name == "ID" || name == "name")
            {
                continue;
            }
            if (name.starts_with('_'))
            {
                add(code::kForbiddenAttribute,
                    node,
                    std::format(
                        "The attribute {} is not allowed on a SubTree: no conditions, no "
                        "remapping.",
                        name));
            }
            else if (std::ranges::none_of(skill->args, [&name](const SkillArg& arg) {
                         return arg.name == name;
                     }))
            {
                add(code::kUnknownArg,
                    node,
                    std::format(
                        "{} has no argument {}. Its arguments are: {}.",
                        id,
                        name,
                        argNames(*skill)));
            }
        }
        for (const SkillArg& arg : skill->args)
        {
            const char* value = node.Attribute(arg.name.c_str());
            if (value == nullptr)
            {
                add(code::kMissingArg,
                    node,
                    std::format("{} needs the argument {}: {}", id, arg.name, arg.description));
            }
            else if (const std::string problem = problemWith(arg, value); !problem.empty())
            {
                add(code::kBadArg,
                    node,
                    std::format("The argument {} of {} {}.", arg.name, id, problem));
            }
        }
        return skill->max_duration_s;
    }

    static std::string argNames(const Skill& skill)
    {
        std::vector<std::string> names;
        names.reserve(skill.args.size());
        for (const SkillArg& arg : skill.args)
        {
            names.push_back(arg.name);
        }
        return names.empty() ? "none" : join(names);
    }

    /// A model entry can add defaults to a macro's ports, so it has to be the library's too.
    void checkModels()
    {
        for (const tinyxml2::XMLElement* model : models_)
        {
            for (const tinyxml2::XMLElement* entry : childrenOf(*model))
            {
                const char*  id    = entry->Attribute("ID");
                const Macro* macro = id != nullptr ? library_->find(id) : nullptr;
                if (std::string_view(entry->Name()) != "SubTree" || macro == nullptr ||
                    canonicalXml(*entry) != macro->model)
                {
                    add(code::kModelMismatch,
                        entry->GetLineNum(),
                        id != nullptr ? id : entry->Name(),
                        "<TreeNodesModel> may only hold the <SubTree> entries of this robot's "
                        "macros, copied exactly from the library.");
                }
            }
        }
    }

    const Catalog*                            catalog_;
    const MacroLibrary*                       library_;
    const Limits*                             limits_;
    const std::set<std::string, std::less<>>* internal_tags_;
    Validation*                               out_;

    const tinyxml2::XMLElement*              mission_ = nullptr;
    std::vector<const tinyxml2::XMLElement*> models_;
    std::set<std::string>                    step_names_;
    double                                   worst_case_s_   = 0.0;
    int                                      nodes_          = 0;
    int                                      steps_          = 0;
    bool                                     too_many_       = false;
    bool                                     too_many_steps_ = false;
    bool                                     too_deep_       = false;
    bool                                     truncated_      = false;
};

}  // namespace

std::string stepIdOf(std::string_view name)
{
    if (name.size() < 3 || name.front() != 's')
    {
        return {};
    }
    std::size_t digits = 1;
    while (digits < name.size() && std::isdigit(static_cast<unsigned char>(name[digits])) != 0)
    {
        ++digits;
    }
    if (digits == 1 || digits >= name.size() || name[digits] != '_')
    {
        return {};
    }
    return std::string(name.substr(0, digits));
}

std::vector<MissionStep> missionSteps(const std::string& xml)
{
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS || doc.RootElement() == nullptr)
    {
        return {};
    }
    // The main tree, or the first one when none is named.
    const char*                 main = doc.RootElement()->Attribute("main_tree_to_execute");
    const tinyxml2::XMLElement* tree = doc.RootElement()->FirstChildElement("BehaviorTree");
    while (main != nullptr && tree != nullptr &&
           (tree->Attribute("ID") == nullptr || std::string_view(tree->Attribute("ID")) != main))
    {
        tree = tree->NextSiblingElement("BehaviorTree");
    }
    if (tree == nullptr)
    {
        return {};
    }
    std::vector<MissionStep>                 steps;
    std::vector<const tinyxml2::XMLElement*> pending{ tree };
    while (!pending.empty())
    {
        const tinyxml2::XMLElement* element = pending.back();
        pending.pop_back();
        const char* id   = element->Attribute("ID");
        const char* name = element->Attribute("name");
        if (std::string_view(element->Name()) == "SubTree" && id != nullptr && name != nullptr)
        {
            MissionStep step{ stepIdOf(name), name, id, {} };
            for (const tinyxml2::XMLAttribute* a = element->FirstAttribute(); a != nullptr;
                 a                               = a->Next())
            {
                if (std::string_view(a->Name()) != "ID" && std::string_view(a->Name()) != "name")
                {
                    step.args.emplace(a->Name(), a->Value());
                }
            }
            steps.push_back(std::move(step));
            continue;
        }
        // Children pushed last first, so the stack hands them out in document order.
        std::vector<const tinyxml2::XMLElement*> children;
        for (const tinyxml2::XMLElement* c = element->FirstChildElement(); c != nullptr;
             c                             = c->NextSiblingElement())
        {
            children.push_back(c);
        }
        pending.insert(pending.end(), children.rbegin(), children.rend());
    }
    return steps;
}

std::string withDecimalNumbers(const std::string& xml, const Catalog& catalog)
{
    tinyxml2::XMLDocument doc;
    if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS || doc.RootElement() == nullptr)
    {
        return xml;
    }
    bool                               changed = false;
    std::vector<tinyxml2::XMLElement*> pending{ doc.RootElement() };
    while (!pending.empty())
    {
        tinyxml2::XMLElement* element = pending.back();
        pending.pop_back();
        for (tinyxml2::XMLElement* child = element->FirstChildElement(); child != nullptr;
             child                       = child->NextSiblingElement())
        {
            pending.push_back(child);
        }
        const char* id = element->Attribute("ID");
        if (id == nullptr || std::string_view(element->Name()) != "SubTree")
        {
            continue;
        }
        const Skill* skill = catalog.findByMacro(id);
        if (skill == nullptr)
        {
            continue;
        }
        for (const SkillArg& arg : skill->args)
        {
            const char* value = element->Attribute(arg.name.c_str());
            const auto  number =
                value != nullptr && arg.format == "number" ? realNumber(value) : std::nullopt;
            if (!number)
            {
                continue;
            }
            std::string decimal = std::format("{}", *number);
            if (decimal.find_first_of(".eE") == std::string::npos)
            {
                decimal += ".0";
            }
            element->SetAttribute(arg.name.c_str(), decimal.c_str());
            changed = true;
        }
    }
    if (!changed)
    {
        return xml;
    }
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return printer.CStr();
}

std::string diagnosticsJson(const std::vector<Diagnostic>& diagnostics)
{
    nlohmann::ordered_json array = nlohmann::ordered_json::array();
    for (const Diagnostic& d : diagnostics)
    {
        nlohmann::ordered_json entry;
        entry["code"]    = d.code;
        entry["line"]    = d.line;
        entry["node"]    = d.node;
        entry["message"] = d.message;
        array.push_back(std::move(entry));
    }
    // A tag or a mission id is the client's, and nlohmann throws on a byte that is not UTF-8.
    return array.dump(-1, ' ', false, nlohmann::ordered_json::error_handler_t::replace);
}

MissionValidator::MissionValidator(Catalog catalog, MacroLibrary library, Limits limits)
  : catalog_(std::move(catalog))
  , library_(std::move(library))
  , limits_(limits)
{
    // validate() lets a mission name any catalog skill and the executor then looks its macro up,
    // so the two have to agree here, not only in a test.
    for (const Skill& skill : catalog_.skills())
    {
        if (library_.find(skill.macro) == nullptr)
        {
            throw std::invalid_argument(std::format(
                "the catalog offers {} but the macro library has no macro {}",
                skill.name,
                skill.macro));
        }
    }
    for (const Macro& macro : library_.macros())
    {
        if (catalog_.findByMacro(macro.id) == nullptr)
        {
            throw std::invalid_argument(
                std::format("the macro library has {} but no catalog skill runs it", macro.id));
        }
    }
    for (const Macro& macro : library_.macros())
    {
        for (const std::string& tag : macro.tags)
        {
            if (!isControl(tag))
            {
                internal_tags_.insert(tag);
            }
        }
    }
}

Validation MissionValidator::validate(
    std::string_view xml, std::optional<std::string_view> expected_sha256) const
{
    Validation validation;
    Checker(&catalog_, &library_, &limits_, &internal_tags_, &validation).run(xml, expected_sha256);
    return validation;
}

}  // namespace g1_orchestration
