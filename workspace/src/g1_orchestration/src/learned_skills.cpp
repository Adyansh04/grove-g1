/**
 * @file learned_skills.cpp
 * @brief Checking, building, keeping and forgetting the skills a planner teaches.
 */

#include "g1_orchestration/learned_skills.hpp"

#include <tinyxml2.h>

#include <algorithm>
#include <behaviortree_cpp/contrib/json.hpp>
#include <cmath>
#include <cstdint>
#include <format>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "g1_orchestration/xml_canonical.hpp"

namespace g1_orchestration
{

namespace
{

constexpr std::size_t kMinNameLength   = 3;
constexpr std::size_t kMaxNameLength   = 40;
constexpr std::size_t kMaxArgs         = 8;
constexpr std::size_t kMaxDescription  = 600;
constexpr std::size_t kMaxArgNameChars = 32;
constexpr std::size_t kMaxBodyBytes    = 16384;
constexpr int         kMaxAttempts     = 3;
/// After an input in `learnable_leaves`: it takes a world-model id.
constexpr std::string_view kWorldIdMark = "(world_id)";

bool isSkillName(std::string_view name)
{
    return name.size() >= kMinNameLength && name.size() <= kMaxNameLength && name.front() >= 'A' &&
           name.front() <= 'Z' && std::ranges::all_of(name, [](char c) {
               return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
           });
}

bool isArgName(std::string_view name)
{
    // `name` is a SubTree's own attribute, its step's name, so it can never be a port.
    return !name.empty() && name != "name" && name.size() <= kMaxArgNameChars &&
           name.front() >= 'a' && name.front() <= 'z' && std::ranges::all_of(name, [](char c) {
               return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
           });
}

/// The argument an attribute names, `{name}`; empty for a literal.
std::string_view boundArg(std::string_view value)
{
    if (value.size() > 2 && value.front() == '{' && value.back() == '}')
    {
        return value.substr(1, value.size() - 2);
    }
    return {};
}

std::optional<std::int64_t> wholeNumber(const char* text)
{
    std::istringstream in(text);
    std::int64_t       n = 0;
    if (in >> n && in.eof())
    {
        return n;
    }
    return std::nullopt;
}

/// Walks a body, collecting its problems and the authority its leaves need.
class BodyCheck
{
public:
    BodyCheck(
        const SkillDefinition& definition, const Learnable& learnable,
        std::vector<Diagnostic>& problems)
      : definition_(&definition)
      , learnable_(&learnable)
      , problems_(&problems)
    {}

    // Depth is capped at max_depth before it recurses, so it cannot run away.
    // NOLINTNEXTLINE(misc-no-recursion)
    void walk(const tinyxml2::XMLElement& node, int depth)
    {
        ++nodes_;
        const std::string_view tag = node.Name();
        if (depth > learnable_->max_depth)
        {
            add(code::kTooDeep,
                node,
                std::format("the body nests deeper than {}", learnable_->max_depth));
            return;
        }
        std::vector<const tinyxml2::XMLElement*> children;
        for (const auto* child = node.FirstChildElement(); child != nullptr;
             child             = child->NextSiblingElement())
        {
            children.push_back(child);
        }
        if (isControlNode(tag))
        {
            control(node, tag, children.size());
        }
        else if (const auto leaf = learnable_->leaves.find(tag); leaf != learnable_->leaves.end())
        {
            if (!children.empty())
            {
                add(code::kBadChildren,
                    node,
                    std::format("{} is a leaf: it takes no children", tag));
            }
            attributes(node, tag);
            if (!leaf->second.empty())
            {
                holds_.insert(leaf->second);
            }
        }
        else
        {
            add(code::kForbiddenNode,
                node,
                std::format(
                    "{} is not one a learned skill may use; it may use Sequence, Fallback, "
                    "RetryUntilSuccessful, Timeout, ForceSuccess and {}",
                    tag,
                    leafList()));
        }
        for (const auto* child : children)
        {
            walk(*child, depth + 1);
        }
    }

    [[nodiscard]] int            nodes() const { return nodes_; }
    [[nodiscard]] const NameSet& holds() const { return holds_; }

private:
    void add(const char* code, const tinyxml2::XMLElement& node, std::string message)
    {
        problems_->push_back(Diagnostic{ code, node.GetLineNum(), node.Name(), std::move(message) });
    }

    std::string leafList() const
    {
        std::string out;
        for (const auto& [leaf, _] : learnable_->leaves)
        {
            out += (out.empty() ? "" : ", ") + leaf;
        }
        return out;
    }

    void control(const tinyxml2::XMLElement& node, std::string_view tag, std::size_t children)
    {
        const bool decorator = tag != "Sequence" && tag != "Fallback";
        if (decorator ? children != 1 : children == 0)
        {
            add(code::kBadChildren,
                node,
                decorator ? std::format("{} takes exactly one child", tag) :
                            std::format("{} needs at least one child", tag));
        }
        const std::string_view own = tag == "RetryUntilSuccessful" ? "num_attempts" :
                                     tag == "Timeout"              ? "msec" :
                                                                     "";
        for (const auto* a = node.FirstAttribute(); a != nullptr; a = a->Next())
        {
            const std::string_view name = a->Name();
            if (name != "name" && name != own)
            {
                add(code::kUnknownAttribute, node, std::format("{} takes no {}", tag, name));
            }
        }
        if (tag == "RetryUntilSuccessful")
        {
            const char* text = node.Attribute("num_attempts");
            const auto  n    = text != nullptr ? wholeNumber(text) : std::nullopt;
            if (!n || *n < 1 || *n > kMaxAttempts)
            {
                add(code::kBadRetries, node, "num_attempts must be a whole number from 1 to 3");
            }
        }
        if (tag == "Timeout")
        {
            const char* text = node.Attribute("msec");
            const auto  msec = text != nullptr ? wholeNumber(text) : std::nullopt;
            if (!msec || *msec <= 0 ||
                static_cast<double>(*msec) > 1000.0 * learnable_->max_duration_s)
            {
                add(code::kBadTimeout,
                    node,
                    std::format(
                        "msec must be a whole number of milliseconds from 1 to {:.0f}",
                        1000.0 * learnable_->max_duration_s));
            }
        }
    }

    void attributes(const tinyxml2::XMLElement& node, std::string_view tag)
    {
        const auto& ports = learnable_->ports.at(std::string(tag));
        for (const auto* a = node.FirstAttribute(); a != nullptr; a = a->Next())
        {
            const std::string_view name  = a->Name();
            const std::string_view value = a->Value();
            if (name == "name")
            {
                continue;
            }
            if (!ports.contains(name))
            {
                add(code::kUnknownAttribute, node, std::format("{} has no input {}", tag, name));
                continue;
            }
            const std::string_view arg = boundArg(value);
            if (!arg.empty())
            {
                const auto known =
                    std::ranges::find_if(definition_->args, [arg](const SkillArg& s) {
                        return s.name == arg;
                    });
                if (known == definition_->args.end())
                {
                    add(code::kBadArg,
                        node,
                        std::format("{{{}}} is not one of the skill's arguments", arg));
                }
                else if (known->type == "world_id")
                {
                    worldId(node, tag, name, arg);
                }
            }
            else if (value.find_first_of("{}") != std::string_view::npos)
            {
                add(code::kBadArg,
                    node,
                    std::format(
                        "{}=\"{}\": a value is a literal or one argument, as {{arm}}",
                        name,
                        value));
            }
        }
        oneOf(node, tag, ports);
    }

    /// A world-model id fed to an input that takes none, such as an object's id as MoveHand's
    /// frame: the leaf would fail only once it runs, after the operator approved it.
    void worldId(
        const tinyxml2::XMLElement& node, std::string_view tag, std::string_view input,
        std::string_view arg)
    {
        const auto typed = learnable_->world_ids.find(tag);
        if (typed == learnable_->world_ids.end() || typed->second.contains(input))
        {
            return;
        }
        std::string takers;
        for (const std::string& port : typed->second)
        {
            takers += (takers.empty() ? "" : " or ") + port;
        }
        add(code::kBadArg,
            node,
            std::format(
                "{{{}}} is a world-model id, and {}'s {} takes none{}",
                arg,
                tag,
                input,
                takers.empty() ? std::string() : "; give it to " + takers));
    }

    /// Each group of inputs of which a leaf takes exactly one, set or bound.
    void oneOf(
        const tinyxml2::XMLElement& node, std::string_view tag,
        const std::map<std::string, std::string, std::less<>>& ports)
    {
        const auto groups = learnable_->one_of.find(tag);
        if (groups == learnable_->one_of.end())
        {
            return;
        }
        for (const auto& group : groups->second)
        {
            std::string choices;
            std::size_t set = 0;
            for (const std::string& port : group)
            {
                const char* value = node.Attribute(port.c_str());
                set += value != nullptr && *value != '\0' ? 1 : 0;
                const auto what = ports.find(port);
                choices += std::format(
                    "{}{} ({})",
                    choices.empty() ? "" : " or ",
                    port,
                    what != ports.end() ? what->second : "");
            }
            if (set == 0)
            {
                add(code::kMissingAttribute, node, std::format("{} needs one of {}", tag, choices));
            }
            else if (set > 1)
            {
                add(code::kForbiddenAttribute,
                    node,
                    std::format("{} takes only one of {}", tag, choices));
            }
        }
    }

    const SkillDefinition*   definition_;
    const Learnable*         learnable_;
    std::vector<Diagnostic>* problems_;
    int                      nodes_ = 0;
    NameSet                  holds_;
};

/// The skill's macro file: its body under a Timeout of its time, and its ports' model. One that
/// moves an arm runs only while both hands are empty: as TuckForTravel knows, an arm swung with
/// something in its hand can drive it into the body.
std::string
macroText(const SkillDefinition& definition, const tinyxml2::XMLElement& body, bool moves_arms)
{
    tinyxml2::XMLDocument doc;
    auto*                 root = doc.NewElement("root");
    root->SetAttribute("BTCPP_format", "4");
    doc.InsertEndChild(root);
    auto* tree = doc.NewElement("BehaviorTree");
    tree->SetAttribute("ID", definition.name.c_str());
    root->InsertEndChild(tree);
    auto* timeout = doc.NewElement("Timeout");
    timeout->SetAttribute(
        "msec",
        std::to_string(std::lround(definition.max_duration_s * 1000.0)).c_str());
    if (moves_arms)
    {
        auto* guarded = doc.NewElement("Sequence");
        auto* empty   = doc.NewElement("ScriptCondition");
        empty->SetAttribute("name", "both_hands_must_be_empty_to_move_an_arm");
        empty->SetAttribute("code", "@held_left == '' && @held_right == ''");
        guarded->InsertEndChild(empty);
        guarded->InsertEndChild(timeout);
        tree->InsertEndChild(guarded);
    }
    else
    {
        tree->InsertEndChild(timeout);
    }
    timeout->InsertEndChild(body.DeepClone(&doc));
    auto* model = doc.NewElement("TreeNodesModel");
    root->InsertEndChild(model);
    auto* entry = doc.NewElement("SubTree");
    entry->SetAttribute("ID", definition.name.c_str());
    model->InsertEndChild(entry);
    for (const SkillArg& arg : definition.args)
    {
        auto* port = doc.NewElement("input_port");
        port->SetAttribute("name", arg.name.c_str());
        port->SetText(arg.description.c_str());
        entry->InsertEndChild(port);
    }
    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    return printer.CStr();
}

nlohmann::json argJson(const SkillArg& arg)
{
    nlohmann::json json{
        { "name", arg.name },     { "type", arg.type },
        { "format", arg.format }, { "description", arg.description },
        { "enum", arg.allowed },
    };
    if (arg.format == "number")
    {
        json["min"] = arg.min;
        json["max"] = arg.max;
    }
    return json;
}

SkillArg argFrom(const nlohmann::json& json)
{
    SkillArg arg;
    arg.name        = json.at("name").get<std::string>();
    arg.type        = json.value("type", std::string("string"));
    arg.format      = json.value("format", std::string());
    arg.description = json.value("description", std::string());
    arg.allowed     = json.value("enum", std::vector<std::string>{});
    if (arg.format == "number")
    {
        arg.min = json.at("min").get<double>();
        arg.max = json.at("max").get<double>();
    }
    return arg;
}

}  // namespace

Learnable Learnable::parse(const std::vector<std::string>& entries, std::string_view palette_xml)
{
    tinyxml2::XMLDocument palette;
    if (palette.Parse(palette_xml.data(), palette_xml.size()) != tinyxml2::XML_SUCCESS)
    {
        throw std::runtime_error(
            std::string("learnable leaves: the palette is not XML: ") + palette.ErrorStr());
    }
    const auto* model = palette.RootElement() == nullptr ?
                            nullptr :
                            palette.RootElement()->FirstChildElement("TreeNodesModel");
    std::map<std::string, std::map<std::string, std::string, std::less<>>, std::less<>> inputs;
    for (const auto* node = model == nullptr ? nullptr : model->FirstChildElement();
         node != nullptr;
         node = node->NextSiblingElement())
    {
        const char* id = node->Attribute("ID");
        if (id == nullptr)
        {
            continue;
        }
        auto& ports = inputs[id];
        for (const auto* port = node->FirstChildElement("input_port"); port != nullptr;
             port             = port->NextSiblingElement("input_port"))
        {
            if (const char* name = port->Attribute("name"))
            {
                const char* text = port->GetText();
                ports[name]      = text != nullptr ? text : "";
            }
        }
    }
    Learnable out;
    for (const std::string& entry : entries)
    {
        const auto        first  = entry.find(':');
        const auto        second = first == std::string::npos ? first : entry.find(':', first + 1);
        const std::string leaf   = entry.substr(0, first);
        const std::string holds =
            first == std::string::npos ? "" : entry.substr(first + 1, second - first - 1);
        if (!holds.empty() && holds != "base" && holds != "arms")
        {
            throw std::runtime_error(
                "learnable leaves: '" + entry + "' must hold base, arms or nothing");
        }
        const auto ports = inputs.find(leaf);
        if (ports == inputs.end())
        {
            throw std::runtime_error("learnable leaves: no leaf " + leaf + " in the palette");
        }
        out.leaves[leaf] = holds;
        if (second == std::string::npos)
        {
            out.ports[leaf] = ports->second;
            continue;
        }
        // Only the ports listed: the rest, such as which server or frame, stay as the leaf has them.
        auto&             allowed = out.ports[leaf];
        auto&             world   = out.world_ids[leaf];
        std::stringstream list(entry.substr(second + 1));
        for (std::string item; std::getline(list, item, ',');)
        {
            std::vector<std::string> group;
            std::stringstream        alternatives(item);
            for (std::string port; std::getline(alternatives, port, '|');)
            {
                const bool takes_id = port.ends_with(kWorldIdMark);
                if (takes_id)
                {
                    port.resize(port.size() - kWorldIdMark.size());
                }
                const auto found = ports->second.find(port);
                if (found == ports->second.end())
                {
                    throw std::runtime_error(std::format(
                        "learnable leaves: {} has no input {} ('{}')",
                        leaf,
                        port,
                        entry));
                }
                allowed.emplace(*found);
                if (takes_id)
                {
                    world.insert(port);
                }
                group.push_back(port);
            }
            if (group.size() > 1)
            {
                out.one_of[leaf].push_back(std::move(group));
            }
        }
    }
    return out;
}

std::string SkillDefinition::json() const
{
    nlohmann::json json{
        { "name", name },
        { "description", description },
        { "args", nlohmann::json::array() },
        { "body_xml", body_xml },
        { "max_duration_s", max_duration_s },
        { "taught_for", taught_for },
        { "taught_s", taught_s },
    };
    for (const SkillArg& arg : args)
    {
        json["args"].push_back(argJson(arg));
    }
    return json.dump(2);
}

SkillDefinition SkillDefinition::fromJson(std::string_view text)
{
    try
    {
        const nlohmann::json json = nlohmann::json::parse(text);
        SkillDefinition      out;
        out.name           = json.at("name").get<std::string>();
        out.description    = json.value("description", std::string());
        out.body_xml       = json.at("body_xml").get<std::string>();
        out.max_duration_s = json.at("max_duration_s").get<double>();
        out.taught_for     = json.value("taught_for", std::string());
        out.taught_s       = json.value("taught_s", 0.0);
        for (const auto& arg : json.value("args", nlohmann::json::array()))
        {
            out.args.push_back(argFrom(arg));
        }
        return out;
    }
    catch (const nlohmann::json::exception& e)
    {
        throw std::runtime_error(std::string("a skill's definition: ") + e.what());
    }
}

std::optional<LearnedSkill> buildLearned(
    const SkillDefinition& definition, const Learnable& learnable, const NameSet& taken,
    std::vector<Diagnostic>& problems)
{
    const auto add = [&problems, &definition](const char* code, std::string message) {
        problems.push_back(Diagnostic{ code, 0, definition.name, std::move(message) });
    };
    if (!isSkillName(definition.name))
    {
        add(code::kBadName,
            "a learned skill's name is 3 to 40 letters and digits, starting with a capital: "
            "PointAt, LookAround");
    }
    else if (
        taken.contains(definition.name) || learnable.leaves.contains(definition.name) ||
        isControlNode(definition.name) || definition.name == "Mission")
    {
        add(code::kDuplicateName, definition.name + " is taken; give the skill a name of its own");
    }
    if (definition.description.empty() || definition.description.size() > kMaxDescription)
    {
        add(code::kBadArg, "say in a sentence or two what the skill does and when to use it");
    }
    if (definition.args.size() > kMaxArgs)
    {
        add(code::kTooLarge, std::format("a learned skill takes at most {} arguments", kMaxArgs));
    }
    NameSet seen;
    for (const SkillArg& arg : definition.args)
    {
        if (!isArgName(arg.name) || !seen.insert(arg.name).second)
        {
            add(code::kBadArg,
                std::format("argument '{}': lower case and _, each named once", arg.name));
        }
        if (const auto why = argProblem(arg))
        {
            add(code::kBadArg, std::format("argument '{}': {}", arg.name, *why));
        }
    }
    if (!std::isfinite(definition.max_duration_s) || definition.max_duration_s < 1.0 ||
        definition.max_duration_s > learnable.max_duration_s)
    {
        add(code::kBadTimeout,
            std::format("max_duration_s is from 1 to {:.0f} s", learnable.max_duration_s));
    }
    if (definition.body_xml.size() > kMaxBodyBytes)
    {
        add(code::kTooLarge, std::format("the body is over {} bytes", kMaxBodyBytes));
        return std::nullopt;
    }

    tinyxml2::XMLDocument doc;
    if (doc.Parse(definition.body_xml.c_str(), definition.body_xml.size()) != tinyxml2::XML_SUCCESS)
    {
        add(code::kXmlSyntax, std::string("the body is not XML: ") + doc.ErrorStr());
        return std::nullopt;
    }
    const tinyxml2::XMLElement* body = doc.RootElement();
    if (body == nullptr || body->NextSiblingElement() != nullptr)
    {
        add(code::kBadStructure, "the body is one element, such as a Sequence of leaves");
        return std::nullopt;
    }
    BodyCheck check(definition, learnable, problems);
    check.walk(*body, 1);
    if (check.nodes() > learnable.max_nodes)
    {
        add(code::kTooManyNodes,
            std::format("the body has {} nodes; {} at most", check.nodes(), learnable.max_nodes));
    }
    if (!problems.empty())
    {
        return std::nullopt;
    }

    LearnedSkill learned;
    learned.definition   = definition;
    Skill& skill         = learned.skill;
    skill.name           = definition.name;
    skill.description    = definition.description;
    skill.args           = definition.args;
    skill.macro          = definition.name;
    skill.max_duration_s = definition.max_duration_s;
    skill.taught_for     = definition.taught_for;
    if (check.holds().contains("base"))
    {
        skill.resources.emplace_back(kBaseResource);
    }
    if (check.holds().contains("arms"))
    {
        skill.resources.emplace_back(kLeftArmResource);
        skill.resources.emplace_back(kRightArmResource);
    }
    skill.risk        = check.holds().contains("arms") ? "manipulation" :
                        check.holds().contains("base") ? "motion" :
                                                         "observe";
    const auto macros = MacroLibrary::parse(
        macroText(definition, *body, check.holds().contains("arms")),
        definition.name);
    learned.macro = macros.front();
    return learned;
}

LearnedSkills::LearnedSkills(std::filesystem::path dir, Learnable learnable, std::size_t max_skills)
  : dir_(std::move(dir))
  , learnable_(std::move(learnable))
  , max_skills_(max_skills)
{}

std::vector<std::string> LearnedSkills::load(const NameSet& taken)
{
    std::vector<std::string> skipped;
    if (dir_.empty() || !std::filesystem::is_directory(dir_))
    {
        return skipped;
    }
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(dir_))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".json")
        {
            files.push_back(entry.path());
        }
    }
    std::ranges::sort(files);
    for (const auto& file : files)
    {
        if (kept_.size() >= max_skills_)
        {
            skipped.push_back(std::format(
                "{}: the robot holds {} learned skills at most",
                file.filename().string(),
                max_skills_));
            continue;
        }
        std::ifstream      in(file);
        std::ostringstream text;
        text << in.rdbuf();
        try
        {
            SkillDefinition definition = SkillDefinition::fromJson(text.str());
            // Forget removes <name>.json, so a skill kept in a file of another name would return.
            if (file.stem().string() != definition.name)
            {
                skipped.push_back(std::format(
                    "{}: holds {}, so it is not named for its skill",
                    file.filename().string(),
                    definition.name));
                continue;
            }
            std::vector<Diagnostic> problems;
            auto                    learned = buildLearned(definition, learnable_, taken, problems);
            if (!learned)
            {
                skipped.push_back(
                    std::format("{}: {}", file.filename().string(), problems.front().message));
                continue;
            }
            learned->skill.learned = "kept";
            kept_[definition.name] = std::move(*learned);
        }
        catch (const std::exception& e)
        {
            skipped.push_back(std::format("{}: {}", file.filename().string(), e.what()));
        }
    }
    return skipped;
}

std::vector<Diagnostic>
LearnedSkills::define(const SkillDefinition& definition, bool keep, const NameSet& taken)
{
    std::vector<Diagnostic> problems;
    auto                    learned = buildLearned(definition, learnable_, taken, problems);
    if (!learned)
    {
        return problems;
    }
    if (!keep && kept_.contains(definition.name))
    {
        problems.push_back(Diagnostic{ code::kDuplicateName,
                                       0,
                                       definition.name,
                                       definition.name + " is a kept skill; forget it first, or "
                                                         "give the new one a name of its own" });
        return problems;
    }
    const bool replaces = kept_.contains(definition.name) || drafts_.contains(definition.name);
    if (!replaces && kept_.size() + drafts_.size() >= max_skills_)
    {
        problems.push_back(
            Diagnostic{ code::kTooLarge,
                        0,
                        definition.name,
                        std::format(
                            "the robot holds {} learned skills already; forget one first",
                            max_skills_) });
        return problems;
    }
    if (keep && !dir_.empty())
    {
        const auto file   = dir_ / (definition.name + ".json");
        const auto temp   = dir_ / (definition.name + ".json.tmp");
        const auto failed = [&](const std::string& why) {
            std::error_code ignored;
            std::filesystem::remove(temp, ignored);
            problems.push_back(Diagnostic{ code::kLoadError,
                                           0,
                                           definition.name,
                                           "the skill could not be kept: " + why });
            return problems;
        };
        std::error_code error;
        std::filesystem::create_directories(dir_, error);
        if (error)
        {
            return failed(dir_.string() + ": " + error.message());
        }
        {
            std::ofstream out(temp);
            out << definition.json();
            if (!out)
            {
                return failed("could not write " + temp.string());
            }
        }
        // Whole or not at all: a skill half written would not load.
        std::filesystem::rename(temp, file, error);
        if (error)
        {
            return failed(file.string() + ": " + error.message());
        }
    }
    learned->skill.learned = keep ? "kept" : "draft";
    drafts_.erase(definition.name);
    kept_.erase(definition.name);
    (keep ? kept_ : drafts_)[definition.name] = std::move(*learned);
    return problems;
}

bool LearnedSkills::forget(std::string_view name)
{
    const std::string key(name);
    const bool        draft = drafts_.erase(key) > 0;
    const bool        kept  = kept_.erase(key) > 0;
    // Its file goes whatever this run held, so it cannot come back at the next start.
    std::error_code removed;
    const bool file = !dir_.empty() && std::filesystem::remove(dir_ / (key + ".json"), removed);
    return draft || kept || file;
}

std::vector<LearnedSkill> LearnedSkills::all() const
{
    std::vector<LearnedSkill> out;
    out.reserve(kept_.size() + drafts_.size());
    for (const auto& [_, skill] : kept_)
    {
        out.push_back(skill);
    }
    for (const auto& [_, skill] : drafts_)
    {
        out.push_back(skill);
    }
    return out;
}

}  // namespace g1_orchestration
