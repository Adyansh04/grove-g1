/**
 * @file repertoire.cpp
 * @brief Building the skill set from the library and the learned skills.
 */

#include "g1_orchestration/repertoire.hpp"

#include <behaviortree_cpp/contrib/json.hpp>
#include <string>
#include <utility>
#include <vector>

namespace g1_orchestration
{

Repertoire::Repertoire(
    Catalog catalog, MacroLibrary library, std::string leaf_palette_xml, NameSet node_ids,
    Limits limits, LearnedSkills learned)
  : catalog_(std::move(catalog))
  , library_(std::move(library))
  , leaf_palette_xml_(std::move(leaf_palette_xml))
  , node_ids_(std::move(node_ids))
  , limits_(limits)
  , learned_(std::move(learned))
{
    const std::lock_guard<std::mutex> lock(mutex_);
    rebuildLocked();
}

std::shared_ptr<const SkillSet> Repertoire::current() const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return current_;
}

std::vector<std::string> Repertoire::load()
{
    const std::lock_guard<std::mutex> lock(mutex_);
    auto                              skipped = learned_.load(takenNames());
    rebuildLocked();
    return skipped;
}

std::vector<Diagnostic> Repertoire::define(const SkillDefinition& definition, bool keep)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    auto                              problems = learned_.define(definition, keep, takenNames());
    if (problems.empty())
    {
        rebuildLocked();
    }
    return problems;
}

bool Repertoire::forget(std::string_view name)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    const bool                        forgot = learned_.forget(name);
    if (forgot)
    {
        rebuildLocked();
    }
    return forgot;
}

NameSet Repertoire::takenNames() const
{
    NameSet taken = node_ids_;
    for (const Skill& skill : catalog_.skills())
    {
        taken.insert(skill.name);
        taken.insert(skill.macro);
    }
    return taken;
}

void Repertoire::rebuildLocked()
{
    std::vector<Skill> skills;
    std::vector<Macro> macros;
    for (const LearnedSkill& learned : learned_.all())
    {
        skills.push_back(learned.skill);
        macros.push_back(learned.macro);
    }
    std::string bodies;
    for (const Macro& macro : macros)
    {
        bodies += macro.xml;
    }
    Catalog            catalog = catalog_.withSkills(skills);
    const MacroLibrary library = library_.withMacros(macros);
    std::string        palette = library.withModelsIn(leaf_palette_xml_);
    catalog.bindPalette(palette, bodies);

    // What a planner needs to teach a skill: the leaves it may build from and what each holds.
    nlohmann::ordered_json json   = nlohmann::ordered_json::parse(catalog.json());
    nlohmann::ordered_json leaves = nlohmann::ordered_json::object();
    for (const auto& [leaf, holds] : learned_.learnable().leaves)
    {
        nlohmann::ordered_json ports = nlohmann::ordered_json::object();
        for (const auto& [port, what] : learned_.learnable().ports.at(leaf))
        {
            ports[port] = what;
        }
        leaves[leaf] = { { "holds", holds }, { "ports", ports } };
    }
    json["learnable"] = { { "leaves", leaves },
                          { "max_duration_s", learned_.learnable().max_duration_s },
                          { "max_nodes", learned_.learnable().max_nodes } };
    current_          = std::make_shared<const SkillSet>(
        SkillSet{ MissionValidator(std::move(catalog), library, limits_),
                  json.dump(),
                  std::move(palette) });
}

}  // namespace g1_orchestration
