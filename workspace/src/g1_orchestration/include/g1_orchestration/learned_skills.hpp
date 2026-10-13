#ifndef G1_ORCHESTRATION__LEARNED_SKILLS_HPP_
#define G1_ORCHESTRATION__LEARNED_SKILLS_HPP_

/**
 * @file learned_skills.hpp
 * @brief Skills a planner teaches the robot, built only from the leaves a learned skill may use.
 *
 * A person writes the library's macros; a learned skill is a macro a planner wrote, so it is held
 * to more: its body may use only the control nodes a mission may and the learnable leaves, whose
 * servers check every goal themselves (Nav2, MoveIt, the bounded walk and turn, the detector, a
 * wait), with each attribute a literal or one of the skill's own arguments. Its resources and risk
 * follow from the leaves it uses, and its Timeout bounds it. A new skill is a draft, served until
 * the executor restarts; one that ran well is kept on disk and loads with the executor, checked
 * again each time.
 */

#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/macro_library.hpp"
#include "g1_orchestration/mission_validator.hpp"

namespace g1_orchestration
{

/// Names as sets that look up by `std::string_view` as well.
using NameSet = std::set<std::string, std::less<>>;

/**
 * @brief What a learned skill may be built from, and how large it may be.
 */
struct Learnable
{
    /// Each learnable leaf's tag, with the authority it needs: `base`, `arms`, or empty.
    std::map<std::string, std::string, std::less<>> leaves;
    /// The input ports a learned skill may set on each learnable leaf: name to what it is for.
    std::map<std::string, std::map<std::string, std::string, std::less<>>, std::less<>> ports;
    int    max_nodes      = 40;
    int    max_depth      = 8;
    double max_duration_s = 300.0;

    /**
     * @brief The learnable leaves from `Leaf:holds:port,port` entries: the authority it needs
     *        (`base`, `arms` or empty) and the only input ports a learned skill may set, each
     *        described as the palette describes it. `Leaf` and `Leaf:holds` open every input.
     *
     * @param palette_xml The `<TreeNodesModel>` document of the leaves.
     * @throws std::runtime_error For an entry of another shape, a leaf the palette lacks, or a
     *         port it does not have.
     */
    static Learnable parse(const std::vector<std::string>& entries, std::string_view palette_xml);
};

/**
 * @brief A skill as a planner teaches it.
 */
struct SkillDefinition
{
    std::string           name;
    std::string           description;
    std::vector<SkillArg> args;
    /// One element: the body, of control nodes and learnable leaves.
    std::string body_xml;
    double      max_duration_s = 0.0;
    /// The operator's request it was taught for.
    std::string taught_for;
    /// Unix seconds.
    double taught_s = 0.0;

    /// The definition as JSON, the form a kept skill takes on disk.
    [[nodiscard]] std::string json() const;

    /**
     * @brief A definition from its JSON form: `name`, `description`, `args` as `GetCatalog`
     *        writes a skill's, `body_xml`, `max_duration_s`, `taught_for` and `taught_s`.
     *
     * @throws std::runtime_error For JSON of another shape, or an argument the catalog would
     *         refuse.
     */
    static SkillDefinition fromJson(std::string_view text);
};

/**
 * @brief A learned skill that passed its checks: its catalog entry and its macro.
 */
struct LearnedSkill
{
    SkillDefinition definition;
    Skill           skill;
    Macro           macro;
};

/**
 * @brief Checks @p definition and builds the skill.
 *
 * @param taken Names it may not take: every skill and macro already served.
 * @param[out] problems Everything wrong with it, for the planner to fix in one go.
 * @return The skill, or nothing when there are problems.
 */
std::optional<LearnedSkill> buildLearned(
    const SkillDefinition& definition, const Learnable& learnable, const NameSet& taken,
    std::vector<Diagnostic>& problems);

/**
 * @brief The learned skills: drafts in memory, kept ones in a folder, a JSON file each.
 *
 * Not thread-safe; the executor holds it under its own lock.
 */
class LearnedSkills
{
public:
    /**
     * @param dir Where kept skills live; empty keeps none on disk.
     * @param max_skills How many it holds at most, drafts and kept together.
     */
    LearnedSkills(std::filesystem::path dir, Learnable learnable, std::size_t max_skills);

    /**
     * @brief Reads the kept skills, `<name>.json` each, up to the most it may hold. One that no
     *        longer passes its checks, such as after a leaf it used went, or whose file is not
     *        named for it, stays on disk and is left out.
     *
     * @return A line for each skill left out, saying why.
     */
    std::vector<std::string> load(const NameSet& taken);

    /**
     * @brief Checks and adds @p definition, replacing a learned skill of its name: a draft, or
     *        kept on disk when @p keep. A kept skill is not replaced by a draft: forget it first.
     *
     * @param taken The names of the skills that are not learned.
     * @return Its problems; none when it was added.
     */
    std::vector<Diagnostic>
    define(const SkillDefinition& definition, bool keep, const NameSet& taken);

    /**
     * @brief Forgets the learned skill @p name, removing it from disk when it was kept.
     *
     * @return Whether there was one.
     */
    bool forget(std::string_view name);

    /// Every learned skill, kept ones first, by name.
    [[nodiscard]] std::vector<LearnedSkill> all() const;

    [[nodiscard]] const Learnable& learnable() const { return learnable_; }

private:
    std::filesystem::path                            dir_;
    Learnable                                        learnable_;
    std::size_t                                      max_skills_;
    std::map<std::string, LearnedSkill, std::less<>> kept_;
    std::map<std::string, LearnedSkill, std::less<>> drafts_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__LEARNED_SKILLS_HPP_
