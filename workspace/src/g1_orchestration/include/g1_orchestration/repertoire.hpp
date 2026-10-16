#ifndef G1_ORCHESTRATION__REPERTOIRE_HPP_
#define G1_ORCHESTRATION__REPERTOIRE_HPP_

/**
 * @file repertoire.hpp
 * @brief The skills the robot has: the library's and the learned ones, served as one set.
 *
 * A mission is checked and built against one SkillSet from start to end, and a define or a
 * forget replaces the set whole, so a mission that runs while the robot learns keeps the skills
 * it started with.
 */

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/learned_skills.hpp"
#include "g1_orchestration/macro_library.hpp"
#include "g1_orchestration/mission_validator.hpp"

namespace g1_orchestration
{

/**
 * @brief What a mission is checked against and what a planner reads, as one unit.
 */
struct SkillSet
{
    MissionValidator validator;
    /// `GetCatalog`'s JSON: the skills, with the learnable leaves for teaching new ones.
    std::string catalog_json;
    /// `GetCatalog`'s `<TreeNodesModel>`: the leaves, then every skill's SubTree.
    std::string palette_xml;
};

/**
 * @brief The library's skills and the learned ones, and the set they make now.
 */
class Repertoire
{
public:
    /**
     * @param catalog The library's skills, restricted as the robot needs.
     * @param library Their macros.
     * @param leaf_palette_xml The leaves' `<TreeNodesModel>`, without any macro.
     * @param node_ids Every node the factory registers, BehaviorTree.CPP's own included: no
     *        learned skill may share an ID with one.
     * @param learned The learned skills, loaded or not.
     * @throws std::invalid_argument When the first set cannot be built.
     */
    Repertoire(
        Catalog catalog, MacroLibrary library, std::string leaf_palette_xml, NameSet node_ids,
        Limits limits, LearnedSkills learned);

    /// The set to check and build a mission against now.
    [[nodiscard]] std::shared_ptr<const SkillSet> current() const;

    /**
     * @brief Reads the kept skills from disk.
     *
     * @return A line for each one left out, saying why.
     */
    std::vector<std::string> load();

    /**
     * @brief Checks and adds a learned skill, as a draft or kept on disk.
     *
     * @return Its problems; none when the set now serves it.
     */
    std::vector<Diagnostic> define(const SkillDefinition& definition, bool keep);

    /// Forgets a learned skill; whether there was one.
    bool forget(std::string_view name);

private:
    /// The library's skill and macro names, which no learned skill may take.
    [[nodiscard]] NameSet takenNames() const;
    /// Builds the set from the library and the learned skills, and serves it.
    void rebuildLocked();

    Catalog       catalog_;
    MacroLibrary  library_;
    std::string   leaf_palette_xml_;
    NameSet       node_ids_;
    Limits        limits_;
    LearnedSkills learned_;

    mutable std::mutex              mutex_;
    std::shared_ptr<const SkillSet> current_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__REPERTOIRE_HPP_
