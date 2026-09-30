#ifndef G1_ORCHESTRATION__CATALOG_HPP_
#define G1_ORCHESTRATION__CATALOG_HPP_

/**
 * @file catalog.hpp
 * @brief The skills a mission may use, as `config/catalog.yaml` describes them.
 *
 * Each skill is a macro subtree in `trees/library`. The catalog holds what a planner needs to know
 * about it and the executor needs to check: arguments, authority, and the time it may take.
 */

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace g1_orchestration
{

/// Authority names a skill may need. `base` is a name only; both arms are taken together.
inline constexpr const char* kBaseResource     = "base";
inline constexpr const char* kLeftArmResource  = "left_arm";
inline constexpr const char* kRightArmResource = "right_arm";

/**
 * @brief One argument of a skill.
 */
struct SkillArg
{
    std::string name;
    /// What a planner writes: "string", or "world_id" for an id from the world model (canopy),
    /// which the planner checks against the live world model.
    std::string type;
    /// How the executor checks a value beyond its type: "" (any text), "id", "phrase" or "station".
    std::string format;
    std::string description;
    /// The only values allowed, or empty when any is.
    std::vector<std::string> allowed;
    /// Names where a planner may get the value instead of asking a model, such as
    /// "label(object_id)". The executor still requires the argument in the mission.
    std::string default_from;
};

/**
 * @brief One skill: a macro subtree and what is known about running it.
 */
struct Skill
{
    std::string              name;
    std::string              description;
    std::vector<SkillArg>    args;
    std::vector<std::string> preconditions;
    std::vector<std::string> effects;
    std::vector<std::string> resources;
    bool                     idempotent = false;
    /// "observe", "world_edit", "motion" or "manipulation".
    std::string risk;
    /// The macro's own Timeout, which is also the most this skill counts for in a worst case.
    double max_duration_s = 0.0;
    /// ID of the macro subtree that implements it.
    std::string macro;

    /// Whether running it needs the arms.
    [[nodiscard]] bool needsArms() const;
};

/**
 * @brief The set of skills, and the identity a planner caches them under.
 */
class Catalog
{
public:
    /**
     * @brief Reads and checks a catalog file.
     *
     * @param file Path of catalog.yaml.
     * @throws std::runtime_error When the file is unreadable or a skill is malformed.
     */
    static Catalog load(const std::filesystem::path& file);

    /**
     * @brief Parses and checks catalog text.
     *
     * @param yaml_text The contents of a catalog.yaml.
     * @throws std::runtime_error When a skill is malformed.
     */
    static Catalog parse(std::string_view yaml_text);

    /// First 12 hex digits of the SHA-256 of the catalog text, and of the palette once bound.
    [[nodiscard]] const std::string& version() const { return version_; }

    /**
     * @brief Makes the version cover the palette that is served beside the catalog.
     *
     * The palette lists every leaf's ports as well as the skills', so a change to either bumps the
     * version a planner caches the catalog under.
     *
     * @param palette_xml The `<TreeNodesModel>` document `GetCatalog` serves.
     */
    void bindPalette(std::string_view palette_xml);

    [[nodiscard]] const std::vector<Skill>& skills() const { return skills_; }

    /// The skill implemented by macro @p macro, or null.
    [[nodiscard]] const Skill* findByMacro(std::string_view macro) const;

    /// The catalog as the JSON `GetCatalog` serves: `catalog_version` and `skills`.
    [[nodiscard]] std::string json() const;

private:
    /// SHA-256 of the catalog text, whole.
    std::string        text_sha256_;
    std::string        version_;
    std::vector<Skill> skills_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__CATALOG_HPP_
