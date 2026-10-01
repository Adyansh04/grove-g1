#ifndef G1_ORCHESTRATION__MISSION_VALIDATOR_HPP_
#define G1_ORCHESTRATION__MISSION_VALIDATOR_HPP_

/**
 * @file mission_validator.hpp
 * @brief Decides whether a behavior tree from a planner may run, before anything is loaded.
 *
 * A mission is one `Mission` tree of the control nodes around SubTrees of the catalog's skills.
 * It is checked against an allowlist, and it may define no other tree: the executor supplies the
 * skills' macros itself, so nothing a planner writes reaches a leaf except through a macro a
 * person wrote. No ROS, no BehaviorTree.CPP: the same code answers `ValidateMission`, and is what
 * `ExecuteMission` runs first.
 */

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/macro_library.hpp"

namespace g1_orchestration
{

/// The `code` of a diagnostic. Written for a planner to act on: each says what to change.
namespace code
{
inline constexpr const char* kTooLarge           = "TOO_LARGE";
inline constexpr const char* kHashMismatch       = "HASH_MISMATCH";
inline constexpr const char* kIncludeForbidden   = "INCLUDE_FORBIDDEN";
inline constexpr const char* kXmlSyntax          = "XML_SYNTAX";
inline constexpr const char* kBadRoot            = "BAD_ROOT";
inline constexpr const char* kBadStructure       = "BAD_STRUCTURE";
inline constexpr const char* kNoMissionTree      = "NO_MISSION_TREE";
inline constexpr const char* kDuplicateTree      = "DUPLICATE_TREE";
inline constexpr const char* kExtraTree          = "EXTRA_TREE";
inline constexpr const char* kModelMismatch      = "MODEL_MISMATCH";
inline constexpr const char* kForbiddenNode      = "FORBIDDEN_NODE";
inline constexpr const char* kInternalLeaf       = "INTERNAL_LEAF";
inline constexpr const char* kUnknownNode        = "UNKNOWN_NODE";
inline constexpr const char* kForbiddenAttribute = "FORBIDDEN_ATTRIBUTE";
inline constexpr const char* kUnknownAttribute   = "UNKNOWN_ATTRIBUTE";
inline constexpr const char* kMissingAttribute   = "MISSING_ATTRIBUTE";
inline constexpr const char* kBadName            = "BAD_NAME";
inline constexpr const char* kDuplicateName      = "DUPLICATE_NAME";
inline constexpr const char* kBadTimeout         = "BAD_TIMEOUT";
inline constexpr const char* kBadRetries         = "BAD_RETRIES";
inline constexpr const char* kBadChildren        = "BAD_CHILDREN";
inline constexpr const char* kUnknownMacro       = "UNKNOWN_MACRO";
inline constexpr const char* kUnknownArg         = "UNKNOWN_ARG";
inline constexpr const char* kMissingArg         = "MISSING_ARG";
inline constexpr const char* kBadArg             = "BAD_ARG";
inline constexpr const char* kTooDeep            = "TOO_DEEP";
inline constexpr const char* kTooManyNodes       = "TOO_MANY_NODES";
inline constexpr const char* kTooManySteps       = "TOO_MANY_STEPS";
inline constexpr const char* kTooLong            = "TOO_LONG";
inline constexpr const char* kTooManyProblems    = "TOO_MANY_PROBLEMS";
inline constexpr const char* kLoadError          = "LOAD_ERROR";
inline constexpr const char* kBusy               = "BUSY";
inline constexpr const char* kShuttingDown       = "SHUTTING_DOWN";
inline constexpr const char* kBadMode            = "BAD_MODE";
}  // namespace code

/**
 * @brief One thing wrong with a mission.
 */
struct Diagnostic
{
    std::string code;
    /// 1-based line of the offending element, or 0 when there is none.
    int line = 0;
    /// The node's `name` (its tag when it has none).
    std::string node;
    std::string message;
};

/**
 * @brief Bounds on a mission's size.
 */
struct Limits
{
    /// Nesting of the Mission tree, counting its first node as 1.
    int max_depth = 10;
    /// Elements in the Mission tree, macros not included.
    int max_nodes = 200;
    /// SubTrees in the Mission tree. Each one is a whole macro at load time, with a client for
    /// every action leaf in it, so this bounds what a small document can make the robot build.
    int         max_steps = 32;
    std::size_t max_bytes = 131072;
    /// The most a mission's worst case may come to, in seconds. The watchdog ends a mission by then
    /// at the latest, so one that could run past it is turned away instead of cut off half done.
    double max_duration_s = 1800.0;
    /// Diagnostics reported; the rest are summed up in one.
    int max_diagnostics = 50;
};

/**
 * @brief What checking a mission found out.
 */
struct Validation
{
    std::vector<Diagnostic> diagnostics;
    /// SHA-256 of the text exactly as given, valid or not.
    std::string sha256;
    /// Seconds the mission can take at most: sums for sequences and fallbacks, times the attempts
    /// for retries, capped by each Timeout. 0 when it was not valid enough to say.
    double worst_case_s = 0.0;
    /// IDs of the macros the Mission tree names.
    std::set<std::string> macros;
    /// Name of each SubTree in the Mission tree, and the macro it runs.
    std::map<std::string, std::string> steps;

    [[nodiscard]] bool ok() const { return diagnostics.empty(); }
};

/**
 * @brief A JSON array of `{code, line, node, message}`.
 *
 * A byte that is not valid UTF-8 (a tag or a mission id comes from a client) is replaced by U+FFFD
 * rather than thrown at, so a refusal can always be sent.
 */
std::string diagnosticsJson(const std::vector<Diagnostic>& diagnostics);

/**
 * @brief @p xml with every number argument of a catalog skill written as a decimal ("90.0").
 *
 * BehaviorTree.CPP 4.10 stores a SubTree literal that looks like an integer as an int, which the
 * skill's double port then refuses as a type clash. A value that is not a number is left for
 * validation to report, and text that does not parse comes back unchanged.
 */
std::string withDecimalNumbers(const std::string& xml, const Catalog& catalog);

/**
 * @brief Checks missions against one catalog and one macro library.
 *
 * Immutable once built, so a service thread and the mission thread can share it.
 */
class MissionValidator
{
public:
    /**
     * @throws std::invalid_argument When a skill has no macro in the library, or a macro no skill:
     *         a mission that names such a skill would pass the check and then not load.
     */
    MissionValidator(Catalog catalog, MacroLibrary library, Limits limits);

    /**
     * @brief Checks a mission's text.
     *
     * Rejects: text that includes another file, a hash that is not the text's SHA-256, a document
     * that is not `BTCPP_format="4"` with `main_tree_to_execute="Mission"`, a Mission tree with
     * anything but Sequence, Fallback, RetryUntilSuccessful (1 to 3 attempts), Timeout (positive)
     * and ForceSuccess around catalog skills, with pre or post condition attributes, or larger than
     * the limits (bytes, depth, nodes, steps, and a worst case past the longest a mission may run);
     * any tree besides Mission; and a `<TreeNodesModel>` that is not the library's.
     *
     * @param xml The mission, as it will be run.
     * @param expected_sha256 The hash it was approved under, or nothing to skip that check. An
     *        empty string is a hash, and does not match.
     * @return The verdict, with the worst-case duration when there are no problems.
     */
    [[nodiscard]] Validation validate(
        std::string_view xml, std::optional<std::string_view> expected_sha256 = std::nullopt) const;

    [[nodiscard]] const Catalog&      catalog() const { return catalog_; }
    [[nodiscard]] const MacroLibrary& library() const { return library_; }

private:
    Catalog      catalog_;
    MacroLibrary library_;
    Limits       limits_;
    /// Tags the macros use that a mission may not: leaves, and control nodes beyond the allowed few.
    std::set<std::string, std::less<>> internal_tags_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__MISSION_VALIDATOR_HPP_
