#ifndef G1_ORCHESTRATION__MACRO_LIBRARY_HPP_
#define G1_ORCHESTRATION__MACRO_LIBRARY_HPP_

/**
 * @file macro_library.hpp
 * @brief The macro subtrees a mission may name, read from `trees/library`.
 *
 * A mission only names macros; the executor registers its own copies with the factory that loads
 * the mission, so a model can arrange the macros a person wrote and never write a leaf of its own.
 */

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace g1_orchestration
{

/**
 * @brief One macro, and what is known about it without loading it.
 */
struct Macro
{
    std::string id;
    /// The macro's file as written, ready for `registerBehaviorTreeFromText`.
    std::string xml;
    /// Its `<SubTree>` entry in the `<TreeNodesModel>`, canonically printed.
    std::string model;
    /// Names of the ports the model declares.
    std::vector<std::string> ports;
    /// Every tag inside the body, repeated as often as it occurs.
    std::vector<std::string> tags;
};

/**
 * @brief Every macro in a directory.
 */
class MacroLibrary
{
public:
    /**
     * @brief Reads the `.xml` files of @p dir, one macro each.
     *
     * A file is a `<root>` holding a `<BehaviorTree>` and the `<TreeNodesModel>` entry that
     * declares its ports. Macros cannot nest, so a mission never has to carry one inside another.
     *
     * @param dir The library directory.
     * @throws std::runtime_error When a file is malformed, a macro has no model or the reverse,
     *         an ID repeats, or a macro names another.
     */
    static MacroLibrary load(const std::filesystem::path& dir);

    /// The macro with this ID, or null.
    [[nodiscard]] const Macro* find(std::string_view id) const;

    [[nodiscard]] const std::vector<Macro>& macros() const { return macros_; }

    /**
     * @brief Adds every macro's model to a palette.
     *
     * @param palette_xml BehaviorTree.CPP's `<TreeNodesModel>` document for the leaves.
     * @return The same document with the macros' `<SubTree>` entries added.
     * @throws std::runtime_error If @p palette_xml is not such a document.
     */
    [[nodiscard]] std::string withModelsIn(const std::string& palette_xml) const;

private:
    std::vector<Macro> macros_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__MACRO_LIBRARY_HPP_
