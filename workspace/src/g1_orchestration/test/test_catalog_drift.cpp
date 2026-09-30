/**
 * @file test_catalog_drift.cpp
 * @brief The catalog, the macro library and the registered leaves still describe one robot.
 *
 * A planner is told what the catalog says and the robot runs what the macros do, so a skill added
 * to one and not the other, an argument that lost its port, or a macro that grew an arm leaf the
 * catalog does not know about is a mission that fails on the robot, or worse, runs unannounced.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <behaviortree_cpp/contrib/json.hpp>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/leaf_report.hpp"
#include "g1_orchestration/macro_library.hpp"
#include "g1_orchestration/sha256.hpp"
#include "g1_orchestration/skill_nodes.hpp"

namespace
{

using g1_orchestration::Catalog;
using g1_orchestration::Macro;
using g1_orchestration::MacroLibrary;
using g1_orchestration::Skill;

const std::string kCatalogFile = std::string(G1_CONFIG_DIR) + "/catalog.yaml";
const std::string kLibraryDir  = std::string(G1_TREES_DIR) + "/library";

std::string readFile(const std::string& path)
{
    std::ifstream in(path);
    return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
}

/// A value each kind of argument accepts, to build a mission from a skill's own description.
std::string exampleValue(const g1_orchestration::SkillArg& arg)
{
    if (!arg.allowed.empty())
    {
        return arg.allowed.front();
    }
    if (arg.type == "world_id")
    {
        return "R2";
    }
    if (arg.format == "station")
    {
        return "1.0;2.0;0.5";
    }
    if (arg.format == "id")
    {
        return "some_object";
    }
    return "some words";
}

/// The Timeout a macro is wrapped in, in seconds, or a negative number when there is none.
double outerTimeoutS(const Macro& macro)
{
    tinyxml2::XMLDocument doc;
    doc.Parse(macro.xml.c_str(), macro.xml.size());
    const tinyxml2::XMLElement* tree = doc.RootElement()->FirstChildElement("BehaviorTree");
    while (tree != nullptr && std::string(tree->Attribute("ID")) != macro.id)
    {
        tree = tree->NextSiblingElement("BehaviorTree");
    }
    const tinyxml2::XMLElement* first = tree != nullptr ? tree->FirstChildElement() : nullptr;
    if (first == nullptr || std::string(first->Name()) != "Timeout" ||
        first->NextSiblingElement() != nullptr)
    {
        return -1.0;
    }
    return first->DoubleAttribute("msec", -1.0) / 1000.0;
}

}  // namespace

TEST(CatalogDrift, EverySkillHasAMacroAndEveryMacroASkill)
{
    const Catalog      catalog = Catalog::load(kCatalogFile);
    const MacroLibrary library = MacroLibrary::load(kLibraryDir);

    std::set<std::string> skills;
    for (const Skill& skill : catalog.skills())
    {
        skills.insert(skill.macro);
        EXPECT_NE(library.find(skill.macro), nullptr)
            << skill.name << " has no macro " << skill.macro;
    }
    std::set<std::string> macros;
    for (const Macro& macro : library.macros())
    {
        macros.insert(macro.id);
    }
    EXPECT_EQ(skills, macros) << "a macro nobody can ask for, or a skill nobody wrote";
}

TEST(CatalogDrift, TheArgumentsAreThePortsTheMacroDeclares)
{
    const Catalog      catalog = Catalog::load(kCatalogFile);
    const MacroLibrary library = MacroLibrary::load(kLibraryDir);

    for (const Skill& skill : catalog.skills())
    {
        const Macro* macro = library.find(skill.macro);
        ASSERT_NE(macro, nullptr) << skill.macro;
        std::vector<std::string> args;
        args.reserve(skill.args.size());
        for (const auto& arg : skill.args)
        {
            args.push_back(arg.name);
        }
        std::vector<std::string> ports = macro->ports;
        std::ranges::sort(args);
        std::ranges::sort(ports);
        EXPECT_EQ(args, ports) << skill.name;
    }
}

TEST(CatalogDrift, EachMacroIsOneTimeoutAndItsLimitIsTheCatalogs)
{
    const Catalog      catalog = Catalog::load(kCatalogFile);
    const MacroLibrary library = MacroLibrary::load(kLibraryDir);

    for (const Skill& skill : catalog.skills())
    {
        const Macro* macro = library.find(skill.macro);
        ASSERT_NE(macro, nullptr) << skill.macro;
        EXPECT_DOUBLE_EQ(outerTimeoutS(*macro), skill.max_duration_s)
            << skill.name
            << ": the macro's own Timeout is what bounds it, so it is the number the worst case "
               "uses";
    }
}

TEST(CatalogDrift, ResourcesCoverWhatTheMacroLeavesNeed)
{
    // What moving each leaf takes control of. A leaf missing here is one the catalog cannot vouch for.
    const std::map<std::string, std::string> needs = {
        { "NavigateToPose", g1_orchestration::kBaseResource },
        { "ApproachObject", g1_orchestration::kBaseResource },
        { "Retreat", g1_orchestration::kBaseResource },
        { "StepClear", g1_orchestration::kBaseResource },
        { "TurnTo", g1_orchestration::kBaseResource },
        { "Pick", g1_orchestration::kRightArmResource },
        { "Place", g1_orchestration::kRightArmResource },
        { "SetArmPosture", g1_orchestration::kRightArmResource },
        { "Grasp", g1_orchestration::kRightArmResource },
    };
    const Catalog      catalog = Catalog::load(kCatalogFile);
    const MacroLibrary library = MacroLibrary::load(kLibraryDir);

    for (const Skill& skill : catalog.skills())
    {
        const Macro* macro = library.find(skill.macro);
        ASSERT_NE(macro, nullptr) << skill.macro;
        for (const std::string& tag : macro->tags)
        {
            const auto need = needs.find(tag);
            if (need == needs.end())
            {
                continue;
            }
            // Both arms are taken together, so either arm name stands for the pair.
            const bool arm = need->second != g1_orchestration::kBaseResource;
            EXPECT_TRUE(
                arm ? skill.needsArms() :
                      std::ranges::find(skill.resources, need->second) != skill.resources.end())
                << skill.name << " uses " << tag << " but its resources are not enough";
        }
    }
}

TEST(CatalogDrift, NoMacroTakesOrGivesBackAuthorityItself)
{
    const MacroLibrary library = MacroLibrary::load(kLibraryDir);

    for (const Macro& macro : library.macros())
    {
        for (const char* forbidden : { "AcquireArm", "ReleaseArm" })
        {
            EXPECT_EQ(std::ranges::count(macro.tags, forbidden), 0)
                << macro.id << " has " << forbidden
                << ": the executor owns the arms for the whole mission";
        }
    }
}

TEST(CatalogDrift, EveryMacroLoadsWithEveryLeafItUsesRegistered)
{
    const Catalog      catalog = Catalog::load(kCatalogFile);
    const MacroLibrary library = MacroLibrary::load(kLibraryDir);
    auto               node    = std::make_shared<rclcpp::Node>("test_catalog_drift");

    for (const Skill& skill : catalog.skills())
    {
        const Macro* macro = library.find(skill.macro);
        ASSERT_NE(macro, nullptr) << skill.macro;
        std::string step = "<SubTree ID=\"" + skill.macro + "\" name=\"s1_" + skill.macro + "\"";
        for (const auto& arg : skill.args)
        {
            step += " " + arg.name + "=\"" + exampleValue(arg) + "\"";
        }
        step += "/>";
        const std::string mission = R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence>)" +
                                    step + R"(</Sequence></BehaviorTree></root>)";

        BT::BehaviorTreeFactory factory;
        g1_orchestration::registerSkillNodes(factory, g1_orchestration::RosContext{ node });
        factory.registerBehaviorTreeFromText(macro->xml);

        EXPECT_NO_THROW({ BT::Tree tree = factory.createTreeFromText(mission); }) << skill.macro;
    }
}

TEST(CatalogDrift, EveryMacroSubtreeIsInThePaletteWithItsPorts)
{
    const MacroLibrary      library = MacroLibrary::load(kLibraryDir);
    auto                    node    = std::make_shared<rclcpp::Node>("test_catalog_model");
    BT::BehaviorTreeFactory factory;
    g1_orchestration::registerSkillNodes(factory, g1_orchestration::RosContext{ node });

    const std::string model = library.withModelsIn(g1_orchestration::nodeModelXml(factory));

    for (const Macro& macro : library.macros())
    {
        EXPECT_THAT(model, ::testing::HasSubstr("<SubTree ID=\"" + macro.id + "\"")) << macro.id;
        for (const std::string& port : macro.ports)
        {
            EXPECT_THAT(model, ::testing::HasSubstr("name=\"" + port + "\""))
                << macro.id << "." << port;
        }
    }
    EXPECT_THAT(model, ::testing::HasSubstr("<Action ID=\"Pick\""))
        << "the leaf palette is still there";
}

TEST(CatalogDrift, TheJsonHasTheShapeThePlannerReads)
{
    const Catalog catalog = Catalog::load(kCatalogFile);

    const auto json = nlohmann::json::parse(catalog.json());

    EXPECT_EQ(
        json.at("catalog_version").get<std::string>(),
        g1_orchestration::sha256Hex(readFile(kCatalogFile)).substr(0, 12));
    EXPECT_EQ(catalog.version(), json.at("catalog_version").get<std::string>());
    ASSERT_TRUE(json.at("skills").is_array());
    ASSERT_EQ(json.at("skills").size(), catalog.skills().size());
    for (const auto& skill : json.at("skills"))
    {
        for (const char* key : { "name",
                                 "description",
                                 "args",
                                 "requires",
                                 "effects",
                                 "resources",
                                 "idempotent",
                                 "risk",
                                 "max_duration_s",
                                 "template" })
        {
            EXPECT_TRUE(skill.contains(key)) << skill.at("name") << " lacks " << key;
        }
        EXPECT_TRUE(skill.at("args").is_array());
        EXPECT_TRUE(skill.at("idempotent").is_boolean());
        EXPECT_TRUE(skill.at("max_duration_s").is_number());
        for (const auto& arg : skill.at("args"))
        {
            for (const char* key : { "name", "type", "description", "enum" })
            {
                EXPECT_TRUE(arg.contains(key)) << skill.at("name") << " arg lacks " << key;
            }
            EXPECT_TRUE(arg.at("enum").is_array());
        }
    }
}

TEST(CatalogDrift, StopAllCoversEveryActionServerALeafUses)
{
    const YAML::Node params = YAML::LoadFile(
        std::string(G1_CONFIG_DIR) +
        "/nervros_executor.yaml")["nervros_executor"]["ros__parameters"];
    std::set<std::string> covered;
    for (const YAML::Node& spec : params["stop_action_servers"])
    {
        const auto text = spec.as<std::string>();
        covered.insert(text.substr(0, text.find(':')));
    }

    auto                    node = std::make_shared<rclcpp::Node>("test_stop_coverage");
    BT::BehaviorTreeFactory factory;
    g1_orchestration::registerSkillNodes(factory, g1_orchestration::RosContext{ node });
    std::set<std::string> used;
    for (const auto& [id, manifest] : factory.manifests())
    {
        if (factory.builtinNodes().contains(id))
        {
            continue;
        }
        const auto leaf = factory.instantiateTreeNode(id, id, BT::NodeConfig());
        if (const auto* report = dynamic_cast<const g1_orchestration::LeafReport*>(leaf.get());
            report != nullptr && !report->actionServer().empty())
        {
            used.insert(report->actionServer());
            EXPECT_TRUE(covered.contains(report->actionServer()))
                << id << " sends goals to " << report->actionServer()
                << ", which StopAll would leave running";
        }
    }
    EXPECT_EQ(used, covered) << "StopAll lists a server no leaf uses";
    EXPECT_FALSE(used.empty());
    // The arm controller is cancelled apart from the leaves' servers, and the hands never are.
    EXPECT_EQ(params["arm_controller_action"].as<std::string>().find("hand"), std::string::npos);
}

TEST(CatalogDrift, EveryPredicateIsOneOfTheFewTheAgentUnderstandsAboutItsOwnArguments)
{
    const std::set<std::string> vocabulary = { "at", "near", "holding", "hand_empty", "inside" };
    const Catalog               catalog    = Catalog::load(kCatalogFile);

    for (const Skill& skill : catalog.skills())
    {
        std::set<std::string> names;
        for (const auto& arg : skill.args)
        {
            names.insert(arg.name);
        }
        std::vector<std::string> predicates = skill.preconditions;
        predicates.insert(predicates.end(), skill.effects.begin(), skill.effects.end());
        for (const std::string& predicate : predicates)
        {
            const std::size_t open  = predicate.find('(');
            const std::size_t close = predicate.find(')');
            ASSERT_NE(open, std::string::npos) << skill.name << ": '" << predicate << "'";
            ASSERT_EQ(close, predicate.size() - 1) << skill.name << ": '" << predicate << "'";
            EXPECT_TRUE(vocabulary.contains(predicate.substr(0, open)))
                << skill.name << ": '" << predicate << "'";
            // Each term names one of the skill's arguments, a bound variable (one capital letter),
            // or a hand.
            std::string terms = predicate.substr(open + 1, close - open - 1);
            for (std::size_t start = 0; start < terms.size();)
            {
                const std::size_t comma = terms.find(',', start);
                std::string       term =
                    terms.substr(start, comma == std::string::npos ? comma : comma - start);
                term.erase(0, term.find_first_not_of(' '));
                const bool variable =
                    term.size() == 1 && std::isupper(static_cast<unsigned char>(term.front())) != 0;
                EXPECT_TRUE(names.contains(term) || variable || term == "left" || term == "right")
                    << skill.name << ": '" << predicate << "' names '" << term << "'";
                start = comma == std::string::npos ? terms.size() : comma + 1;
            }
        }
    }
}

TEST(CatalogDrift, ThePhrasesAreFilledInFromTheWorldModelsLabelForTheObject)
{
    const auto json = nlohmann::json::parse(Catalog::load(kCatalogFile).json());

    std::map<std::string, std::string> defaults;
    for (const auto& skill : json.at("skills"))
    {
        for (const auto& arg : skill.at("args"))
        {
            if (arg.contains("default_from"))
            {
                defaults[skill.at("name").get<std::string>() + "." + arg.at("name").get<std::string>()] =
                    arg.at("default_from");
            }
        }
    }

    EXPECT_THAT(
        defaults,
        ::testing::UnorderedElementsAre(
            ::testing::Pair("PickObject.phrase", "label(object_id)"),
            ::testing::Pair("PlaceInto.phrase", "label(container_id)")));
}

TEST(CatalogDrift, TheVersionChangesWithTheText)
{
    const std::string text    = readFile(kCatalogFile);
    const Catalog     changed = Catalog::parse(text + "\n# edited\n");

    EXPECT_NE(changed.version(), Catalog::parse(text).version());
    EXPECT_EQ(Catalog::parse(text).version().size(), 12U);
}

TEST(CatalogDrift, TheVersionAlsoCoversThePaletteServedBesideTheCatalog)
{
    // The palette lists every leaf's ports, so a leaf that changes is a different catalog.
    const std::string text  = readFile(kCatalogFile);
    Catalog           plain = Catalog::parse(text);
    Catalog           one   = Catalog::parse(text);
    Catalog           two   = Catalog::parse(text);
    Catalog           again = Catalog::parse(text);
    one.bindPalette("<root>one</root>");
    two.bindPalette("<root>two</root>");
    again.bindPalette("<root>one</root>");

    EXPECT_NE(one.version(), plain.version());
    EXPECT_NE(one.version(), two.version());
    EXPECT_EQ(one.version(), again.version()) << "the same catalog and palette, the same version";
    EXPECT_EQ(one.version().size(), 12U);
    EXPECT_EQ(nlohmann::json::parse(one.json()).at("catalog_version"), one.version())
        << "the JSON says what the response says";
    one.bindPalette("<root>two</root>");
    EXPECT_EQ(one.version(), two.version()) << "binding again replaces, it does not stack";
}

TEST(CatalogDrift, RefusesACatalogThatIsBroken)
{
    const auto parse = [](const std::string& yaml) { return Catalog::parse(yaml); };

    EXPECT_THROW((void)parse(""), std::runtime_error);
    EXPECT_THROW((void)parse("skills: []"), std::runtime_error);
    EXPECT_THROW((void)parse("skills:\n  - name: A\n"), std::runtime_error)
        << "no description, template and the rest";
    std::string dup = readFile(kCatalogFile);
    dup.replace(dup.find("risk: motion"), std::string("risk: motion").size(), "risk: flying");
    EXPECT_THROW((void)parse(dup), std::runtime_error) << "an unknown risk";
}

int main(int argc, char** argv)
{
    // No other thread exists yet.
    // NOLINTNEXTLINE(concurrency-mt-unsafe)
    setenv("ROS_DOMAIN_ID", "79", 1);
    ::testing::InitGoogleMock(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
