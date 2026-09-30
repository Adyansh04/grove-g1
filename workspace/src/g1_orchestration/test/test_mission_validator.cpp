/**
 * @file test_mission_validator.cpp
 * @brief What a mission may and may not say, and how long the ones that pass can take.
 *
 * Runs against the real catalog and macro library, so a skill that is renamed or loses an
 * argument breaks these as well as the drift test.
 */

#include <gmock/gmock.h>

#include <algorithm>
#include <behaviortree_cpp/contrib/json.hpp>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/macro_library.hpp"
#include "g1_orchestration/mission_validator.hpp"
#include "g1_orchestration/sha256.hpp"

namespace
{

using g1_orchestration::Catalog;
using g1_orchestration::Diagnostic;
using g1_orchestration::Limits;
using g1_orchestration::MacroLibrary;
using g1_orchestration::MissionValidator;
using g1_orchestration::Validation;

constexpr const char* kGoToPose =
    R"(<SubTree ID="GoToPose" name="s1_GoToPose" station="4.30;-5.60;1.5708"/>)";

/// A whole document around @p tree_body, the way the compiler writes one.
std::string mission(const std::string& tree_body)
{
    return R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">)" +
           tree_body + R"(</BehaviorTree>
</root>)";
}

MissionValidator makeValidator(Limits limits = {})
{
    return {
        Catalog::load(std::string(G1_CONFIG_DIR) + "/catalog.yaml"),
        MacroLibrary::load(std::string(G1_TREES_DIR) + "/library"),
        limits,
    };
}

/// Whether @p text is JSON, without throwing.
bool isJson(const std::string& text)
{
    return !nlohmann::json::parse(text, nullptr, /*allow_exceptions=*/false).is_discarded();
}

/// The codes a verdict carries, in the order found.
std::vector<std::string> codesOf(const Validation& validation)
{
    std::vector<std::string> codes;
    codes.reserve(validation.diagnostics.size());
    for (const Diagnostic& d : validation.diagnostics)
    {
        codes.push_back(d.code);
    }
    return codes;
}

Validation check(const std::string& xml, const Limits& limits = {})
{
    return makeValidator(limits).validate(xml);
}

/// The first diagnostic with @p code, so a test can read its message and line.
const Diagnostic* find(const Validation& validation, const std::string& code)
{
    const auto found = std::ranges::find(validation.diagnostics, code, &Diagnostic::code);
    return found == validation.diagnostics.end() ? nullptr : &*found;
}

}  // namespace

// --- what a compiled mission looks like ------------------------------------------------------

TEST(MissionValidator, AcceptsAMissionTheWayTheCompilerWritesIt)
{
    // Spec 8.2: each step is [ForceSuccess] [Retry] Timeout SubTree, named s<N>_<Skill>, and the
    // whole is wrapped in a Timeout of 1.2 times the worst case.
    const std::string xml = mission(R"(
    <Timeout msec="1800000">
      <Sequence name="mission">
        <ForceSuccess>
          <RetryUntilSuccessful num_attempts="2">
            <Timeout msec="300000">
              <SubTree ID="GoToPose" name="s1_GoToPose" station="4.30;-5.60;1.5708"/>
            </Timeout>
          </RetryUntilSuccessful>
        </ForceSuccess>
        <Timeout msec="420000">
          <SubTree ID="PickObject" name="s2_PickObject" object_id="red_block"
                   phrase="bright red plastic ball" arm="right"/>
        </Timeout>
        <Timeout msec="300000">
          <SubTree ID="GoToTarget" name="s3_GoToTarget" target="R2"/>
        </Timeout>
        <Timeout msec="360000">
          <SubTree ID="PlaceInto" name="s4_PlaceInto" container_id="brown_box"
                   phrase="brown box container" arm="right"/>
        </Timeout>
      </Sequence>
    </Timeout>)");

    const Validation validation = check(xml);

    EXPECT_THAT(validation.diagnostics, ::testing::IsEmpty())
        << g1_orchestration::diagnosticsJson(validation.diagnostics);
    EXPECT_THAT(
        validation.macros,
        ::testing::UnorderedElementsAre("GoToPose", "PickObject", "GoToTarget", "PlaceInto"));
    EXPECT_EQ(validation.steps.at("s2_PickObject"), "PickObject");
    EXPECT_EQ(validation.steps.size(), 4U);
    // 2 x 240 for the retried walk, then 420, 240 and 360.
    EXPECT_DOUBLE_EQ(validation.worst_case_s, 1500.0);
}

TEST(MissionValidator, AcceptsTheWorkedExampleShippedWithThePackage)
{
    // The mission the simulator ran: what the compiler is checked against, so it has to stay valid.
    std::ifstream     in(std::string(G1_TREES_DIR) + "/missions/pick_and_place.xml");
    const std::string xml{ std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
    ASSERT_FALSE(xml.empty());

    const Validation validation = check(xml);

    EXPECT_THAT(validation.diagnostics, ::testing::IsEmpty())
        << g1_orchestration::diagnosticsJson(validation.diagnostics);
    EXPECT_EQ(validation.steps.size(), 6U);
    EXPECT_DOUBLE_EQ(validation.worst_case_s, 120.0 + 240.0 + 420.0 + 240.0 + 360.0 + 120.0);
}

TEST(MissionValidator, AcceptsAMissionWithTheLibraryModelsAlongside)
{
    // The compiler may copy each macro's <TreeNodesModel> entry; only the library's own passes.
    const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence>)" +
                            std::string(kGoToPose) + R"(</Sequence></BehaviorTree>
  <TreeNodesModel>
    <SubTree ID="GoToPose">
      <input_port name="station">Where to stand and face, as x;y;yaw in the map frame (metres, radians).</input_port>
    </SubTree>
  </TreeNodesModel>
</root>)";

    EXPECT_THAT(check(xml).diagnostics, ::testing::IsEmpty());
}

TEST(MissionValidator, ReportsTheHashOfWhatItWasGivenWhetherOrNotItIsValid)
{
    const std::string xml = mission("<Sequence/>");
    EXPECT_EQ(check(xml).sha256, g1_orchestration::sha256Hex(xml));
}

// --- integrity ------------------------------------------------------------------------------

TEST(MissionValidator, ChecksTheHashAgainstTheExactBytes)
{
    const std::string xml   = mission("<Sequence>" + std::string(kGoToPose) + "</Sequence>");
    const auto        valid = makeValidator();

    EXPECT_THAT(
        valid.validate(xml, g1_orchestration::sha256Hex(xml)).diagnostics,
        ::testing::IsEmpty());
    EXPECT_THAT(
        codesOf(valid.validate(xml, g1_orchestration::sha256Hex(xml + " "))),
        ::testing::Contains("HASH_MISMATCH"));

    std::string upper = g1_orchestration::sha256Hex(xml);
    std::ranges::transform(upper, upper.begin(), [](char c) {
        return static_cast<char>(std::toupper(c));
    });
    EXPECT_THAT(codesOf(valid.validate(xml, upper)), ::testing::Contains("HASH_MISMATCH"))
        << "the hash is lowercase hex; another spelling is not the approved one";
}

TEST(MissionValidator, SkipsTheHashCheckWhenNoneIsGiven)
{
    EXPECT_THAT(
        check(mission("<Sequence>" + std::string(kGoToPose) + "</Sequence>")).diagnostics,
        ::testing::IsEmpty());
}

TEST(MissionValidator, RejectsAnyIncludeInAnySpelling)
{
    for (const char* include : { "<include path=\"other.xml\"/>",
                                 "<INCLUDE path=\"x\"/>",
                                 "<!-- <include path=\"x\"/> -->" })
    {
        const Validation validation =
            check(mission("<Sequence>" + std::string(kGoToPose) + include + "</Sequence>"));
        EXPECT_THAT(codesOf(validation), ::testing::Contains("INCLUDE_FORBIDDEN")) << include;
    }
}

TEST(MissionValidator, ReportsTheLineOfAnInclude)
{
    const Validation validation =
        check("<root BTCPP_format=\"4\" main_tree_to_execute=\"Mission\">\n\n<include "
              "path=\"x\"/>\n</root>");

    const Diagnostic* include = find(validation, "INCLUDE_FORBIDDEN");
    ASSERT_NE(include, nullptr);
    EXPECT_EQ(include->line, 3);
}

TEST(MissionValidator, RejectsATreeTheRobotDidNotOffer)
{
    // The place a leaf could be smuggled in: BehaviorTree.CPP would let this replace the macro.
    const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence><SubTree ID="PickObject" name="s1_PickObject" object_id="a" phrase="b" arm="left"/></Sequence></BehaviorTree>
  <BehaviorTree ID="PickObject"><Sequence><Pick object_id="{object_id}" arm="left"/><AcquireArm/></Sequence></BehaviorTree>
</root>)";

    const Validation validation = check(xml);

    const Diagnostic* extra = find(validation, "EXTRA_TREE");
    ASSERT_NE(extra, nullptr);
    EXPECT_EQ(extra->node, "PickObject");
    EXPECT_EQ(extra->line, 3);
}

TEST(MissionValidator, RejectsATreeThatOnlyPretendsToBeAMacro)
{
    const std::string xml = R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence>)" +
                            std::string(kGoToPose) + R"(</Sequence></BehaviorTree>
  <BehaviorTree ID="MyOwnSkill"><Sequence><Script code="x:=1"/></Sequence></BehaviorTree>
</root>)";

    EXPECT_THAT(codesOf(check(xml)), ::testing::Contains("EXTRA_TREE"));
}

TEST(MissionValidator, OnlyTheLibrarysOwnModelEntriesPass)
{
    const auto entry = [](const std::string& body) {
        return R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence>)" +
               std::string(kGoToPose) +
               R"(</Sequence></BehaviorTree>
  <TreeNodesModel>)" +
               body + R"(</TreeNodesModel>
</root>)";
    };

    // A default on a port would fill in an argument the planner never gave.
    EXPECT_THAT(
        codesOf(check(entry(
            R"(<SubTree ID="GoToPose"><input_port name="station" default="1;2;3"/></SubTree>)"))),
        ::testing::Contains("MODEL_MISMATCH"));
    EXPECT_THAT(
        codesOf(check(entry(R"(<Action ID="Pick"><input_port name="object_id"/></Action>)"))),
        ::testing::Contains("MODEL_MISMATCH"));
    EXPECT_THAT(
        codesOf(check(entry(R"(<SubTree ID="Invented"/>)"))),
        ::testing::Contains("MODEL_MISMATCH"));
}

TEST(MissionValidator, RejectsTextThatIsNotXmlAndSaysWhere)
{
    const Validation validation = check("<root BTCPP_format=\"4\">\n<BehaviorTree "
                                        "ID=\"Mission\">\n<Sequence>\n</BehaviorTree>\n</root>");

    const Diagnostic* syntax = find(validation, "XML_SYNTAX");
    ASSERT_NE(syntax, nullptr);
    EXPECT_GT(syntax->line, 0);
}

TEST(MissionValidator, RejectsATextWithANulByte)
{
    std::string xml = mission("<Sequence>" + std::string(kGoToPose) + "</Sequence>");
    xml.insert(xml.size() / 2, 1, '\0');
    EXPECT_THAT(codesOf(check(xml)), ::testing::Contains("XML_SYNTAX"));
}

TEST(MissionValidator, RejectsATextPastTheSizeLimit)
{
    Limits limits;
    limits.max_bytes = 100;
    EXPECT_THAT(
        codesOf(check(mission("<Sequence>" + std::string(kGoToPose) + "</Sequence>"), limits)),
        ::testing::ElementsAre("TOO_LARGE"));
}

// --- the document ---------------------------------------------------------------------------

TEST(MissionValidator, RequiresTheRootToNameFormat4AndTheMissionTree)
{
    const std::string tree = "<BehaviorTree ID=\"Mission\"><Sequence>" + std::string(kGoToPose) +
                             "</Sequence></BehaviorTree>";

    EXPECT_THAT(
        codesOf(
            check("<root BTCPP_format=\"3\" main_tree_to_execute=\"Mission\">" + tree + "</root>")),
        ::testing::Contains("BAD_ROOT"));
    EXPECT_THAT(
        codesOf(check("<root main_tree_to_execute=\"Mission\">" + tree + "</root>")),
        ::testing::Contains("BAD_ROOT"));
    EXPECT_THAT(
        codesOf(check("<root BTCPP_format=\"4\">" + tree + "</root>")),
        ::testing::Contains("BAD_ROOT"));
    EXPECT_THAT(
        codesOf(
            check("<root BTCPP_format=\"4\" main_tree_to_execute=\"Other\">" + tree + "</root>")),
        ::testing::Contains("BAD_ROOT"));
    EXPECT_THAT(codesOf(check("<tree>" + tree + "</tree>")), ::testing::Contains("BAD_ROOT"));
    EXPECT_THAT(
        codesOf(check(
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"Mission\" extra=\"1\">" + tree +
            "</root>")),
        ::testing::Contains("UNKNOWN_ATTRIBUTE"));
}

TEST(MissionValidator, RequiresAMissionTree)
{
    EXPECT_THAT(
        codesOf(check(
            R"(<root BTCPP_format="4" main_tree_to_execute="Mission"><BehaviorTree ID="Other"><Sequence/></BehaviorTree></root>)")),
        ::testing::Contains("NO_MISSION_TREE"));
}

TEST(MissionValidator, RejectsTwoTreesWithOneId)
{
    const std::string tree = "<BehaviorTree ID=\"Mission\"><Sequence>" + std::string(kGoToPose) +
                             "</Sequence></BehaviorTree>";
    EXPECT_THAT(
        codesOf(check(
            "<root BTCPP_format=\"4\" main_tree_to_execute=\"Mission\">" + tree + tree + "</root>")),
        ::testing::Contains("DUPLICATE_TREE"));
}

TEST(MissionValidator, RejectsAnythingElseInsideTheRoot)
{
    EXPECT_THAT(
        codesOf(check(R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence/></BehaviorTree>
  <Script code="x:=1"/>
</root>)")),
        ::testing::Contains("BAD_STRUCTURE"));
}

TEST(MissionValidator, ExpectsExactlyOneNodeInTheMissionTree)
{
    EXPECT_THAT(
        codesOf(check(mission("<Sequence>" + std::string(kGoToPose) + "</Sequence><Sequence/>"))),
        ::testing::Contains("BAD_CHILDREN"));
}

// --- what may appear in the Mission tree ----------------------------------------------------

class ForbiddenNode : public ::testing::TestWithParam<const char*>
{
};

TEST_P(ForbiddenNode, IsRejectedWherever)
{
    const std::string tag = GetParam();
    const std::string xml = mission(
        "<Sequence>" + std::string(kGoToPose) + "<" + tag + ">" + kGoToPose + "</" + tag +
        "></Sequence>");

    const Validation  validation = check(xml);
    const Diagnostic* rejected   = find(validation, "FORBIDDEN_NODE");

    ASSERT_NE(rejected, nullptr) << g1_orchestration::diagnosticsJson(validation.diagnostics);
    EXPECT_EQ(rejected->node, tag);
    EXPECT_GT(rejected->line, 0);
    EXPECT_FALSE(rejected->message.empty());
}

INSTANTIATE_TEST_SUITE_P(
    EverythingTheSpecNames, ForbiddenNode,
    ::testing::Values(
        "Parallel", "ReactiveSequence", "ReactiveFallback", "Repeat", "KeepRunningUntilFailure",
        "Script", "SetBlackboard", "AcquireArm", "ReleaseArm"));

TEST(MissionValidator, RejectsALeafWrittenDirectlyAndPointsToTheSkills)
{
    const Validation validation =
        check(mission(R"(<Sequence><Pick object_id="red_block" arm="right"/></Sequence>)"));

    const Diagnostic* leaf = find(validation, "INTERNAL_LEAF");
    ASSERT_NE(leaf, nullptr);
    EXPECT_THAT(leaf->message, ::testing::HasSubstr("skill"));
}

TEST(MissionValidator, RejectsAnUnknownNodeAndListsTheSkills)
{
    const Validation validation = check(mission("<Sequence><Teleport/></Sequence>"));

    const Diagnostic* unknown = find(validation, "UNKNOWN_NODE");
    ASSERT_NE(unknown, nullptr);
    EXPECT_THAT(unknown->message, ::testing::HasSubstr("PickObject"));
}

TEST(MissionValidator, RejectsTheMisspeltRetryBehaviorTreeCppStillRegisters)
{
    EXPECT_THAT(
        codesOf(check(mission(
            "<RetryUntilSuccesful num_attempts=\"2\">" + std::string(kGoToPose) +
            "</RetryUntilSuccesful>"))),
        ::testing::Contains("UNKNOWN_NODE"));
}

TEST(MissionValidator, RejectsEveryPreAndPostConditionAttribute)
{
    for (const char* attribute : { "_skipIf",
                                   "_successIf",
                                   "_failureIf",
                                   "_while",
                                   "_onSuccess",
                                   "_onFailure",
                                   "_onHalted",
                                   "_post" })
    {
        const std::string on_control = mission(
            std::string("<Sequence ") + attribute + "=\"true\">" + kGoToPose + "</Sequence>");
        const std::string on_skill = mission(
            std::string(R"(<Sequence><SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0" )") +
            attribute + "=\"true\"/></Sequence>");

        EXPECT_THAT(codesOf(check(on_control)), ::testing::Contains("FORBIDDEN_ATTRIBUTE"))
            << attribute;
        EXPECT_THAT(codesOf(check(on_skill)), ::testing::Contains("FORBIDDEN_ATTRIBUTE"))
            << attribute;
    }
}

TEST(MissionValidator, RejectsAnAttributeAControlNodeDoesNotTake)
{
    EXPECT_THAT(
        codesOf(
            check(mission("<Sequence colour=\"red\">" + std::string(kGoToPose) + "</Sequence>"))),
        ::testing::Contains("UNKNOWN_ATTRIBUTE"));
    EXPECT_THAT(
        codesOf(check(mission(
            "<Timeout msec=\"1000\" num_attempts=\"2\">" + std::string(kGoToPose) + "</Timeout>"))),
        ::testing::Contains("UNKNOWN_ATTRIBUTE"));
}

TEST(MissionValidator, RejectsANameThatCouldConfuseAPath)
{
    for (const char* name : { "s1/PickObject", "a b", "", "x*" })
    {
        const std::string xml = mission(
            std::string(R"(<Sequence><SubTree ID="GoToPose" station="1;2;0" name=")") + name +
            "\"/></Sequence>");
        EXPECT_THAT(codesOf(check(xml)), ::testing::Contains("BAD_NAME")) << name;
    }
}

TEST(MissionValidator, RejectsTwoStepsWithOneName)
{
    const std::string step = R"(<SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/>)";
    EXPECT_THAT(
        codesOf(check(mission("<Sequence>" + step + step + "</Sequence>"))),
        ::testing::Contains("DUPLICATE_NAME"));
}

// --- bounds ---------------------------------------------------------------------------------

TEST(MissionValidator, RequiresAPositiveTimeout)
{
    for (const char* msec : { "0", "-5", "abc", "1.5", "", "99999999999" })
    {
        const std::string xml =
            mission(std::string("<Timeout msec=\"") + msec + "\">" + kGoToPose + "</Timeout>");
        EXPECT_THAT(codesOf(check(xml)), ::testing::Contains("BAD_TIMEOUT")) << "msec=" << msec;
    }
    EXPECT_THAT(
        codesOf(check(mission("<Timeout>" + std::string(kGoToPose) + "</Timeout>"))),
        ::testing::Contains("BAD_TIMEOUT"));
}

TEST(MissionValidator, AllowsOneToThreeAttemptsAndNothingElse)
{
    for (const char* attempts : { "1", "2", "3" })
    {
        const std::string xml = mission(
            std::string("<RetryUntilSuccessful num_attempts=\"") + attempts + "\">" + kGoToPose +
            "</RetryUntilSuccessful>");
        EXPECT_THAT(check(xml).diagnostics, ::testing::IsEmpty()) << attempts;
    }
    for (const char* attempts : { "0", "4", "-1", "abc", "" })
    {
        const std::string xml = mission(
            std::string("<RetryUntilSuccessful num_attempts=\"") + attempts + "\">" + kGoToPose +
            "</RetryUntilSuccessful>");
        EXPECT_THAT(codesOf(check(xml)), ::testing::Contains("BAD_RETRIES")) << attempts;
    }
    EXPECT_THAT(
        codesOf(check(mission(
            "<RetryUntilSuccessful>" + std::string(kGoToPose) + "</RetryUntilSuccessful>"))),
        ::testing::Contains("BAD_RETRIES"))
        << "-1 loops forever, and so would a missing count";
}

TEST(MissionValidator, HoldsTheDepthAndTheNodeCountToTheirLimits)
{
    std::string nested = std::string(kGoToPose);
    for (int i = 0; i < 6; ++i)
    {
        nested.insert(0, "<ForceSuccess>");
        nested += "</ForceSuccess>";
    }
    Limits shallow;
    shallow.max_depth = 4;
    EXPECT_THAT(codesOf(check(mission(nested), shallow)), ::testing::Contains("TOO_DEEP"));
    EXPECT_THAT(check(mission(nested)).diagnostics, ::testing::IsEmpty());

    std::string many = "<Sequence>";
    for (int i = 0; i < 30; ++i)
    {
        many += R"(<SubTree ID="GoToPose" name="s)" + std::to_string(i) +
                R"(_GoToPose" station="1;2;0"/>)";
    }
    many += "</Sequence>";
    Limits small;
    small.max_nodes             = 10;
    const Validation validation = check(mission(many), small);
    EXPECT_THAT(codesOf(validation), ::testing::Contains("TOO_MANY_NODES"));
    EXPECT_EQ(std::ranges::count(codesOf(validation), "TOO_MANY_NODES"), 1)
        << "once, not once per node";
}

TEST(MissionValidator, HoldsTheStepsToTheirLimitBecauseEachOneIsAWholeMacroAtLoadTime)
{
    const auto steps = [](int count) {
        std::string body = "<Sequence>";
        for (int i = 0; i < count; ++i)
        {
            body += R"(<SubTree ID="GoToPose" name="s)" + std::to_string(i) +
                    R"(_GoToPose" station="1;2;0"/>)";
        }
        return mission(body + "</Sequence>");
    };
    Limits limits;
    limits.max_steps      = 3;
    limits.max_duration_s = 1e9;

    EXPECT_THAT(check(steps(3), limits).diagnostics, ::testing::IsEmpty());
    const Validation over = check(steps(9), limits);
    EXPECT_EQ(codesOf(over), std::vector<std::string>{ "TOO_MANY_STEPS" })
        << "once, not once per step";
    EXPECT_THAT(find(over, "TOO_MANY_STEPS")->message, ::testing::HasSubstr("more than 3 steps"));

    EXPECT_EQ(Limits{}.max_steps, 32) << "the default the executor ships with";
    limits.max_steps = Limits{}.max_steps;
    EXPECT_THAT(check(steps(32), limits).diagnostics, ::testing::IsEmpty());
    EXPECT_THAT(codesOf(check(steps(33), limits)), ::testing::Contains("TOO_MANY_STEPS"));
}

TEST(MissionValidator, TurnsAwayAMissionThatCouldRunPastTheLongestOneMayRun)
{
    const std::string two_walks_and_a_tuck = R"(<Sequence>
        <SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/>
        <SubTree ID="GoToPose" name="s2_GoToPose" station="3;4;0"/>
        <SubTree ID="TuckForTravel" name="s3_TuckForTravel"/></Sequence>)";
    Limits            limits;
    limits.max_duration_s = 500.0;

    // 240 + 240 + 120 = 600 s in the worst case: the watchdog would cut it off at 500.
    const Validation over = check(mission(two_walks_and_a_tuck), limits);
    EXPECT_EQ(codesOf(over), std::vector<std::string>{ "TOO_LONG" });
    EXPECT_THAT(find(over, "TOO_LONG")->message, ::testing::HasSubstr("600 s"));
    EXPECT_THAT(find(over, "TOO_LONG")->message, ::testing::HasSubstr("500 s"));
    EXPECT_GT(find(over, "TOO_LONG")->line, 0);

    limits.max_duration_s = 600.0;
    EXPECT_THAT(check(mission(two_walks_and_a_tuck), limits).diagnostics, ::testing::IsEmpty())
        << "exactly the limit is allowed";

    // A Timeout over the whole says it cannot take longer than that, whatever the steps add up to.
    limits.max_duration_s = 500.0;
    EXPECT_THAT(
        check(mission("<Timeout msec=\"450000\">" + two_walks_and_a_tuck + "</Timeout>"), limits)
            .diagnostics,
        ::testing::IsEmpty());
}

TEST(MissionValidator, ShipsWithTheCapTheExecutorsWatchdogHas)
{
    EXPECT_DOUBLE_EQ(Limits{}.max_duration_s, 1800.0);
    // Five picks come to 5 x 420 s.
    std::string picks = "<Sequence>";
    for (int i = 1; i <= 5; ++i)
    {
        picks += R"(<SubTree ID="PickObject" name="s)" + std::to_string(i) +
                 R"(_PickObject" object_id="red_block" phrase="red block" arm="right"/>)";
    }
    EXPECT_THAT(codesOf(check(mission(picks + "</Sequence>"))), ::testing::Contains("TOO_LONG"));
}

TEST(MissionValidator, RequiresTheRightNumberOfChildren)
{
    EXPECT_THAT(codesOf(check(mission("<Sequence/>"))), ::testing::Contains("BAD_CHILDREN"));
    EXPECT_THAT(codesOf(check(mission("<Fallback/>"))), ::testing::Contains("BAD_CHILDREN"));
    EXPECT_THAT(
        codesOf(check(mission(
            "<Timeout msec=\"1000\">" + std::string(kGoToPose) + kGoToPose + "</Timeout>"))),
        ::testing::Contains("BAD_CHILDREN"));
    EXPECT_THAT(
        codesOf(check(mission(
            R"(<Sequence><SubTree ID="GoToPose" name="s1" station="1;2;0"><Sequence/></SubTree></Sequence>)"))),
        ::testing::Contains("BAD_CHILDREN"));
}

TEST(MissionValidator, CapsHowManyProblemsItReports)
{
    std::string many = "<Sequence>";
    for (int i = 0; i < 100; ++i)
    {
        many += "<Teleport/>";
    }
    many += "</Sequence>";
    Limits limits;
    limits.max_diagnostics = 5;

    const Validation validation = check(mission(many), limits);

    EXPECT_EQ(validation.diagnostics.size(), 6U);
    EXPECT_EQ(validation.diagnostics.back().code, "TOO_MANY_PROBLEMS");
}

// --- the worst case -------------------------------------------------------------------------

TEST(MissionValidator, AddsUpASequenceAndTakesTheCatalogTimeOfASkill)
{
    const Validation validation = check(mission(R"(<Sequence>
        <SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/>
        <SubTree ID="TuckForTravel" name="s2_TuckForTravel"/>
      </Sequence>)"));

    EXPECT_THAT(validation.diagnostics, ::testing::IsEmpty());
    EXPECT_DOUBLE_EQ(validation.worst_case_s, 240.0 + 120.0);
}

TEST(MissionValidator, AddsUpAFallbackBecauseItMayTryEveryBranch)
{
    const Validation validation = check(mission(R"(<Fallback>
        <SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/>
        <SubTree ID="GoToPose" name="s2_GoToPose" station="3;4;0"/>
      </Fallback>)"));

    EXPECT_DOUBLE_EQ(validation.worst_case_s, 480.0);
}

TEST(MissionValidator, MultipliesByTheAttemptsOfARetry)
{
    const Validation validation = check(mission(R"(<RetryUntilSuccessful num_attempts="3">
        <SubTree ID="TuckForTravel" name="s1_TuckForTravel"/>
      </RetryUntilSuccessful>)"));

    EXPECT_DOUBLE_EQ(validation.worst_case_s, 3 * 120.0);
}

TEST(MissionValidator, CapsAStepAtItsTimeoutWhenTheTimeoutIsShorter)
{
    const Validation shorter = check(mission("<Timeout msec=\"45000\"><SubTree ID=\"GoToPose\" "
                                             "name=\"s1_GoToPose\" station=\"1;2;0\"/></Timeout>"));
    const Validation longer  = check(mission("<Timeout msec=\"900000\"><SubTree ID=\"GoToPose\" "
                                             "name=\"s1_GoToPose\" station=\"1;2;0\"/></Timeout>"));

    EXPECT_DOUBLE_EQ(shorter.worst_case_s, 45.0);
    EXPECT_DOUBLE_EQ(longer.worst_case_s, 240.0) << "the skill's own limit is the smaller";
}

TEST(MissionValidator, MultipliesBeforeCappingSoARetriedStepStaysUnderItsTimeout)
{
    // Timeout outside Retry: the Timeout bounds all the attempts together.
    const Validation outer =
        check(mission(R"(<Timeout msec="400000"><RetryUntilSuccessful num_attempts="3">
        <SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/></RetryUntilSuccessful></Timeout>)"));
    // Timeout inside Retry: each attempt is bounded, and there are three.
    const Validation inner =
        check(mission(R"(<RetryUntilSuccessful num_attempts="3"><Timeout msec="100000">
        <SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/></Timeout></RetryUntilSuccessful>)"));

    EXPECT_DOUBLE_EQ(outer.worst_case_s, 400.0);
    EXPECT_DOUBLE_EQ(inner.worst_case_s, 300.0);
}

TEST(MissionValidator, ForceSuccessCostsWhatItsChildDoes)
{
    const Validation validation = check(mission(
        R"(<ForceSuccess><SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/></ForceSuccess>)"));
    EXPECT_DOUBLE_EQ(validation.worst_case_s, 240.0);
}

TEST(MissionValidator, GivesNoWorstCaseForAMissionItRejected)
{
    EXPECT_DOUBLE_EQ(check(mission("<Sequence><Teleport/></Sequence>")).worst_case_s, 0.0);
}

// --- what a step is given -------------------------------------------------------------------

std::string step(const std::string& attributes)
{
    return mission("<Sequence><SubTree name=\"s1_Step\" " + attributes + "/></Sequence>");
}

TEST(MissionValidator, RejectsASkillTheCatalogDoesNotHave)
{
    const Validation validation = check(step(R"(ID="Teleport")"));

    const Diagnostic* unknown = find(validation, "UNKNOWN_MACRO");
    ASSERT_NE(unknown, nullptr);
    EXPECT_THAT(unknown->message, ::testing::HasSubstr("GoToPose"));
    EXPECT_THAT(codesOf(check(step(R"(foo="1")"))), ::testing::Contains("MISSING_ATTRIBUTE"));
}

TEST(MissionValidator, RequiresEveryArgumentAndNoOthers)
{
    EXPECT_THAT(codesOf(check(step(R"(ID="GoToPose")"))), ::testing::Contains("MISSING_ARG"));
    EXPECT_THAT(
        codesOf(check(step(R"(ID="GoToPose" station="1;2;0" speed="fast")"))),
        ::testing::Contains("UNKNOWN_ARG"));
    const Validation  missing = check(step(R"(ID="PickObject" object_id="a" phrase="b")"));
    const Diagnostic* arm     = find(missing, "MISSING_ARG");
    ASSERT_NE(arm, nullptr);
    EXPECT_THAT(arm->message, ::testing::HasSubstr("arm"));
}

TEST(MissionValidator, HoldsAnEnumeratedArgumentToItsValues)
{
    EXPECT_THAT(
        check(step(R"(ID="PickObject" object_id="a" phrase="b" arm="left")")).diagnostics,
        ::testing::IsEmpty());
    const Validation  bad = check(step(R"(ID="PickObject" object_id="a" phrase="b" arm="middle")"));
    const Diagnostic* arm = find(bad, "BAD_ARG");
    ASSERT_NE(arm, nullptr);
    EXPECT_THAT(arm->message, ::testing::HasSubstr("left, right"));
}

TEST(MissionValidator, HoldsIdsPhrasesAndStationsToTheirFormat)
{
    const auto pick = [](const std::string& id, const std::string& phrase) {
        return step(
            R"(ID="PickObject" object_id=")" + id + R"(" phrase=")" + phrase + R"(" arm="left")");
    };
    EXPECT_THAT(
        check(pick("red_block", "bright red plastic ball")).diagnostics,
        ::testing::IsEmpty());
    // The world model's ids are accepted as they are: the detector's own id for the object is the
    // lowercased form, which the macro learns from LookFor.
    EXPECT_THAT(check(pick("O17", "a mug")).diagnostics, ::testing::IsEmpty());
    EXPECT_THAT(codesOf(check(pick("red block", "a mug"))), ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(codesOf(check(pick("o17;o18", "a mug"))), ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(codesOf(check(pick("a-b", "a mug"))), ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(codesOf(check(pick(std::string(70, 'a'), "a mug"))), ::testing::Contains("BAD_ARG"));
    // A comma or an equals sign would split what LookFor reads as one entry.
    EXPECT_THAT(codesOf(check(pick("mug", "red, big"))), ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(codesOf(check(pick("mug", "a=b"))), ::testing::Contains("BAD_ARG"));

    for (const char* station : { "1;2", "1;2;3;4", "a;b;c", "1;2;nan", "5000;0;0", "0;0;9", ";;" })
    {
        EXPECT_THAT(
            codesOf(check(step(std::string(R"(ID="GoToPose" station=")") + station + "\""))),
            ::testing::Contains("BAD_ARG"))
            << station;
    }
    EXPECT_THAT(
        check(step(R"(ID="GoToPose" station="-3.5;10;-1.57")")).diagnostics,
        ::testing::IsEmpty());
}

TEST(MissionValidator, HoldsAWorldIdToTheShapeOfCanopysIds)
{
    for (const char* id : { "R2", "O17", "O1234" })
    {
        EXPECT_THAT(
            check(step(std::string(R"(ID="GoToTarget" target=")") + id + "\"")).diagnostics,
            ::testing::IsEmpty())
            << id;
    }
    for (const char* id : { "kitchen", "o17", "O", "17", "O1x", "R 2", "O17;O18" })
    {
        const Validation validation =
            check(step(std::string(R"(ID="GoToTarget" target=")") + id + "\""));
        const Diagnostic* bad = find(validation, "BAD_ARG");
        ASSERT_NE(bad, nullptr) << id;
        EXPECT_THAT(bad->message, ::testing::HasSubstr("world model")) << id;
    }
}

TEST(MissionValidator, RejectsAnArgumentThatPointsAtTheBlackboard)
{
    EXPECT_THAT(
        codesOf(check(step(R"(ID="GoToPose" station="{goal}")"))),
        ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(
        codesOf(check(step(R"(ID="GoToTarget" target="{x}")"))),
        ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(
        codesOf(check(step(R"(ID="GoToTarget" target="" )"))),
        ::testing::Contains("BAD_ARG"));
    EXPECT_THAT(
        codesOf(check(step(R"(ID="GoToTarget" target="R2" _autoremap="true")"))),
        ::testing::Contains("FORBIDDEN_ATTRIBUTE"));
}

// --- the diagnostics as JSON ----------------------------------------------------------------

TEST(MissionValidator, WritesDiagnosticsAsAJsonArrayWithTheAgreedKeys)
{
    const Validation validation = check(mission("<Sequence><Parallel/></Sequence>"));

    const auto json =
        nlohmann::json::parse(g1_orchestration::diagnosticsJson(validation.diagnostics));

    ASSERT_TRUE(json.is_array());
    ASSERT_FALSE(json.empty());
    EXPECT_THAT(json[0].items().begin().key(), ::testing::Eq("code"));
    for (const char* key : { "code", "line", "node", "message" })
    {
        EXPECT_TRUE(json[0].contains(key)) << key;
    }
    EXPECT_EQ(g1_orchestration::diagnosticsJson({}), "[]");
}

TEST(MissionValidator, WritesItsVerdictAsJsonWhateverBytesTheClientSent)
{
    // A tag, a name and an id are the client's, the verdict quotes them, and a byte that is not
    // UTF-8 made nlohmann throw out of a service callback.
    const std::vector<std::string> bodies = {
        "<Sequence><\xff/></Sequence>",
        "<Sequence name=\"a\xc3\"><Teleport/></Sequence>",
        "<Sequence><SubTree ID=\"\xfe\" name=\"s1_\x80\"/></Sequence>",
        "<Sequence><Teleport \xe2=\"1\"/></Sequence>",
    };
    for (const std::string& body : bodies)
    {
        const Validation validation = check(mission(body));
        ASSERT_FALSE(validation.ok()) << body;
        std::string text;
        EXPECT_NO_THROW(text = g1_orchestration::diagnosticsJson(validation.diagnostics)) << body;
        EXPECT_TRUE(isJson(text)) << "not valid JSON: " << text;
    }

    const std::string with_tag = g1_orchestration::diagnosticsJson(
        check(mission("<Sequence><\xff/></Sequence>")).diagnostics);
    EXPECT_THAT(with_tag, ::testing::HasSubstr("\xef\xbf\xbd"))
        << "the bad byte is replaced, not dropped";

    // Not through a verdict at all: a mission id in a BUSY diagnostic.
    const std::string busy =
        g1_orchestration::diagnosticsJson({ { g1_orchestration::code::kBusy, 0, "\xff", "busy" } });
    EXPECT_TRUE(isJson(busy)) << busy;
}

TEST(MissionValidator, RefusesACatalogAndALibraryThatDisagree)
{
    const std::string catalog_text = [] {
        std::ifstream in(std::string(G1_CONFIG_DIR) + "/catalog.yaml");
        return std::string(std::istreambuf_iterator<char>(in), {});
    }();
    const auto library = [] { return MacroLibrary::load(std::string(G1_TREES_DIR) + "/library"); };

    // A skill whose macro is not in the library would pass validate() and then crash the loader.
    std::string renamed = catalog_text;
    const auto  at      = renamed.find("template: TuckForTravel");
    ASSERT_NE(at, std::string::npos);
    renamed.replace(at, std::string("template: TuckForTravel").size(), "template: NoSuchMacro");
    EXPECT_THROW(
        (MissionValidator{ Catalog::parse(renamed), library(), Limits{} }),
        std::invalid_argument);

    // A macro no skill runs is a file nobody can reach.
    std::string trimmed = catalog_text;
    const auto  from    = trimmed.find("  - name: TuckForTravel");
    ASSERT_NE(from, std::string::npos);
    trimmed.erase(from);
    EXPECT_THROW(
        (MissionValidator{ Catalog::parse(trimmed), library(), Limits{} }),
        std::invalid_argument);

    EXPECT_NO_THROW((MissionValidator{ Catalog::parse(catalog_text), library(), Limits{} }));
}

// --- SHA-256 --------------------------------------------------------------------------------

TEST(Sha256, MatchesTheKnownAnswers)
{
    EXPECT_EQ(
        g1_orchestration::sha256Hex(""),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    EXPECT_EQ(
        g1_orchestration::sha256Hex("abc"),
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    EXPECT_EQ(
        g1_orchestration::sha256Hex("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    EXPECT_EQ(
        g1_orchestration::sha256Hex(std::string(1000000, 'a')),
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

// --- hostile input --------------------------------------------------------------------------

TEST(MissionValidator, NeverThrowsOnBrokenInput)
{
    const std::string good = mission(
        R"(<Timeout msec="1000"><Sequence><SubTree ID="GoToPose" name="s1_GoToPose" station="1;2;0"/></Sequence></Timeout>)");
    const auto validator = makeValidator();

    for (std::size_t cut = 0; cut < good.size(); cut += 7)
    {
        EXPECT_NO_THROW({ (void)validator.validate(good.substr(0, cut)); }) << "cut at " << cut;
    }
    for (const char* garbage :
         { "", "<", "<<<>>>", "]]>", "<!DOCTYPE x [<!ENTITY a \"b\">]><root/>", "\xff\xfe" })
    {
        EXPECT_NO_THROW({ (void)validator.validate(garbage); }) << garbage;
        EXPECT_FALSE(validator.validate(garbage).ok()) << garbage;
    }
}

TEST(MissionValidator, SurvivesNestingFarPastTheLimits)
{
    std::string deep;
    for (int i = 0; i < 400; ++i)
    {
        deep += "<Sequence>";
    }
    for (int i = 0; i < 400; ++i)
    {
        deep += "</Sequence>";
    }
    EXPECT_THAT(codesOf(check(mission(deep))), ::testing::Contains("TOO_DEEP"));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
