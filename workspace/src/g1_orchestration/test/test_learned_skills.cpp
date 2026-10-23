/**
 * @file test_learned_skills.cpp
 * @brief What a learned skill may be built from, how it is kept, and how the served set changes.
 */

#include <gmock/gmock.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/learned_skills.hpp"
#include "g1_orchestration/macro_library.hpp"
#include "g1_orchestration/repertoire.hpp"

namespace
{

using g1_orchestration::buildLearned;
using g1_orchestration::Catalog;
using g1_orchestration::Diagnostic;
using g1_orchestration::Learnable;
using g1_orchestration::LearnedSkills;
using g1_orchestration::Limits;
using g1_orchestration::MacroLibrary;
using g1_orchestration::NameSet;
using g1_orchestration::Repertoire;
using g1_orchestration::SkillArg;
using g1_orchestration::SkillDefinition;
using ::testing::HasSubstr;

/// A few leaves, as registerSkillNodes() writes their models.
constexpr const char* kPalette = R"(<root BTCPP_format="4"><TreeNodesModel>
  <Action ID="MoveHand"><input_port name="arm"/><input_port name="target"/><input_port name="frame"/>
    <input_port name="mode"/><input_port name="offset_z"/></Action>
  <Action ID="Wait"><input_port name="seconds"/></Action>
  <Action ID="TurnBy"><input_port name="degrees"/></Action>
  <Action ID="Pick"><input_port name="object_id"/></Action>
</TreeNodesModel></root>)";

Learnable learnable()
{
    return Learnable::parse({ "MoveHand:arms", "Wait", "TurnBy:base" }, kPalette);
}

SkillArg arg(const std::string& name, const std::string& type = "string")
{
    SkillArg a;
    a.name        = name;
    a.type        = type;
    a.description = "the " + name;
    return a;
}

SkillDefinition pointAt()
{
    SkillDefinition d;
    d.name           = "PointAt";
    d.description    = "Point at an object with one hand, and hold it for two seconds.";
    d.args           = { arg("object", "world_id"), arg("arm") };
    d.body_xml       = R"(<Sequence><MoveHand arm="{arm}" target="{object}" mode="point"/>
                          <Wait seconds="2"/></Sequence>)";
    d.max_duration_s = 30.0;
    d.taught_for     = "point at the red mug";
    return d;
}

/// The codes of what is wrong with @p definition.
std::vector<std::string> problemsOf(const SkillDefinition& definition, const NameSet& taken = {})
{
    std::vector<Diagnostic> problems;
    EXPECT_FALSE(buildLearned(definition, learnable(), taken, problems).has_value());
    std::vector<std::string> codes;
    codes.reserve(problems.size());
    for (const Diagnostic& d : problems)
    {
        codes.push_back(d.code);
    }
    return codes;
}

}  // namespace

TEST(LearnedSkill, AGestureOfLearnableLeavesIsBuiltWithTheAuthorityItsLeavesTake)
{
    std::vector<Diagnostic> problems;
    const auto              learned = buildLearned(pointAt(), learnable(), {}, problems);
    ASSERT_TRUE(learned.has_value()) << (problems.empty() ? "" : problems.front().message);
    EXPECT_EQ(learned->skill.risk, "manipulation");
    EXPECT_THAT(learned->skill.resources, ::testing::ElementsAre("left_arm", "right_arm"));
    EXPECT_EQ(learned->skill.macro, "PointAt");
    EXPECT_EQ(learned->macro.id, "PointAt");
    EXPECT_THAT(learned->macro.xml, HasSubstr(R"(<Timeout msec="30000">)"));
    EXPECT_THAT(learned->macro.xml, HasSubstr("both_hands_must_be_empty_to_move_an_arm"));
    EXPECT_THAT(learned->macro.ports, ::testing::ElementsAre("object", "arm"));

    SkillDefinition look = pointAt();
    look.name            = "LookAround";
    look.args            = {};
    look.body_xml        = R"(<Sequence><TurnBy degrees="90"/><Wait seconds="1"/></Sequence>)";
    const auto turning   = buildLearned(look, learnable(), {}, problems);
    ASSERT_TRUE(turning.has_value());
    EXPECT_EQ(turning->skill.risk, "motion");
    EXPECT_THAT(turning->skill.resources, ::testing::ElementsAre("base"));
    EXPECT_THAT(turning->macro.xml, ::testing::Not(HasSubstr("both_hands_must_be_empty")));
}

TEST(LearnedSkill, ALeafOpensOnlyTheInputsItsEntryLists)
{
    const Learnable narrow =
        Learnable::parse({ "MoveHand:arms:arm,target(world_id),mode", "Wait::seconds" }, kPalette);
    EXPECT_THAT(narrow.ports.at("MoveHand"), ::testing::SizeIs(3));
    SkillDefinition d = pointAt();
    d.body_xml        = R"(<MoveHand arm="{arm}" target="{object}" frame="torso_link"/>)";
    std::vector<Diagnostic> problems;
    EXPECT_FALSE(buildLearned(d, narrow, {}, problems).has_value());
    ASSERT_FALSE(problems.empty());
    EXPECT_EQ(problems.front().code, "UNKNOWN_ATTRIBUTE");
    EXPECT_THROW(Learnable::parse({ "MoveHand:arms:find_service" }, kPalette), std::runtime_error);
    EXPECT_TRUE(Learnable::parse({ "Wait::" }, kPalette).ports.at("Wait").empty());
}

TEST(LearnedSkill, MoveHandTakesATargetOrAFrameAndAWorldIdOnlyAsItsTarget)
{
    const Learnable rules = Learnable::parse(
        { "MoveHand:arms:arm,target(world_id)|frame,mode", "Wait::seconds" },
        kPalette);
    EXPECT_THAT(rules.ports.at("MoveHand"), ::testing::SizeIs(4));
    const auto check = [&rules](const std::string& body) {
        SkillDefinition d = pointAt();
        d.body_xml        = body;
        std::vector<Diagnostic> problems;
        const bool              built = buildLearned(d, rules, {}, problems).has_value();
        EXPECT_EQ(built, problems.empty());
        return problems;
    };
    EXPECT_TRUE(check(R"(<MoveHand arm="{arm}" target="{object}" mode="point"/>)").empty());
    EXPECT_TRUE(check(R"(<MoveHand arm="{arm}" frame="torso_link"/>)").empty());

    // What a model taught on 15 Oct: each failed only when the step ran, after two approvals.
    const auto both = check(R"(<MoveHand arm="{arm}" target="{object}" frame="map"/>)");
    ASSERT_THAT(both, ::testing::SizeIs(1));
    EXPECT_EQ(both.front().code, "FORBIDDEN_ATTRIBUTE");
    EXPECT_THAT(both.front().message, HasSubstr("only one of target"));
    EXPECT_EQ(check(R"(<MoveHand arm="{arm}" mode="point"/>)").front().code, "MISSING_ATTRIBUTE");
    const auto as_frame = check(R"(<MoveHand arm="{arm}" frame="{object}"/>)");
    ASSERT_THAT(as_frame, ::testing::SizeIs(1));
    EXPECT_EQ(as_frame.front().code, "BAD_ARG");
    EXPECT_THAT(as_frame.front().message, HasSubstr("give it to target"));
    EXPECT_EQ(
        check(
            R"(<Sequence><MoveHand arm="{arm}" target="{object}"/><Wait seconds="{object}"/></Sequence>)")
            .front()
            .code,
        "BAD_ARG")
        << "a leaf whose inputs are listed takes no world id unless one is marked";
}

TEST(LearnedSkill, AnInputsValuesHoldItsLiteralsAndTheArgumentFedToIt)
{
    const Learnable rules = Learnable::parse(
        { "MoveHand:arms:arm=left/right,target(world_id)|frame,mode=reach/point", "Wait::seconds" },
        kPalette);
    EXPECT_THAT(rules.values.at("MoveHand").at("arm"), ::testing::ElementsAre("left", "right"));
    EXPECT_TRUE(rules.ports.at("MoveHand").contains("arm")) << "the values leave the port's name";

    // Every taught PointAt of 23 Oct first ran with arm=left_arm, refused only once it ran.
    std::vector<Diagnostic> problems;
    const auto              learned = buildLearned(pointAt(), rules, {}, problems);
    ASSERT_TRUE(learned.has_value());
    const auto arm = std::ranges::find_if(learned->skill.args, [](const SkillArg& a) {
        return a.name == "arm";
    });
    ASSERT_NE(arm, learned->skill.args.end());
    EXPECT_THAT(arm->allowed, ::testing::ElementsAre("left", "right"));

    SkillDefinition literal = pointAt();
    literal.body_xml        = R"(<MoveHand arm="left_arm" target="{object}" mode="point"/>)";
    problems.clear();
    EXPECT_FALSE(buildLearned(literal, rules, {}, problems).has_value());
    ASSERT_THAT(problems, ::testing::SizeIs(1));
    EXPECT_EQ(problems.front().code, "BAD_ARG");
    EXPECT_THAT(problems.front().message, HasSubstr("takes only left or right, not 'left_arm'"));
}

TEST(LearnedSkill, OnlyTheControlNodesAndLearnableLeavesWithLiteralsOrItsArgumentsPass)
{
    SkillDefinition d = pointAt();
    d.body_xml        = R"(<Sequence><Pick object_id="{object}"/></Sequence>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("FORBIDDEN_NODE"));

    d.body_xml = R"(<Parallel><Wait seconds="1"/></Parallel>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("FORBIDDEN_NODE"));

    d.body_xml = R"(<MoveHand arm="{arm}" target="{object}" speed_up="9"/>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("UNKNOWN_ATTRIBUTE"));

    d.body_xml = R"(<MoveHand arm="{hand}" target="{object}"/>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_ARG"));

    d.body_xml = R"(<MoveHand arm="left{arm}" target="{object}"/>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_ARG"));

    d.body_xml =
        R"(<RetryUntilSuccessful num_attempts="5"><Wait seconds="1"/></RetryUntilSuccessful>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_RETRIES"));

    d.body_xml = R"(<Wait seconds="1"/><Wait seconds="1"/>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_STRUCTURE"));
}

TEST(LearnedSkill, ItsNameArgumentsAndTimeAreChecked)
{
    SkillDefinition d = pointAt();
    d.name            = "point_at";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_NAME"));
    d.name = "PickObject";
    EXPECT_THAT(problemsOf(d, { "PickObject" }), ::testing::Contains("DUPLICATE_NAME"));
    d.name = "Wait";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("DUPLICATE_NAME"));

    d          = pointAt();
    d.args     = { arg("Arm") };
    d.body_xml = R"(<Wait seconds="1"/>)";
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_ARG"));

    d                = pointAt();
    d.max_duration_s = 900.0;
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_TIMEOUT"));
    d.max_duration_s = 0.0004;
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_TIMEOUT")) << "it would round to 0 ms";

    d      = pointAt();
    d.args = { arg("object", "world_id"), arg("name") };
    EXPECT_THAT(problemsOf(d), ::testing::Contains("BAD_ARG")) << "a SubTree's own attribute";
}

TEST(LearnedSkills, ADraftIsKeptOnDiskAndLoadsAgainUntilForgotten)
{
    const auto dir = std::filesystem::temp_directory_path() / "g1_learned_skills_test";
    std::filesystem::remove_all(dir);
    {
        LearnedSkills skills(dir, learnable(), 4);
        EXPECT_TRUE(skills.define(pointAt(), /*keep=*/false, {}).empty());
        ASSERT_EQ(skills.all().size(), 1U);
        EXPECT_EQ(skills.all().front().skill.learned, "draft");
        EXPECT_FALSE(std::filesystem::exists(dir / "PointAt.json"));
        EXPECT_TRUE(skills.define(pointAt(), /*keep=*/true, {}).empty());
        EXPECT_TRUE(std::filesystem::exists(dir / "PointAt.json"));
        EXPECT_EQ(skills.all().size(), 1U) << "kept replaces its draft";
    }
    LearnedSkills again(dir, learnable(), 4);
    EXPECT_TRUE(again.load({}).empty());
    ASSERT_EQ(again.all().size(), 1U);
    EXPECT_EQ(again.all().front().skill.learned, "kept");
    EXPECT_EQ(again.all().front().definition.taught_for, "point at the red mug");
    EXPECT_THAT(again.define(pointAt(), /*keep=*/false, {}), ::testing::SizeIs(1))
        << "a draft does not replace a kept skill";
    EXPECT_TRUE(again.forget("PointAt"));
    EXPECT_FALSE(std::filesystem::exists(dir / "PointAt.json"));
    EXPECT_FALSE(again.forget("PointAt"));
    std::filesystem::remove_all(dir);
}

TEST(LearnedSkills, AFileNotNamedForItsSkillOrOverTheCapIsLeftOut)
{
    const auto dir = std::filesystem::temp_directory_path() / "g1_learned_skills_files";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    {
        LearnedSkills writer(dir, learnable(), 4);
        ASSERT_TRUE(writer.define(pointAt(), /*keep=*/true, {}).empty());
    }
    std::filesystem::copy_file(dir / "PointAt.json", dir / "OldCopy.json");
    LearnedSkills reader(dir, learnable(), 4);
    const auto    skipped = reader.load({});
    ASSERT_THAT(skipped, ::testing::SizeIs(1));
    EXPECT_THAT(skipped.front(), HasSubstr("OldCopy.json"));

    // A file left by an earlier run goes with forget even when this run never loaded it.
    LearnedSkills fresh(dir, learnable(), 0);
    EXPECT_THAT(fresh.load({}), ::testing::SizeIs(2)) << "over a cap of none, both are left out";
    EXPECT_TRUE(fresh.forget("PointAt"));
    EXPECT_FALSE(std::filesystem::exists(dir / "PointAt.json"));
    std::filesystem::remove_all(dir);
}

TEST(Repertoire, ALearnedSkillJoinsTheSetAndAMissionStartedBeforeKeepsItsOwn)
{
    Repertoire repertoire(
        Catalog::load(std::string(G1_CONFIG_DIR) + "/catalog.yaml"),
        MacroLibrary::load(std::string(G1_TREES_DIR) + "/library"),
        kPalette,
        NameSet{ "Pick", "Sequence" },
        Limits{},
        LearnedSkills({}, learnable(), 4));
    const auto      before = repertoire.current();
    SkillDefinition look   = pointAt();
    look.name              = "LookAround";
    look.args              = {};
    look.body_xml          = R"(<Sequence><TurnBy degrees="90"/><Wait seconds="1"/></Sequence>)";
    ASSERT_TRUE(repertoire.define(look, /*keep=*/false).empty());

    const auto        after   = repertoire.current();
    const std::string mission = R"(<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission"><Sequence><SubTree ID="LookAround" name="s1_LookAround"/></Sequence></BehaviorTree>
</root>)";
    EXPECT_TRUE(after->validator.validate(mission).ok());
    EXPECT_FALSE(before->validator.validate(mission).ok());
    EXPECT_NE(before->validator.catalog().version(), after->validator.catalog().version());
    EXPECT_THAT(after->catalog_json, HasSubstr(R"("learned":"draft")"));
    EXPECT_THAT(after->catalog_json, HasSubstr(R"("learnable")"));
    EXPECT_THAT(after->palette_xml, HasSubstr(R"(SubTree ID="LookAround")"));

    SkillDefinition taken = look;
    taken.name            = "PickObject";
    EXPECT_FALSE(repertoire.define(taken, false).empty()) << "a library skill's name is taken";
    taken.name = "Pick";
    EXPECT_FALSE(repertoire.define(taken, false).empty()) << "so is a registered leaf's";

    SkillDefinition slower = look;
    slower.body_xml        = R"(<Sequence><TurnBy degrees="90"/><Wait seconds="3"/></Sequence>)";
    ASSERT_TRUE(repertoire.define(slower, /*keep=*/false).empty());
    EXPECT_NE(
        repertoire.current()->validator.catalog().version(),
        after->validator.catalog().version())
        << "a new body is a new catalog";
    EXPECT_TRUE(repertoire.forget("LookAround"));
    EXPECT_FALSE(repertoire.current()->validator.validate(mission).ok());
}
