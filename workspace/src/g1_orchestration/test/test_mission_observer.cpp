/**
 * @file test_mission_observer.cpp
 * @brief What the executor learns from a tree as it ticks: events, the running step, the failure.
 *
 * Trees here are built from stand-in leaves, so no ROS graph is involved.
 */

#include <behaviortree_cpp/bt_factory.h>
#include <gmock/gmock.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "g1_orchestration/leaf_report.hpp"
#include "g1_orchestration/mission_observer.hpp"

namespace
{

using g1_orchestration::MissionObserver;

/// Fails, and says why, the way a skill leaf does.
class Reporting : public BT::SyncActionNode, public g1_orchestration::LeafReport
{
public:
    Reporting(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("why") }; }

    BT::NodeStatus tick() override
    {
        setFailureText(getInput<std::string>("why").value_or(""));
        return BT::NodeStatus::FAILURE;
    }
};

/// Never finishes on its own.
class Forever : public BT::StatefulActionNode
{
public:
    Forever(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override { return BT::NodeStatus::RUNNING; }
    BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }
    void           onHalted() override {}
};

struct Rig
{
    BT::BehaviorTreeFactory  factory;
    BT::Tree                 tree;
    std::vector<std::string> picked;

    Rig()
    {
        factory.registerSimpleAction(
            "Pick",
            [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; },
            { BT::InputPort<std::string>("object_id"), BT::InputPort<std::string>("arm") });
        factory.registerSimpleAction("Ok", [](BT::TreeNode&) { return BT::NodeStatus::SUCCESS; });
        factory.registerSimpleAction("Nope", [](BT::TreeNode&) { return BT::NodeStatus::FAILURE; });
        factory.registerNodeType<Reporting>("Reporting");
        factory.registerNodeType<Forever>("Forever");
    }

    void load(const std::string& mission, const std::string& macros = "")
    {
        if (!macros.empty())
        {
            factory.registerBehaviorTreeFromText(macros);
        }
        tree = factory.createTreeFromText(
            R"(<root BTCPP_format="4" main_tree_to_execute="Mission"><BehaviorTree ID="Mission">)" +
            mission + "</BehaviorTree></root>");
    }

    std::unique_ptr<MissionObserver>
    watch(MissionObserver::Level level, MissionObserver::Hooks hooks = {}) const
    {
        return std::make_unique<MissionObserver>(
            tree.rootNode(),
            std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME),
            level,
            std::set<std::string>{ "Pick", "Ok", "Nope", "Reporting", "Forever" },
            std::move(hooks));
    }

    BT::NodeStatus run()
    {
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        for (int i = 0; i < 200 && status == BT::NodeStatus::RUNNING; ++i)
        {
            status = tree.tickOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return status;
    }
};

const char* const kMacro = R"(<root BTCPP_format="4"><BehaviorTree ID="Macro">
  <Sequence name="inside"><Ok name="first"/><Ok name="second"/></Sequence></BehaviorTree></root>)";

std::vector<std::string> namesOf(const std::vector<nervros_interfaces::msg::NodeEvent>& events)
{
    std::vector<std::string> names;
    names.reserve(events.size());
    for (const auto& event : events)
    {
        names.push_back(event.name);
    }
    return names;
}

}  // namespace

TEST(StepIds, ComeFromANamePlannersUse)
{
    using g1_orchestration::stepIdOf;
    EXPECT_EQ(stepIdOf("s1_PickObject"), "s1");
    EXPECT_EQ(stepIdOf("s12_GoToPose"), "s12");
    for (const char* name :
         { "", "s", "s_x", "s1", "s1x", "x1_a", "S1_a", "1_a", "Sequence", "s1a_b" })
    {
        EXPECT_EQ(stepIdOf(name), "") << name;
    }
}

TEST(StepIds, AreTheFirstOneOnAPath)
{
    using g1_orchestration::stepOfPath;
    EXPECT_EQ(stepOfPath("s3_Pick/pick_the_object"), "s3");
    EXPECT_EQ(stepOfPath("s10_Place/inside/Sequence::7"), "s10");
    EXPECT_EQ(stepOfPath("Timeout::4"), "");
    EXPECT_EQ(stepOfPath("mission"), "");
    EXPECT_EQ(stepOfPath(""), "");
}

TEST(MissionObserver, KeepsStepsAndSkillLeavesAtTheStepLevel)
{
    Rig rig;
    rig.load(R"(<Sequence name="mission"><SubTree ID="Macro" name="s1_Macro"/></Sequence>)", kMacro);
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    EXPECT_EQ(rig.run(), BT::NodeStatus::SUCCESS);

    const auto events = observer->takeEvents();
    EXPECT_THAT(namesOf(events), ::testing::Contains("s1_Macro"));
    EXPECT_THAT(namesOf(events), ::testing::Contains("first"));
    EXPECT_THAT(namesOf(events), ::testing::Not(::testing::Contains("mission")))
        << "a control node is not a step";
    EXPECT_THAT(namesOf(events), ::testing::Not(::testing::Contains("inside")));
    const auto step =
        std::ranges::find(events, "s1_Macro", &nervros_interfaces::msg::NodeEvent::name);
    ASSERT_NE(step, events.end());
    EXPECT_EQ(step->registration_id, "SubTree");
    EXPECT_EQ(step->path, "s1_Macro");
    EXPECT_EQ(step->prev_status, 0U) << "IDLE";
    EXPECT_EQ(step->status, 1U) << "RUNNING";
}

TEST(MissionObserver, KeepsEveryNodeAtTheAllLevel)
{
    Rig rig;
    rig.load(R"(<Sequence name="mission"><SubTree ID="Macro" name="s1_Macro"/></Sequence>)", kMacro);
    auto observer = rig.watch(MissionObserver::Level::kAll);

    rig.run();

    EXPECT_THAT(
        namesOf(observer->takeEvents()),
        ::testing::IsSupersetOf({ "mission", "s1_Macro", "inside", "first", "second" }));
}

TEST(MissionObserver, ReportsAHaltButNotAReset)
{
    Rig rig;
    rig.load(R"(<Sequence name="mission"><Ok name="quick"/><Forever name="long"/></Sequence>)");
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    rig.tree.tickOnce();
    rig.tree.tickOnce();
    rig.tree.haltTree();

    const auto events = observer->takeEvents();
    const auto halted = std::ranges::find_if(events, [](const auto& e) {
        return e.name == "long" && e.status == 0;
    });
    ASSERT_NE(halted, events.end()) << "the halted node must show RUNNING to IDLE";
    EXPECT_EQ(halted->prev_status, 1U);
    const auto reset = std::ranges::find_if(events, [](const auto& e) {
        return e.name == "quick" && e.status == 0;
    });
    EXPECT_EQ(reset, events.end()) << "SUCCESS to IDLE is BehaviorTree.CPP tidying up, not news";
}

TEST(MissionObserver, KnowsWhatIsRunningAndWhichStepStartedLast)
{
    Rig rig;
    rig.load(
        R"(<Sequence name="mission"><SubTree ID="Macro" name="s1_Macro"/><SubTree ID="Macro" name="s2_Macro"/><Forever name="long"/></Sequence>)",
        kMacro);
    std::vector<std::string> steps;
    MissionObserver::Hooks   hooks;
    hooks.on_step = [&steps](const std::string& step) { steps.push_back(step); };
    auto observer = rig.watch(MissionObserver::Level::kSteps, hooks);

    rig.tree.tickOnce();

    EXPECT_THAT(steps, ::testing::ElementsAre("s1_Macro", "s2_Macro"));
    EXPECT_EQ(observer->currentStep(), "s2_Macro");
    EXPECT_THAT(observer->running(), ::testing::Contains("long"));
    EXPECT_THAT(observer->running(), ::testing::Not(::testing::Contains("s1_Macro")))
        << "it finished";
}

TEST(MissionObserver, GivesTheHookTheLeafItself)
{
    Rig rig;
    rig.load(R"(<Sequence><Pick object_id="red_block" arm="left"/></Sequence>)");
    std::string            id;
    std::string            arm;
    MissionObserver::Hooks hooks;
    hooks.on_success = [&](const BT::TreeNode& node) {
        if (node.registrationName() == "Pick")
        {
            id  = node.getInput<std::string>("object_id").value_or("");
            arm = node.getInput<std::string>("arm").value_or("");
        }
    };
    auto observer = rig.watch(MissionObserver::Level::kSteps, hooks);

    rig.run();

    EXPECT_EQ(id, "red_block");
    EXPECT_EQ(arm, "left");
}

TEST(MissionObserver, TellsTheHookWhichNodesWereHaltedWhileTheyRan)
{
    Rig rig;
    // The Timeout halts what it wraps from its own timer thread, which no executor code sees.
    rig.load(
        R"(<Sequence><Ok name="fine"/><Timeout msec="60"><Forever name="stuck"/></Timeout></Sequence>)");
    std::mutex               mutex;
    std::vector<std::string> halted;
    MissionObserver::Hooks   hooks;
    hooks.on_halt = [&](const BT::TreeNode& node) {
        const std::lock_guard<std::mutex> lock(mutex);
        halted.push_back(node.name());
    };
    auto observer = rig.watch(MissionObserver::Level::kSteps, hooks);

    EXPECT_EQ(rig.run(), BT::NodeStatus::FAILURE);

    const std::lock_guard<std::mutex> lock(mutex);
    EXPECT_THAT(halted, ::testing::Contains("stuck"));
    EXPECT_THAT(halted, ::testing::Not(::testing::Contains("fine")))
        << "a node that finished and was reset was not halted";
}

TEST(MissionObserver, TheFailureIsTheLastLeafThatFailedBeforeTheTreeDid)
{
    Rig rig;
    // The first failure is made optional; the mission ends on the second.
    rig.load(R"(<Sequence><ForceSuccess><Reporting name="optional" why="first"/></ForceSuccess>
                          <Reporting name="fatal" why="second"/></Sequence>)");
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    EXPECT_EQ(rig.run(), BT::NodeStatus::FAILURE);

    ASSERT_TRUE(observer->failure().has_value());
    EXPECT_EQ(observer->failure()->path, "fatal");
    EXPECT_EQ(observer->failure()->reason, "second");
    EXPECT_FALSE(observer->failure()->timed_out);
}

TEST(MissionObserver, ARetriedFailureThatRecoveredIsNotTheMissionsFailure)
{
    Rig rig;
    rig.load(R"(<Sequence><Fallback><Reporting name="flaky" why="once"/><Ok name="fine"/></Fallback>
                          <Nope name="last"/></Sequence>)");
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    rig.run();

    ASSERT_TRUE(observer->failure().has_value());
    EXPECT_EQ(observer->failure()->path, "last");
    EXPECT_EQ(observer->failure()->reason, "last failed")
        << "no words from the leaf: say which one";
}

TEST(MissionObserver, FindsTheStepOfAFailureFromItsPath)
{
    Rig rig;
    rig.load(
        R"(<Sequence><SubTree ID="Boom" name="s2_Boom"/></Sequence>)",
        R"(<root BTCPP_format="4"><BehaviorTree ID="Boom">
        <Sequence><Ok name="fine"/><Reporting name="broke" why="the grasp missed"/></Sequence></BehaviorTree></root>)");
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    rig.run();

    ASSERT_TRUE(observer->failure().has_value());
    EXPECT_EQ(observer->failure()->path, "s2_Boom/broke");
    EXPECT_EQ(observer->failure()->step, "s2");
    EXPECT_EQ(observer->failure()->reason, "the grasp missed");
}

TEST(MissionObserver, ATimeoutThatGivesUpIsTheFailureAndKnowsItsStep)
{
    Rig rig;
    // The step's own Timeout has no name and no step in its path; the step is what it wraps.
    rig.load(
        R"(<Timeout msec="60"><SubTree ID="Hang" name="s3_Hang"/></Timeout>)",
        R"(<root BTCPP_format="4"><BehaviorTree ID="Hang"><Forever name="stuck"/></BehaviorTree></root>)");
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    EXPECT_EQ(rig.run(), BT::NodeStatus::FAILURE);

    ASSERT_TRUE(observer->failure().has_value());
    EXPECT_TRUE(observer->failure()->timed_out);
    EXPECT_EQ(observer->failure()->step, "s3");
    EXPECT_THAT(observer->failure()->reason, ::testing::StartsWith("timed out after"));
}

TEST(MissionObserver, ATimeoutThatOnlyPassesOnItsChildsFailureIsNotATimeout)
{
    Rig rig;
    rig.load(
        R"(<Sequence><Timeout msec="5000"><Reporting name="leaf" why="the grasp missed"/></Timeout></Sequence>)");
    auto observer = rig.watch(MissionObserver::Level::kSteps);

    EXPECT_EQ(rig.run(), BT::NodeStatus::FAILURE);

    ASSERT_TRUE(observer->failure().has_value());
    EXPECT_EQ(observer->failure()->path, "leaf")
        << "the leaf, not the Timeout that carried its failure up";
    EXPECT_FALSE(observer->failure()->timed_out);
    EXPECT_EQ(observer->failure()->reason, "the grasp missed");
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
