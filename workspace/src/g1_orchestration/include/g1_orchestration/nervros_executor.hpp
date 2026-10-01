#ifndef G1_ORCHESTRATION__NERVROS_EXECUTOR_HPP_
#define G1_ORCHESTRATION__NERVROS_EXECUTOR_HPP_

/**
 * @file nervros_executor.hpp
 * @brief Runs the behavior trees a NervROS agent sends, one mission at a time.
 *
 * The agent compiles a plan to a tree of catalog skills. This node checks it (mission_validator),
 * loads it against the library macros with a factory of its own, takes the arms once for the whole
 * mission, ticks it while reporting what changes, and releases the arms afterwards only when both
 * hands are empty, since a released Dex3 hand goes limp and drops what it holds. A hand is not
 * known to be empty when the arms were already taken before this executor found them, or when a
 * pick was cut off after its goal went out; the arms are then kept, and the hand is treated as
 * holding something, until a place with that arm succeeds.
 *
 * Threads: the action server, the services and the state publisher run on a multi-threaded
 * executor, so StopAll and ValidateMission answer while a mission runs. The mission runs on a
 * thread of its own, and that thread also spins the node the leaves use, between ticks, so leaf
 * callbacks and tree destruction never overlap. A tick can still take long, in a blocking call,
 * so a stop, a cancel or the watchdog also sets a flag the leaves look at before they start
 * anything (RosContext::stop).
 */

#include <behaviortree_cpp/bt_factory.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <nervros_interfaces/action/execute_mission.hpp>
#include <nervros_interfaces/msg/robot_state.hpp>
#include <nervros_interfaces/srv/get_catalog.hpp>
#include <nervros_interfaces/srv/stop_all.hpp>
#include <nervros_interfaces/srv/validate_mission.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "g1_orchestration/authority_port.hpp"
#include "g1_orchestration/mission_observer.hpp"
#include "g1_orchestration/mission_validator.hpp"
#include "g1_orchestration/server_fleet.hpp"

namespace g1_orchestration
{

class MissionExecutor : public rclcpp::Node
{
public:
    using ExecuteMission = nervros_interfaces::action::ExecuteMission;
    using GoalHandle     = rclcpp_action::ServerGoalHandle<ExecuteMission>;
    using RobotState     = nervros_interfaces::msg::RobotState;

    /**
     * @param authority How the arms are taken and released, or null for controller_manager.
     * @param options Node options; tests use them to override parameters.
     * @throws std::exception If a parameter is unusable or the catalog or macros cannot be read.
     */
    explicit MissionExecutor(
        std::shared_ptr<AuthorityPort> authority = nullptr,
        const rclcpp::NodeOptions&     options   = rclcpp::NodeOptions());

    ~MissionExecutor() override;

    MissionExecutor(const MissionExecutor&)            = delete;
    MissionExecutor& operator=(const MissionExecutor&) = delete;
    MissionExecutor(MissionExecutor&&)                 = delete;
    MissionExecutor& operator=(MissionExecutor&&)      = delete;

    /**
     * @brief Ends the mission if one runs, then releases the arms unless a hand holds something.
     *
     * Call it once, while the executor is still spinning, before shutting ROS down.
     */
    void shutdown();

    /**
     * @brief How long a mission may run before the watchdog ends it.
     *
     * @param requested_s What the goal asked for; not positive, or not finite, means no limit.
     * @param worst_case_s The mission's worst case, in seconds.
     * @param factor The margin over the worst case when no limit was asked for.
     * @param cap_s The most any mission may run.
     * @return @p requested_s, else the worst case times @p factor, and never more than @p cap_s.
     */
    [[nodiscard]] static double
    watchdogLimitS(double requested_s, double worst_case_s, double factor, double cap_s);

private:
    /// A hand's load: the id the mission named it by, the id the detector reports it under (the
    /// slug of that, which Place looks for after letting go), and the words it found it by. Or
    /// `unknown`: it may hold something the executor does not know about.
    struct Held
    {
        std::string id;
        std::string seen_as;
        std::string phrase;
        bool        unknown = false;
    };

    struct Hands
    {
        Held left;
        Held right;
    };

    /// What one mission ended with, before it is shaped into a result.
    struct Outcome
    {
        std::uint8_t code = ExecuteMission::Result::OUTCOME_SUCCESS;
        std::string  failed_node;
        std::string  failed_step;
        std::string  reason;
        std::string  diagnostics = "[]";
    };

    /// Why the tick loop stopped.
    enum class Ended
    {
        kDone,
        kCanceled,
        kStopped,
        kTimeout,
        kShutdown,
    };

    // --- ROS surface -------------------------------------------------------------------------
    static rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&                     uuid,
        const std::shared_ptr<const ExecuteMission::Goal>& goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>& handle);
    /// Starts the mission, or turns the goal away. Nothing escapes it: a callback that threw would
    /// take the executor's thread, and the process, with it.
    void handleAccepted(const std::shared_ptr<GoalHandle>& handle);
    void startMission(const std::shared_ptr<GoalHandle>& handle);
    void rejectGoal(
        const std::shared_ptr<GoalHandle>& handle, const Diagnostic& diagnostic,
        const std::string& reason);
    void onValidate(
        const std::shared_ptr<nervros_interfaces::srv::ValidateMission::Request>&  request,
        const std::shared_ptr<nervros_interfaces::srv::ValidateMission::Response>& response);
    void onCatalog(
        const std::shared_ptr<nervros_interfaces::srv::GetCatalog::Request>&  request,
        const std::shared_ptr<nervros_interfaces::srv::GetCatalog::Response>& response);
    void onStopAll(
        const std::shared_ptr<nervros_interfaces::srv::StopAll::Request>&  request,
        const std::shared_ptr<nervros_interfaces::srv::StopAll::Response>& response);

    // --- one mission -------------------------------------------------------------------------
    void    run(const std::shared_ptr<GoalHandle>& handle);
    Outcome execute(const ExecuteMission::Goal& goal, const std::shared_ptr<GoalHandle>& handle);
    static Outcome rejected(const std::vector<Diagnostic>& diagnostics);

    /// A factory with the real leaves, or with stand-ins that only wait, and the macros @p used.
    BT::BehaviorTreeFactory
    makeFactory(bool dry_run, const Validation& validation, const std::set<std::string>& used);
    /// A blackboard holding what each hand holds, as "id=phrase" or empty, for the macros to read.
    BT::Blackboard::Ptr seededBlackboard();
    /// Loads @p xml into @p tree, its number arguments written as decimals, or says why not.
    std::optional<Diagnostic> load(
        BT::BehaviorTreeFactory& factory, const std::string& xml,
        const BT::Blackboard::Ptr& blackboard, BT::Tree& tree) const;

    bool  takeAuthority(const Validation& validation, Outcome& outcome);
    Ended tickLoop(
        BT::Tree& tree, MissionObserver& observer, const std::shared_ptr<GoalHandle>& handle,
        double limit_s, std::chrono::steady_clock::time_point begin, BT::NodeStatus& status);
    /// Why the mission has to end now, if it does: shutdown, a stop, a cancel or the watchdog.
    [[nodiscard]] std::optional<Ended> interruption(
        const std::shared_ptr<GoalHandle>& handle, std::chrono::steady_clock::time_point begin,
        double limit_s) const;
    /// Raises the stop flag once the running mission is past its limit, even inside a long tick.
    void        watchdog();
    static void sendFeedback(
        const std::shared_ptr<GoalHandle>& handle, MissionObserver& observer,
        std::chrono::steady_clock::time_point begin);
    /// Halts the tree and spins until its cancels have gone out and the servers are quiet.
    std::string halt(BT::Tree& tree);
    /// What StopAll tells the caller, from what actually happened.
    [[nodiscard]] std::string describeStop(
        bool mission, bool halted, std::size_t leaf_servers_asked, std::size_t arm_asked,
        const std::vector<std::string>& busy) const;
    void spinLeaves(std::chrono::duration<double> span);
    void finish(bool execute_mode);

    // --- what the robot holds ----------------------------------------------------------------
    void noteLeafSuccess(const BT::TreeNode& node);
    /// A pick that was halted while it ran may have got as far as the grasp, and nothing will say.
    void noteLeafHalted(const BT::TreeNode& node);
    void pushState();
    /// "; right hand keeps O17", or empty when both hands are; the caller holds live_mutex_.
    [[nodiscard]] std::string describeHandsLocked(const char* verb) const;
    /// What the mission reasons with: the real hands, or a copy of them for a dry run. The caller
    /// holds live_mutex_.
    Hands& handsInUseLocked() { return dry_running_ ? dry_hands_ : hands_; }
    /// Copies what the hands hold into the running mission's blackboard; the caller holds
    /// live_mutex_.
    void syncBlackboardLocked();
    /// Whether a hand holds something, or may; the caller holds live_mutex_.
    [[nodiscard]] bool mayHoldSomethingLocked() const;
    static std::string entryOf(const Held& held);

    // --- parameters and parts ----------------------------------------------------------------
    double                 tick_rate_hz_;
    double                 max_duration_cap_s_;
    int                    groot2_port_;
    bool                   hands_empty_on_attach_;
    double                 duration_factor_;
    double                 dry_run_time_scale_;
    double                 dry_run_min_leaf_s_;
    double                 feedback_period_s_;
    double                 cancel_settle_s_;
    double                 stop_wait_s_;
    MissionObserver::Level event_level_;

    std::unique_ptr<MissionValidator> validator_;
    std::string                       palette_xml_;
    /// Built once at start, so a catalog that cannot be written as JSON stops the executor there.
    std::string                    catalog_json_;
    std::shared_ptr<AuthorityPort> authority_;

    rclcpp::CallbackGroup::SharedPtr action_group_;
    rclcpp::CallbackGroup::SharedPtr stop_group_;
    rclcpp::CallbackGroup::SharedPtr query_group_;
    rclcpp::CallbackGroup::SharedPtr state_group_;
    /// The servers' clients and status subscriptions: mutually exclusive, so that status messages
    /// are handled in order and the newest is the one that stays.
    rclcpp::CallbackGroup::SharedPtr status_group_;

    /// The node the leaves use, spun by the mission thread and by nothing else.
    rclcpp::Node::SharedPtr                   skills_node_;
    rclcpp::executors::SingleThreadedExecutor skills_executor_;

    std::unique_ptr<ServerFleet> leaf_servers_;
    std::unique_ptr<ServerFleet> arm_controller_;

    rclcpp_action::Server<ExecuteMission>::SharedPtr                     action_server_;
    rclcpp::Service<nervros_interfaces::srv::ValidateMission>::SharedPtr validate_service_;
    rclcpp::Service<nervros_interfaces::srv::GetCatalog>::SharedPtr      catalog_service_;
    rclcpp::Service<nervros_interfaces::srv::StopAll>::SharedPtr         stop_service_;
    rclcpp::Publisher<RobotState>::SharedPtr                             state_pub_;
    rclcpp::TimerBase::SharedPtr                                         state_timer_;

    // --- shared between the mission thread, StopAll and the state publisher ------------------
    std::mutex  live_mutex_;
    std::string mission_id_;
    std::string mission_step_;
    bool        base_held_ = false;
    bool        arms_held_ = false;
    bool        stopped_   = false;
    std::string note_;
    Hands       hands_;
    /// What a dry run's what-if hands hold, so that it can walk a pick into a place without
    /// touching what the robot's hands really hold.
    Hands dry_hands_;
    bool  dry_running_ = false;
    /// The running mission's root blackboard, kept in step with what the hands hold.
    BT::Blackboard::Ptr mission_blackboard_;

    // A mission starts and ends, and a stop lands, under stop_mutex_: a stop is then always
    // for the mission that was running when it was asked for, and cannot end the next one.
    std::mutex              stop_mutex_;
    std::atomic<bool>       running_{ false };
    std::atomic<bool>       stop_requested_{ false };
    std::string             stop_reason_;
    rclcpp_action::GoalUUID goal_id_{};
    /// Set by a stop, a cancel, the watchdog or a shutdown: what the leaves look at.
    std::atomic<bool> halting_{ false };
    /// When the watchdog fires, in steady-clock ticks; 0 when no mission runs.
    std::atomic<std::chrono::steady_clock::rep> deadline_ticks_{ 0 };
    std::atomic<bool>                           shutting_down_{ false };
    std::mutex                                  halted_mutex_;
    std::condition_variable                     halted_cv_;
    bool                                        halted_ = true;

    /// Guards worker_, which the action callback and shutdown() both touch.
    std::mutex worker_mutex_;
    /// Last, so it joins before anything it uses is destroyed.
    std::jthread worker_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__NERVROS_EXECUTOR_HPP_
