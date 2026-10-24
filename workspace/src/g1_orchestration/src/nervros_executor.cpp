/**
 * @file nervros_executor.cpp
 * @brief The mission executor: goals in, ticks, state and stops.
 */

#include "g1_orchestration/nervros_executor.hpp"

#include <behaviortree_cpp/actions/test_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/decorators/subtree_node.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <behaviortree_cpp/contrib/json.hpp>
#include <cmath>
#include <filesystem>
#include <format>
#include <limits>
#include <map>
#include <optional>
#include <ranges>
#include <set>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "g1_orchestration/catalog.hpp"
#include "g1_orchestration/leaf_report.hpp"
#include "g1_orchestration/macro_library.hpp"
#include "g1_orchestration/registration.hpp"
#include "g1_orchestration/skill_nodes.hpp"

namespace g1_orchestration
{

namespace
{

using Seconds = std::chrono::duration<double>;
using Clock   = std::chrono::steady_clock;

constexpr const char* kPackage = "g1_orchestration";

/// What the macros read from held_left and held_right for a hand that may hold something they
/// cannot name. An id from the detector is a slug, so it can never be this.
constexpr const char* kUnknownHeld = "?";

/// What the steps handed on before the mission ended, by key, as a JSON object of text.
std::string outputsJson(const BT::Blackboard& blackboard, const std::set<std::string>& keys)
{
    nlohmann::json out = nlohmann::json::object();
    for (const std::string& key : keys)
    {
        // An entry a step never wrote, or not as text, hands nothing on.
        const auto any = blackboard.getAnyLocked(key);
        if (!any || any->empty())
        {
            continue;
        }
        if (const auto value = any->tryCast<std::string>())
        {
            out[key] = *value;
        }
    }
    return out.dump();
}

std::filesystem::path resolve(const std::string& configured)
{
    const std::filesystem::path path(configured);
    return path.is_absolute() ?
               path :
               std::filesystem::path(ament_index_cpp::get_package_share_directory(kPackage)) / path;
}

double positive(double value, const char* name)
{
    if (!std::isfinite(value) || value <= 0.0)
    {
        throw std::invalid_argument(
            std::format("{} must be positive and finite, got {}", name, value));
    }
    return value;
}

Clock::duration inClockUnits(double seconds)
{
    return std::chrono::duration_cast<Clock::duration>(Seconds(seconds));
}

std::string join(const std::vector<std::string>& words)
{
    std::string out;
    for (const std::string& word : words)
    {
        out += out.empty() ? "" : ", ";
        out += word;
    }
    return out;
}

MissionObserver::Level parseLevel(const std::string& level)
{
    if (level == "steps")
    {
        return MissionObserver::Level::kSteps;
    }
    if (level == "all")
    {
        return MissionObserver::Level::kAll;
    }
    throw std::invalid_argument("event_level must be 'steps' or 'all', got '" + level + "'");
}

/// The registration IDs of the leaves this package provides, which are the ones that do things.
std::set<std::string> skillLeavesOf(const BT::BehaviorTreeFactory& factory)
{
    std::set<std::string> leaves;
    for (const auto& [id, manifest] : factory.manifests())
    {
        if (!factory.builtinNodes().contains(id))
        {
            leaves.insert(id);
        }
    }
    return leaves;
}

/// A step as the tree holds it: its SubTree node, and the outermost node that is the step's
/// alone, the decorators wrapped around it up to the Sequence or Fallback that holds it.
struct StepNodes
{
    BT::TreeNode*    top     = nullptr;
    BT::SubTreeNode* subtree = nullptr;
};

/// Step @p name under @p node, which is @p top's or the top of a step itself; with no name, the
/// step that runs.
// The mission tree's depth is capped by the validator, so the recursion is bounded.
// NOLINTNEXTLINE(misc-no-recursion)
StepNodes findStep(BT::TreeNode* node, BT::TreeNode* top, std::string_view name)
{
    if (node == nullptr)
    {
        return {};
    }
    if (auto* subtree = dynamic_cast<BT::SubTreeNode*>(node))
    {
        const bool wanted =
            name.empty() ? subtree->status() == BT::NodeStatus::RUNNING : subtree->name() == name;
        return wanted ? StepNodes{ top, subtree } : StepNodes{};
    }
    if (auto* control = dynamic_cast<BT::ControlNode*>(node))
    {
        for (BT::TreeNode* child : control->children())
        {
            if (StepNodes found = findStep(child, child, name); found.subtree != nullptr)
            {
                return found;
            }
        }
        return {};
    }
    if (auto* decorator = dynamic_cast<BT::DecoratorNode*>(node))
    {
        return findStep(decorator->child(), top, name);
    }
    return {};
}

/// What the leaves of @p step begin their paths with, which the pause gate goes by: its name, or
/// for a SubTree without one, the instance name BT.CPP gives it.
std::string pathOf(const BT::TreeNode& step)
{
    std::string prefix;
    BT::applyRecursiveVisitor(&step, [&prefix](const BT::TreeNode* node) {
        const std::string& path = node->fullPath();
        if (const auto slash = path.find('/'); prefix.empty() && slash != std::string::npos)
        {
            prefix = path.substr(0, slash);
        }
    });
    return prefix.empty() ? step.name() : prefix;
}

/// The cause a rejection's code names: NONE for a problem of the plan's own.
std::uint8_t causeOf(std::string_view code)
{
    using Result = nervros_interfaces::action::ExecuteMission::Result;
    if (code == code::kRobotCannotMove)
    {
        return Result::CAUSE_ROBOT;
    }
    if (code == code::kShuttingDown)
    {
        return Result::CAUSE_SHUTDOWN;
    }
    if (code == code::kTeleopActive)
    {
        return Result::CAUSE_TELEOP;
    }
    if (code == code::kBusy)
    {
        return Result::CAUSE_BUSY;
    }
    return Result::CAUSE_NONE;
}

}  // namespace

MissionExecutor::MissionExecutor(
    std::shared_ptr<AuthorityPort> authority, const rclcpp::NodeOptions& options)
  : rclcpp::Node("nervros_executor", options)
  , authority_(std::move(authority))
{
    tick_rate_hz_ = positive(declare_parameter<double>("tick_rate_hz", 10.0), "tick_rate_hz");
    max_duration_cap_s_ =
        positive(declare_parameter<double>("max_duration_cap_s", 1800.0), "max_duration_cap_s");
    groot2_port_           = static_cast<int>(declare_parameter<int>("groot2_port", 0));
    hands_empty_on_attach_ = declare_parameter<bool>("hands_empty_on_attach", false);
    duration_factor_ =
        positive(declare_parameter<double>("watchdog_factor", 1.2), "watchdog_factor");
    dry_run_time_scale_ =
        positive(declare_parameter<double>("dry_run_time_scale", 0.02), "dry_run_time_scale");
    dry_run_min_leaf_s_ =
        positive(declare_parameter<double>("dry_run_min_leaf_s", 0.05), "dry_run_min_leaf_s");
    feedback_period_s_ =
        positive(declare_parameter<double>("feedback_period_s", 0.25), "feedback_period_s");
    cancel_settle_s_ =
        positive(declare_parameter<double>("cancel_settle_s", 0.5), "cancel_settle_s");
    stop_wait_s_ = positive(declare_parameter<double>("stop_wait_s", 3.0), "stop_wait_s");
    max_pause_s_ = positive(declare_parameter<double>("max_pause_s", 600.0), "max_pause_s");
    event_level_ = parseLevel(declare_parameter<std::string>("event_level", "steps"));
    const double authority_timeout_s =
        positive(declare_parameter<double>("authority_timeout_s", 15.0), "authority_timeout_s");
    ControllerManagerAuthority::Fingers fingers;
    fingers.joints = declare_parameter<std::vector<std::string>>(
        "finger_joints",
        { "left_hand_index_0_joint",
          "left_hand_middle_0_joint",
          "right_hand_index_0_joint",
          "right_hand_middle_0_joint" });
    fingers.closed_rad =
        positive(declare_parameter<double>("finger_closed_rad", 0.3), "finger_closed_rad");
    fingers.timeout_s =
        positive(declare_parameter<double>("finger_read_timeout_s", 2.0), "finger_read_timeout_s");

    Limits limits;
    limits.max_depth = static_cast<int>(declare_parameter<int>("max_tree_depth", limits.max_depth));
    limits.max_nodes = static_cast<int>(declare_parameter<int>("max_tree_nodes", limits.max_nodes));
    limits.max_steps = static_cast<int>(declare_parameter<int>("max_tree_steps", limits.max_steps));
    limits.max_bytes = static_cast<std::size_t>(
        declare_parameter<int>("max_xml_bytes", static_cast<int>(limits.max_bytes)));
    limits.max_duration_s = max_duration_cap_s_;

    const std::string catalog_file =
        declare_parameter<std::string>("catalog_file", "config/catalog.yaml");
    const std::string macros_dir = declare_parameter<std::string>("macros_dir", "trees/library");
    Catalog           catalog    = Catalog::load(resolve(catalog_file));
    MacroLibrary      library    = MacroLibrary::load(resolve(macros_dir));
    // `Skill.arg=a|b;...`: here, the only names the simulator's mock detector finds.
    const std::string choices = declare_parameter<std::string>("arg_choices", "");
    for (const auto choice : std::views::split(std::string_view(choices), ';'))
    {
        if (!std::ranges::empty(choice))
        {
            catalog.restrict(std::string_view(std::ranges::data(choice), std::ranges::size(choice)));
        }
    }

    // The palette a planner or Groot2 may import: the leaves, then the skills built from them. The
    // catalog's version covers it, so a leaf whose ports change is a different catalog.
    std::string leaf_palette;
    NameSet     node_ids;
    {
        BT::BehaviorTreeFactory scratch;
        registerSkillNodes(scratch, RosContext{});
        leaf_palette = nodeModelXml(scratch);
        for (const auto& [id, _] : scratch.manifests())
        {
            node_ids.insert(id);
        }
    }
    Learnable learnable = Learnable::parse(
        declare_parameter<std::vector<std::string>>("learnable_leaves", std::vector<std::string>{}),
        leaf_palette);
    learnable.max_duration_s = positive(
        declare_parameter<double>("max_learned_duration_s", learnable.max_duration_s),
        "max_learned_duration_s");
    learnable.max_nodes =
        static_cast<int>(declare_parameter<int>("max_learned_nodes", learnable.max_nodes));
    const auto max_learned = static_cast<std::size_t>(
        std::max<std::int64_t>(0, declare_parameter<int>("max_learned_skills", 32)));
    const std::string learned_dir = declare_parameter<std::string>("learned_skills_dir", "");
    repertoire_                   = std::make_unique<Repertoire>(
        std::move(catalog),
        std::move(library),
        std::move(leaf_palette),
        std::move(node_ids),
        limits,
        LearnedSkills(learned_dir, std::move(learnable), max_learned));
    for (const std::string& skipped : repertoire_->load())
    {
        RCLCPP_WARN(get_logger(), "a learned skill was left out: %s", skipped.c_str());
    }

    if (!authority_)
    {
        authority_ = std::make_shared<ControllerManagerAuthority>(
            get_logger(),
            authority_timeout_s,
            std::move(fingers));
    }

    action_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    stop_group_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    query_group_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_group_  = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    status_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    skills_node_ = std::make_shared<rclcpp::Node>("nervros_executor_skills");
    skills_executor_.add_node(skills_node_);

    const auto stop_servers = declare_parameter<std::vector<std::string>>(
        "stop_action_servers",
        std::vector<std::string>{});
    const auto agent_servers = declare_parameter<std::vector<std::string>>(
        "agent_action_servers",
        std::vector<std::string>{});
    const std::string arm_action = declare_parameter<std::string>("arm_controller_action", "");
    if (stop_servers.empty() || arm_action.empty())
    {
        RCLCPP_WARN(
            get_logger(),
            "stop_action_servers or arm_controller_action is empty: StopAll will only cancel what "
            "this executor's own mission sent, and will not hold the arm");
    }
    // The agent's own motions are stopped with the mission's: StopAll means every motion.
    std::vector<std::string> stopped = stop_servers;
    stopped.insert(stopped.end(), agent_servers.begin(), agent_servers.end());
    leaf_servers_   = std::make_unique<ServerFleet>(*this, stopped, status_group_);
    arm_controller_ = std::make_unique<ServerFleet>(
        *this,
        arm_action.empty() ? std::vector<std::string>{} : std::vector<std::string>{ arm_action },
        status_group_);

    state_pub_ =
        create_publisher<RobotState>("~/robot_state", rclcpp::QoS(1).reliable().transient_local());
    state_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this] {
            watchdog();
            pushState();
        },
        state_group_);

    // Declared whether or not the gate is on, so one config file serves both.
    RobotHealth::Params health;
    health_gate_     = declare_parameter<bool>("health_gate", true);
    health.imu_topic = declare_parameter<std::string>("imu_topic", "/imu_sensor_broadcaster/imu");
    health.imu_timeout_s =
        positive(declare_parameter<double>("imu_timeout_s", 1.0), "imu_timeout_s");
    health.max_tilt_deg = positive(declare_parameter<double>("max_tilt_deg", 45.0), "max_tilt_deg");
    health.walk_controllers = declare_parameter<std::vector<std::string>>(
        "walk_controllers",
        std::vector<std::string>{ "agile_controller" });
    health.stand_controllers = declare_parameter<std::vector<std::string>>(
        "stand_controllers",
        std::vector<std::string>{ "locomotion_freeze_controller" });
    health.controller_manager =
        declare_parameter<std::string>("controller_manager", "/controller_manager");
    health.poll_s = positive(declare_parameter<double>("health_poll_s", 1.0), "health_poll_s");
    if (health_gate_)
    {
        health_ = std::make_unique<RobotHealth>(*this, std::move(health), state_group_);
    }

    preview_group_         = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    preview_clients_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    MissionPreview::Params preview;
    preview.planner_action =
        declare_parameter<std::string>("planner_action", "/compute_path_to_pose");
    preview.approach_service =
        declare_parameter<std::string>("approach_service", "/canopy/get_approach_pose");
    preview.clear_costmap_service = declare_parameter<std::string>(
        "clear_costmap_service",
        "/global_costmap/clear_entirely_global_costmap");
    preview.clear_wait_s = positive(declare_parameter<double>("clear_wait_s", 1.5), "clear_wait_s");
    preview.map_frame    = declare_parameter<std::string>("map_frame", "map");
    preview.base_frame   = declare_parameter<std::string>("base_frame", "base_footprint");
    preview.timeout_s =
        positive(declare_parameter<double>("preview_timeout_s", 5.0), "preview_timeout_s");
    preview_ = std::make_unique<MissionPreview>(*this, std::move(preview), preview_clients_group_);

    TeleopDriver::Params teleop;
    teleop.command_topic = declare_parameter<std::string>("teleop_command_topic", "/cmd_vel_raw");
    teleop.max_x         = positive(declare_parameter<double>("teleop_max_x", 0.5), "teleop_max_x");
    teleop.max_y         = positive(declare_parameter<double>("teleop_max_y", 0.3), "teleop_max_y");
    teleop.max_yaw = positive(declare_parameter<double>("teleop_max_yaw", 0.8), "teleop_max_yaw");
    teleop.accel_xy =
        positive(declare_parameter<double>("teleop_accel_xy", 1.0), "teleop_accel_xy");
    teleop.accel_yaw =
        positive(declare_parameter<double>("teleop_accel_yaw", 2.0), "teleop_accel_yaw");
    teleop.deadman_s =
        positive(declare_parameter<double>("teleop_deadman_s", 0.4), "teleop_deadman_s");
    teleop.idle_s  = positive(declare_parameter<double>("teleop_idle_s", 20.0), "teleop_idle_s");
    teleop.rate_hz = positive(declare_parameter<double>("teleop_rate_hz", 20.0), "teleop_rate_hz");
    teleop_        = std::make_unique<TeleopDriver>(*this, std::move(teleop), state_group_, [this] {
        pushState();
    });

    rclcpp::SubscriptionOptions heartbeat_options;
    heartbeat_options.callback_group = state_group_;
    // Best effort: a late heartbeat is of no use, and the timeout spans several of them.
    heartbeat_sub_ = create_subscription<nervros_interfaces::msg::Heartbeat>(
        "~/heartbeat",
        rclcpp::QoS(1).best_effort(),
        [this](const nervros_interfaces::msg::Heartbeat::ConstSharedPtr& beat) {
            const std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            if (heartbeat_client_.empty() || beat->client == heartbeat_client_)
            {
                heartbeat_ticks_ = Clock::now().time_since_epoch().count();
            }
        },
        heartbeat_options);
    const double deadman_hz =
        positive(declare_parameter<double>("deadman_check_hz", 10.0), "deadman_check_hz");
    max_heartbeat_timeout_s_ = positive(
        declare_parameter<double>("max_heartbeat_timeout_s", 10.0),
        "max_heartbeat_timeout_s");
    health_trip_s_ = positive(declare_parameter<double>("health_trip_s", 0.5), "health_trip_s");
    max_lend_s_    = positive(declare_parameter<double>("max_lend_s", 120.0), "max_lend_s");
    lend_heartbeat_timeout_s_ = positive(
        declare_parameter<double>("lend_heartbeat_timeout_s", 2.0),
        "lend_heartbeat_timeout_s");
    deadman_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / deadman_hz),
        [this] { deadman(); },
        state_group_);

    action_server_ = rclcpp_action::create_server<ExecuteMission>(
        this,
        "~/execute_mission",
        [](const rclcpp_action::GoalUUID&                     uuid,
           const std::shared_ptr<const ExecuteMission::Goal>& goal) {
            return handleGoal(uuid, goal);
        },
        [this](const std::shared_ptr<GoalHandle>& handle) { return handleCancel(handle); },
        [this](const std::shared_ptr<GoalHandle>& handle) { handleAccepted(handle); },
        rcl_action_server_get_default_options(),
        action_group_);
    using ValidateSrv = nervros_interfaces::srv::ValidateMission;
    using CatalogSrv  = nervros_interfaces::srv::GetCatalog;
    using StopSrv     = nervros_interfaces::srv::StopAll;
    validate_service_ = create_service<ValidateSrv>(
        "~/validate_mission",
        [this](
            const ValidateSrv::Request::SharedPtr&  request,
            const ValidateSrv::Response::SharedPtr& response) { onValidate(request, response); },
        rclcpp::ServicesQoS(),
        query_group_);
    catalog_service_ = create_service<CatalogSrv>(
        "~/get_catalog",
        [this](
            const CatalogSrv::Request::SharedPtr&  request,
            const CatalogSrv::Response::SharedPtr& response) { onCatalog(request, response); },
        rclcpp::ServicesQoS(),
        query_group_);
    stop_service_ = create_service<StopSrv>(
        "~/stop_all",
        [this](
            const StopSrv::Request::SharedPtr&  request,
            const StopSrv::Response::SharedPtr& response) { onStopAll(request, response); },
        rclcpp::ServicesQoS(),
        stop_group_);
    using PreviewSrv = nervros_interfaces::srv::PreviewMission;
    using TeleopSrv  = nervros_interfaces::srv::Teleop;
    preview_service_ = create_service<PreviewSrv>(
        "~/preview_mission",
        [this](
            const PreviewSrv::Request::SharedPtr&  request,
            const PreviewSrv::Response::SharedPtr& response) { onPreview(request, response); },
        rclcpp::ServicesQoS(),
        preview_group_);
    // With StopAll, so that a stop and a request to drive are taken one at a time.
    teleop_service_ = create_service<TeleopSrv>(
        "~/teleop",
        [this](
            const TeleopSrv::Request::SharedPtr&  request,
            const TeleopSrv::Response::SharedPtr& response) { onTeleop(request, response); },
        rclcpp::ServicesQoS(),
        stop_group_);

    using PauseSrv = nervros_interfaces::srv::PauseMission;
    // With StopAll, so a pause and a stop are taken one at a time.
    pause_service_ = create_service<PauseSrv>(
        "~/pause_mission",
        [this](
            const PauseSrv::Request::SharedPtr&  request,
            const PauseSrv::Response::SharedPtr& response) { onPause(request, response); },
        rclcpp::ServicesQoS(),
        stop_group_);
    define_skill_service_ = create_service<nervros_interfaces::srv::DefineSkill>(
        "~/define_skill",
        [this](
            const nervros_interfaces::srv::DefineSkill::Request::SharedPtr&  request,
            const nervros_interfaces::srv::DefineSkill::Response::SharedPtr& response) {
            onDefineSkill(request, response);
        },
        rclcpp::ServicesQoS(),
        query_group_);
    forget_skill_service_ = create_service<nervros_interfaces::srv::ForgetSkill>(
        "~/forget_skill",
        [this](
            const nervros_interfaces::srv::ForgetSkill::Request::SharedPtr&  request,
            const nervros_interfaces::srv::ForgetSkill::Response::SharedPtr& response) {
            onForgetSkill(request, response);
        },
        rclcpp::ServicesQoS(),
        query_group_);

    lend_group_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lend_service_ = create_service<nervros_interfaces::srv::LendResources>(
        "~/lend_resources",
        [this](
            const nervros_interfaces::srv::LendResources::Request::SharedPtr&  request,
            const nervros_interfaces::srv::LendResources::Response::SharedPtr& response) {
            onLend(request, response);
        },
        rclcpp::ServicesQoS(),
        lend_group_);

    pushState();
    RCLCPP_INFO(
        get_logger(),
        "ready: %zu skills (catalog %s), stop covers %zu action servers",
        repertoire_->current()->validator.catalog().skills().size(),
        repertoire_->current()->validator.catalog().version().c_str(),
        leaf_servers_->names().size());
}

MissionExecutor::~MissionExecutor() { shutting_down_ = true; }

double
MissionExecutor::watchdogLimitS(double requested_s, double worst_case_s, double factor, double cap_s)
{
    double limit_s = requested_s;
    if (!std::isfinite(limit_s) || limit_s <= 0.0)
    {
        limit_s = factor * worst_case_s;
    }
    return limit_s > 0.0 ? std::min(limit_s, cap_s) : cap_s;
}

void MissionExecutor::shutdown()
{
    {
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        shutting_down_ = true;
        halting_       = true;
    }
    {
        const std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_.joinable())
        {
            worker_.join();
        }
    }
    if (releaser_.joinable())
    {
        releaser_.join();
    }
    if (letGoOfArms())
    {
        RCLCPP_WARN(
            get_logger(),
            "shutting down with the arms acquired: a hand holds something, or may. Release them "
            "with deactivate_arm once it is put down.");
    }
}

// --- ROS surface -------------------------------------------------------------------------------

rclcpp_action::GoalResponse MissionExecutor::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    const std::shared_ptr<const ExecuteMission::Goal>& /*goal*/)
{
    // Every goal is accepted, so the client gets a result that says why when it is turned away:
    // a refusal at this stage cannot carry a reason.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MissionExecutor::handleCancel(const std::shared_ptr<GoalHandle>& handle)
{
    // The leaves are told to wind down now, since the tick loop may be inside a long tick; it sees
    // the cancel itself the moment the tick returns.
    const std::lock_guard<std::mutex> lock(stop_mutex_);
    if (running_ && handle->get_goal_id() == goal_id_)
    {
        halting_ = true;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionExecutor::handleAccepted(const std::shared_ptr<GoalHandle>& handle)
{
    std::string what;
    try
    {
        startMission(handle);
        return;
    }
    catch (const std::exception& e)
    {
        what = e.what();
    }
    catch (...)
    {
        what = "unknown error";
    }
    RCLCPP_ERROR(get_logger(), "could not start a mission: %s", what.c_str());
    try
    {
        auto result            = std::make_shared<ExecuteMission::Result>();
        result->outcome        = ExecuteMission::Result::OUTCOME_ERROR;
        result->failure_reason = "the executor could not start the mission: " + what;
        handle->abort(result);
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(), "and could not tell the client so");
    }
}

void MissionExecutor::startMission(const std::shared_ptr<GoalHandle>& handle)
{
    const std::lock_guard<std::mutex> worker_lock(worker_mutex_);
    bool                              refused_shutting_down = false;
    bool                              refused_busy          = false;
    bool                              refused_teleop        = false;
    std::string                       lent_to;
    {
        // The check and the claim are one step, and the stop flags are cleared with the claim: a
        // stop that comes later is for this mission, and one that came earlier was not.
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        if (shutting_down_)
        {
            refused_shutting_down = true;
        }
        else if (running_)
        {
            refused_busy = true;
        }
        else if (teleop_->active() && handle->get_goal()->mode == ExecuteMission::Goal::MODE_EXECUTE)
        {
            refused_teleop = true;
        }
        else if (!lent_to_.empty() && handle->get_goal()->mode == ExecuteMission::Goal::MODE_EXECUTE)
        {
            lent_to = lent_to_;
        }
        else
        {
            running_        = true;
            goal_id_        = handle->get_goal_id();
            stop_requested_ = false;
            halting_        = false;
            pause_wanted_   = false;
        }
    }
    if (refused_shutting_down)
    {
        rejectGoal(
            handle,
            { code::kShuttingDown,
              0,
              "",
              "The executor is shutting down and takes no more missions." },
            "the executor is shutting down");
        return;
    }
    if (!lent_to.empty())
    {
        rejectGoal(
            handle,
            { code::kBusy,
              0,
              "",
              "The arms are lent to " + lent_to +
                  " for a motion of its own; it gives them back when it is done." },
            "the arms are lent to " + lent_to);
        return;
    }
    if (refused_teleop)
    {
        rejectGoal(
            handle,
            { code::kTeleopActive,
              0,
              "",
              "The base is being driven by hand. End teleop before sending a mission." },
            "teleop is driving the base");
        return;
    }
    if (refused_busy)
    {
        std::string current;
        {
            const std::lock_guard<std::mutex> lock(live_mutex_);
            current = mission_id_;
        }
        rejectGoal(
            handle,
            { code::kBusy,
              0,
              current,
              "A mission is already running. Wait for it to finish, or stop it, before sending "
              "another." },
            "a mission is already running (" + current + "); the executor runs one at a time");
        return;
    }
    try
    {
        {
            const std::lock_guard<std::mutex> lock(halted_mutex_);
            halted_ = false;
        }
        if (worker_.joinable())
        {
            worker_.join();
        }
        worker_ = std::jthread([this, handle] { run(handle); });
    }
    catch (...)
    {
        {
            const std::lock_guard<std::mutex> lock(halted_mutex_);
            halted_ = true;
        }
        halted_cv_.notify_all();
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        running_ = false;
        throw;
    }
}

void MissionExecutor::rejectGoal(
    const std::shared_ptr<GoalHandle>& handle, const Diagnostic& diagnostic,
    const std::string& reason)
{
    auto result              = std::make_shared<ExecuteMission::Result>();
    result->outcome          = ExecuteMission::Result::OUTCOME_REJECTED;
    result->failure_reason   = reason;
    result->diagnostics_json = diagnosticsJson({ diagnostic });
    result->cause            = causeOf(diagnostic.code);
    RCLCPP_WARN(
        get_logger(),
        "rejecting goal %s: %s",
        handle->get_goal()->mission_id.c_str(),
        reason.c_str());
    handle->abort(result);
}

void MissionExecutor::onValidate(
    const std::shared_ptr<nervros_interfaces::srv::ValidateMission::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::ValidateMission::Response>& response)
{
    // Nothing may escape a service callback: it would end the executor's thread and the process.
    try
    {
        const auto skills     = repertoire_->current();
        Validation validation = skills->validator.validate(request->tree_xml);
        if (validation.ok())
        {
            BT::BehaviorTreeFactory factory =
                makeFactory(*skills, /*dry_run=*/true, validation, validation.macros);
            BT::Tree tree;
            if (auto problem =
                    load(*skills, factory, request->tree_xml, BT::Blackboard::create(), tree))
            {
                validation.diagnostics.push_back(std::move(*problem));
            }
        }
        response->ok                    = validation.ok();
        response->diagnostics_json      = diagnosticsJson(validation.diagnostics);
        response->tree_sha256           = validation.sha256;
        response->worst_case_duration_s = static_cast<float>(validation.worst_case_s);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "could not check a mission: %s", e.what());
        response->ok                    = false;
        response->diagnostics_json      = diagnosticsJson({ { code::kLoadError,
                                                              0,
                                                              "",
                                                              std::string("The robot could not "
                                                                          "check the mission: ") +
                                                                  e.what() } });
        response->worst_case_duration_s = 0.0F;
    }
}

void MissionExecutor::onCatalog(
    const std::shared_ptr<nervros_interfaces::srv::GetCatalog::Request>& /*request*/,
    const std::shared_ptr<nervros_interfaces::srv::GetCatalog::Response>& response)
{
    const auto skills              = repertoire_->current();
    response->catalog_json         = skills->catalog_json;
    response->tree_nodes_model_xml = skills->palette_xml;
    response->bt_cpp_version       = BT::LibraryVersionString();
    response->catalog_version      = skills->validator.catalog().version();
}

void MissionExecutor::onDefineSkill(
    const std::shared_ptr<nervros_interfaces::srv::DefineSkill::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::DefineSkill::Response>& response)
{
    // Nothing may escape a service callback: it would end the executor's thread and the process.
    try
    {
        nlohmann::json definition = {
            { "name", request->name },
            { "description", request->description },
            { "args",
              nlohmann::json::parse(request->args_json.empty() ? "[]" : request->args_json) },
            { "body_xml", request->body_xml },
            { "max_duration_s", request->max_duration_s },
            { "taught_for", request->taught_for },
            { "taught_s",
              std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
                  .count() },
        };
        const auto problems =
            repertoire_->define(SkillDefinition::fromJson(definition.dump()), request->keep);
        response->ok               = problems.empty();
        response->diagnostics_json = diagnosticsJson(problems);
        response->message =
            problems.empty() ?
                std::format("{} is {}", request->name, request->keep ? "kept" : "a draft") :
                problems.front().message;
        if (problems.empty())
        {
            RCLCPP_INFO(
                get_logger(),
                "learned skill %s: %s",
                request->name.c_str(),
                request->keep ? "kept" : "draft");
        }
    }
    catch (const std::exception& e)
    {
        response->ok      = false;
        response->message = std::string("the skill could not be read: ") + e.what();
        response->diagnostics_json =
            diagnosticsJson({ { code::kBadStructure, 0, request->name, response->message } });
    }
    response->catalog_version = repertoire_->current()->validator.catalog().version();
}

void MissionExecutor::onForgetSkill(
    const std::shared_ptr<nervros_interfaces::srv::ForgetSkill::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::ForgetSkill::Response>& response)
{
    try
    {
        response->ok      = repertoire_->forget(request->name);
        response->message = response->ok ? request->name + " is forgotten" :
                                           "no learned skill is called " + request->name;
    }
    catch (const std::exception& e)
    {
        response->ok      = false;
        response->message = std::string("the skill could not be forgotten: ") + e.what();
    }
    response->catalog_version = repertoire_->current()->validator.catalog().version();
}

void MissionExecutor::onStopAll(
    const std::shared_ptr<nervros_interfaces::srv::StopAll::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::StopAll::Response>& response)
{
    *response = stopEverything(request->reason, ExecuteMission::Result::CAUSE_NONE);
}

nervros_interfaces::srv::StopAll::Response
MissionExecutor::stopEverything(const std::string& reason, std::uint8_t cause)
{
    nervros_interfaces::srv::StopAll::Response response;
    // Nothing may escape: this runs in a service callback and in a timer. A stop that fails part
    // way still says what it knows.
    try
    {
        const auto deadline  = Clock::now() + inClockUnits(stop_wait_s_);
        const auto remaining = [&deadline] {
            return Seconds(std::max(0.0, Seconds(deadline - Clock::now()).count()));
        };
        RCLCPP_WARN(get_logger(), "stop requested: %s", reason.c_str());
        teleop_->disable("stopped: " + reason);
        {
            const std::lock_guard<std::mutex> lock(live_mutex_);
            note_ = "stopping";
        }
        pushState();

        // (1) The tree halts on its own thread, which cancels its leaves' goals. Whether a mission
        // runs, and the flags for it, are one step: a stop cannot land on the next mission.
        bool mission = false;
        {
            const std::lock_guard<std::mutex> lock(stop_mutex_);
            mission = running_;
            // The first stop says why: a later one, as the operator's after a deadman, does not
            // rename it.
            if (mission && !stop_requested_)
            {
                stop_reason_ = reason;
                stop_cause_  = cause;
            }
            if (mission)
            {
                stop_requested_ = true;
                halting_        = true;
            }
        }
        // (2) Every goal on every server the leaves use, whoever sent it. And the arm's trajectory
        // controller, which holds the arm where it is when its goal is cancelled: no snap, and the
        // arm stops now rather than at the end of the manipulation skill's phase. The hand
        // controllers are left alone: cancelling their goal would let go of the grip.
        std::size_t leaf_asked = leaf_servers_->cancelAll();
        std::size_t arm_asked  = arm_controller_->cancelAll();

        // (3) Wait for the tree to be halted, ask again for any goal accepted since, and wait for
        // the servers to go quiet.
        bool halted = true;
        if (mission)
        {
            std::unique_lock<std::mutex> lock(halted_mutex_);
            halted = halted_cv_.wait_until(lock, deadline, [this] { return halted_; });
        }
        leaf_asked                    = std::max(leaf_asked, leaf_servers_->cancelAll());
        arm_asked                     = std::max(arm_asked, arm_controller_->cancelAll());
        std::vector<std::string> busy = leaf_servers_->settle(remaining());
        for (std::string& server : arm_controller_->settle(remaining()))
        {
            busy.push_back(std::move(server));
        }

        // A lend ends with the stop: what was lent is let go of as at a mission's end.
        (void)takeBack("stopped: " + reason);

        // (4) to (6) Nothing else moves the base or the arm now; the hands were never touched.
        std::string hands;
        {
            const std::lock_guard<std::mutex> lock(live_mutex_);
            stopped_ = true;
            note_.clear();
            hands = describeHandsLocked("keeps");
        }
        pushState();

        response.ok = halted && busy.empty();
        response.state_after =
            "holding posture" + (hands.empty() ? std::string("; hands empty") : hands);
        response.message = describeStop(mission, halted, leaf_asked, arm_asked, busy);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "the stop failed part way: %s", e.what());
        response.ok          = false;
        response.message     = std::string("the stop failed part way: ") + e.what();
        response.state_after = "unknown";
    }
    return response;
}

void MissionExecutor::deadman()
{
    checkLend();
    if (!running_ || deadman_fired_)
    {
        return;
    }
    const Clock::rep now     = Clock::now().time_since_epoch().count();
    const Clock::rep timeout = heartbeat_timeout_ticks_.load();
    const Clock::rep silent  = now - heartbeat_ticks_.load();
    std::string      reason;
    std::uint8_t     cause = ExecuteMission::Result::CAUSE_DEADMAN;
    if (timeout != 0 && silent >= timeout)
    {
        reason = std::format(
            "deadman: no heartbeat from the agent for {:.1f} s",
            Seconds(Clock::duration(silent)).count());
    }
    else if (health_ && mission_walks_)
    {
        // A fall mid-walk, or the safety controller handing the legs to a freeze. A blip in a
        // reading does not stop a mission; one that lasts does.
        const Health health = health_->now();
        if (health.can_walk)
        {
            unhealthy_since_ = 0;
            return;
        }
        Clock::rep since = 0;
        if (unhealthy_since_.compare_exchange_strong(since, now) ||
            now - since < inClockUnits(health_trip_s_).count())
        {
            return;
        }
        reason = "the robot can no longer walk: " + health.reason;
        cause  = ExecuteMission::Result::CAUSE_ROBOT;
    }
    if (reason.empty() || deadman_fired_.exchange(true))
    {
        return;
    }
    RCLCPP_ERROR(get_logger(), "%s; stopping the mission", reason.c_str());
    stopEverything(reason, cause);
}

void MissionExecutor::onLend(
    const std::shared_ptr<nervros_interfaces::srv::LendResources::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::LendResources::Response>& response)
{
    const std::string& client = request->client;
    if (!request->lend)
    {
        response->granted = true;
        response->reason  = takeBack(client + " gave them back") ? "" : "nothing was lent";
        return;
    }
    const auto refuse = [&response](const std::string& why) {
        response->granted = false;
        response->reason  = why;
    };
    if (client.empty())
    {
        refuse("say who borrows them");
        return;
    }
    const std::set<std::string> wanted(request->resources.begin(), request->resources.end());
    const bool arms = wanted.contains(kLeftArmResource) || wanted.contains(kRightArmResource);
    if (auto why = healthRefusal(wanted))
    {
        refuse(*why);
        return;
    }
    {
        // Claimed as a mission is, under the same lock: a lend and a mission exclude each other.
        std::unique_lock<std::mutex> lock(stop_mutex_);
        awaitRelease(lock);
        if (shutting_down_)
        {
            refuse("the executor is shutting down");
            return;
        }
        if (running_)
        {
            refuse("a mission is running");
            return;
        }
        if (teleop_->active())
        {
            refuse("the base is being driven by hand");
            return;
        }
        if (!lent_to_.empty() && lent_to_ != client)
        {
            refuse("they are lent to " + lent_to_);
            return;
        }
        lent_to_ = client;
    }
    {
        const std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        heartbeat_client_ = client;
    }
    heartbeat_ticks_     = Clock::now().time_since_epoch().count();
    const double limit_s = std::isfinite(request->max_s) && request->max_s > 0.0 ?
                               std::min(request->max_s, max_lend_s_) :
                               max_lend_s_;
    lent_until_          = (Clock::now() + inClockUnits(limit_s)).time_since_epoch().count();
    bool held            = false;
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        held = arms_held_;
    }
    if (arms && !held)
    {
        Acquired taken = Acquired::kFailed;
        try
        {
            taken = authority_->acquire();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "taking the arms to lend them threw: %s", e.what());
        }
        if (taken == Acquired::kFailed)
        {
            (void)takeBack("the arms could not be taken");
            refuse("the arms could not be taken; is the control stack up?");
            return;
        }
        const std::lock_guard<std::mutex> lock(live_mutex_);
        arms_held_ = true;
        if (taken == Acquired::kAlready && !hands_empty_on_attach_)
        {
            // As for a mission: someone else took them, so what the hands hold is not known.
            hands_.left.unknown  = hands_.left.id.empty();
            hands_.right.unknown = hands_.right.id.empty();
            syncBlackboardLocked();
        }
    }
    bool stopped_meanwhile = false;
    {
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        stopped_meanwhile = lent_to_ != client;
    }
    if (stopped_meanwhile)
    {
        // The stop let go of nothing: the arms were not yet held when it came.
        letGoOfArms();
        pushState();
        refuse("a stop came while the arms were being taken");
        return;
    }
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        note_    = "arms lent to " + client;
        stopped_ = false;
    }
    pushState();
    RCLCPP_INFO(get_logger(), "lent the arms to %s for up to %.0f s", client.c_str(), limit_s);
    response->granted = true;
}

bool MissionExecutor::takeBack(const std::string& why)
{
    std::string client;
    {
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        if (lent_to_.empty())
        {
            return false;
        }
        client     = std::exchange(lent_to_, std::string());
        releasing_ = true;
    }
    lent_until_ = 0;
    RCLCPP_INFO(get_logger(), "took back what %s borrowed: %s", client.c_str(), why.c_str());
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        note_.clear();
    }
    // Off the caller's thread, which may be StopAll's or the deadman's. No lend is granted while
    // releasing_ is set, so the last releaser has finished and the assignment does not wait.
    releaser_ = std::jthread([this] {
        (void)letGoOfArms();
        {
            const std::lock_guard<std::mutex> lock(stop_mutex_);
            releasing_ = false;
        }
        released_cv_.notify_all();
        pushState();
    });
    return true;
}

void MissionExecutor::awaitRelease(std::unique_lock<std::mutex>& lock)
{
    released_cv_.wait(lock, [this] { return !releasing_; });
}

bool MissionExecutor::letGoOfArms()
{
    const auto kept = [this] {
        RCLCPP_INFO(
            get_logger(),
            "keeping the arms: a hand holds something, or may%s",
            describeHandsLocked("holds").c_str());
        return true;
    };
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        if (!arms_held_)
        {
            return false;
        }
        if (mayHoldSomethingLocked())
        {
            return kept();
        }
    }
    // A hand closed by something the executor does not follow, a borrower's grip or a grasp
    // policy, may hold what it closed on, and a released hand drops it.
    const ClosedHands closed = authority_->closedHands();
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        hands_.left.unknown  = hands_.left.unknown || closed.left;
        hands_.right.unknown = hands_.right.unknown || closed.right;
        if (mayHoldSomethingLocked())
        {
            syncBlackboardLocked();
            return kept();
        }
    }
    authority_->release();
    const std::lock_guard<std::mutex> lock(live_mutex_);
    arms_held_ = false;
    return false;
}

void MissionExecutor::checkLend()
{
    const Clock::rep until = lent_until_.load();
    if (until == 0)
    {
        return;
    }
    const Clock::rep now    = Clock::now().time_since_epoch().count();
    const Clock::rep silent = now - heartbeat_ticks_.load();
    if (now >= until)
    {
        (void)takeBack("its time ran out");
    }
    else if (silent >= inClockUnits(lend_heartbeat_timeout_s_).count())
    {
        (void)takeBack(
            std::format("no heartbeat for {:.1f} s", Seconds(Clock::duration(silent)).count()));
    }
}

std::optional<std::string>
MissionExecutor::healthRefusal(const std::set<std::string>& resources) const
{
    if (!health_ || resources.empty())
    {
        return std::nullopt;
    }
    const Health health = health_->now();
    const bool   walks  = resources.contains(kBaseResource);
    if (walks ? health.can_walk : health.can_stand)
    {
        return std::nullopt;
    }
    return std::format(
        "The robot cannot {} now: {}. Check on it before sending a mission.",
        walks ? "walk" : "move its arms",
        health.reason);
}

void MissionExecutor::onPreview(
    const std::shared_ptr<nervros_interfaces::srv::PreviewMission::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::PreviewMission::Response>& response)
{
    try
    {
        const Validation validation = repertoire_->current()->validator.validate(request->tree_xml);
        if (!validation.ok())
        {
            response->ok = false;
            response->message =
                "the mission is not valid: " + validation.diagnostics.front().message;
            return;
        }
        std::string why;
        response->steps   = preview_->preview(missionSteps(request->tree_xml), why);
        response->ok      = why.empty();
        response->message = why;
    }
    catch (const std::exception& e)
    {
        response->ok      = false;
        response->message = std::string("the preview failed: ") + e.what();
    }
}

void MissionExecutor::onTeleop(
    const std::shared_ptr<nervros_interfaces::srv::Teleop::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::Teleop::Response>& response)
{
    if (!request->enable)
    {
        teleop_->disable("the operator ended it");
        pushState();
        response->ok      = true;
        response->message = "teleop off";
        return;
    }
    if (auto why = healthRefusal({ kBaseResource }))
    {
        response->ok      = false;
        response->message = *why;
        return;
    }
    {
        // Under the same lock a mission is claimed with, so the two cannot both start.
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        if (running_)
        {
            response->ok      = false;
            response->message = "a mission is running; wait for it or stop it first";
            return;
        }
        if (!lent_to_.empty())
        {
            response->ok = false;
            response->message =
                "the arms are lent to " + lent_to_ + "; wait for it or stop it first";
            return;
        }
        teleop_->enable();
    }
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        stopped_ = false;
    }
    pushState();
    response->ok      = true;
    response->message = "teleop on: geometry_msgs/Twist on ~/teleop_cmd drives the base; a pause "
                        "in them stops it";
}

void MissionExecutor::onPause(
    const std::shared_ptr<nervros_interfaces::srv::PauseMission::Request>&  request,
    const std::shared_ptr<nervros_interfaces::srv::PauseMission::Response>& response)
{
    // The mission thread does the halting between ticks; the claim and the flags are one step, as
    // for a stop, so a pause lands on the mission that was running or on none.
    const std::lock_guard<std::mutex> lock(stop_mutex_);
    if (!running_)
    {
        response->ok      = false;
        response->message = "no mission is running";
        return;
    }
    response->ok = true;
    if (request->pause == pause_wanted_.load())
    {
        response->message = request->pause ? "already paused, or pausing" : "not paused";
        return;
    }
    pause_wanted_ = request->pause;
    RCLCPP_INFO(
        get_logger(),
        "%s requested: %s",
        request->pause ? "pause" : "resume",
        request->reason.c_str());
    response->message = request->pause ? "pausing: the running step halts now, or once a pick or "
                                         "place it is in the middle of ends" :
                                         "resuming: the halted step starts again";
}

std::string MissionExecutor::describeStop(
    bool mission, bool halted, std::size_t leaf_servers_asked, std::size_t arm_asked,
    const std::vector<std::string>& busy) const
{
    std::string text = !mission ? "no mission was running" :
                       halted   ? "the mission was halted" :
                                  "the mission thread has not halted yet";
    text += leaf_servers_->size() == 0 ?
                "; no skill action servers are followed, so none was asked to cancel" :
                std::format(
                    "; cancel sent to {} of {} skill action servers",
                    leaf_servers_asked,
                    leaf_servers_->size());
    if (arm_controller_->size() == 0)
    {
        text += "; the arm controller is not followed, so the arm was not asked to hold";
    }
    else
    {
        text += arm_asked > 0 ?
                    "; the arm controller was asked to cancel its goal, which holds the arm where "
                    "it is" :
                    "; the arm controller has no action server up";
    }
    if (!busy.empty())
    {
        text += "; still winding down: " + join(busy);
    }
    return text;
}

// --- one mission -------------------------------------------------------------------------------

void MissionExecutor::run(const std::shared_ptr<GoalHandle>& handle)
{
    const auto                                        begin = Clock::now();
    const std::shared_ptr<const ExecuteMission::Goal> goal  = handle->get_goal();

    Outcome outcome;
    try
    {
        outcome = execute(*goal, handle);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "mission %s aborted: %s", goal->mission_id.c_str(), e.what());
        outcome.code   = ExecuteMission::Result::OUTCOME_ERROR;
        outcome.reason = e.what();
    }

    // The arms go back only when both hands are empty, and the state is idle before the result
    // is out, so a client that acts on the result sees the robot as it is.
    paused_ = false;
    pause_gate_.open();
    try
    {
        finish(goal->mode == ExecuteMission::Goal::MODE_EXECUTE);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "could not settle the state after a mission: %s", e.what());
    }

    auto result              = std::make_shared<ExecuteMission::Result>();
    result->outcome          = outcome.code;
    result->failed_node      = outcome.failed_node;
    result->failed_step_id   = outcome.failed_step;
    result->failure_reason   = outcome.reason;
    result->diagnostics_json = outcome.diagnostics;
    result->cause            = outcome.cause;
    result->outputs_json     = outcome.outputs;
    result->elapsed_s        = static_cast<float>(Seconds(Clock::now() - begin).count());
    RCLCPP_INFO(
        get_logger(),
        "mission %s finished: outcome %u in %.1f s%s%s",
        goal->mission_id.c_str(),
        static_cast<unsigned>(outcome.code),
        static_cast<double>(result->elapsed_s),
        outcome.reason.empty() ? "" : ": ",
        outcome.reason.c_str());

    {
        const std::lock_guard<std::mutex> lock(halted_mutex_);
        halted_ = true;
    }
    halted_cv_.notify_all();
    deadline_ticks_          = 0;
    heartbeat_timeout_ticks_ = 0;
    mission_walks_           = false;
    {
        // Before the result goes out, or a client that sends the next goal at once is told we are
        // busy. Under the mutex a stop lands on this mission or on none, and the flags do not
        // outlive it.
        const std::lock_guard<std::mutex> lock(stop_mutex_);
        stop_requested_ = false;
        halting_        = false;
        running_        = false;
        pause_wanted_   = false;
    }

    // This thread is not an executor's, so an exception here would end the process: the client of
    // a goal that cannot be answered any more has left, and nothing more is to be done for it.
    try
    {
        if (outcome.code == ExecuteMission::Result::OUTCOME_SUCCESS)
        {
            handle->succeed(result);
        }
        else if (outcome.code == ExecuteMission::Result::OUTCOME_CANCELED && handle->is_canceling())
        {
            handle->canceled(result);
        }
        else
        {
            handle->abort(result);
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "the result of mission %s could not be sent: %s",
            goal->mission_id.c_str(),
            e.what());
    }
}

MissionExecutor::Outcome MissionExecutor::rejected(const std::vector<Diagnostic>& diagnostics)
{
    Outcome outcome;
    outcome.code        = ExecuteMission::Result::OUTCOME_REJECTED;
    outcome.reason      = diagnostics.empty() ? "rejected" : diagnostics.front().message;
    outcome.diagnostics = diagnosticsJson(diagnostics);
    outcome.cause       = diagnostics.empty() ? outcome.cause : causeOf(diagnostics.front().code);
    return outcome;
}

BT::BehaviorTreeFactory MissionExecutor::makeFactory(
    const SkillSet& skills, bool dry_run, const Validation& validation,
    const std::set<std::string>& used)
{
    BT::BehaviorTreeFactory factory;
    if (!dry_run)
    {
        registerSkillNodes(factory, RosContext{ skills_node_, &halting_, &pause_gate_ });
    }
    else
    {
        // Each leaf becomes a TestNode that only waits, for a share of its skill's catalog time,
        // so a dry run walks the tree's shape and timing and moves nothing. Registered under the
        // leaves' own IDs rather than as substitution rules, which match by name and path and
        // would also replace a control node a planner happened to name like a leaf.
        BT::BehaviorTreeFactory scratch;
        registerSkillNodes(scratch, RosContext{});
        const std::set<std::string> leaves = skillLeavesOf(scratch);

        auto delays =
            std::make_shared<std::map<std::string, std::shared_ptr<BT::TestNodeConfig>>>();
        for (const auto& [step, macro_id] : validation.steps)
        {
            const Macro* macro = skills.validator.library().find(macro_id);
            const Skill* skill = skills.validator.catalog().findByMacro(macro_id);
            if (macro == nullptr || skill == nullptr)
            {
                continue;
            }
            const auto count = std::max<std::ptrdiff_t>(
                1,
                std::ranges::count_if(macro->tags, [&leaves](const std::string& tag) {
                    return leaves.contains(tag);
                }));
            const double each_s = std::max(
                dry_run_min_leaf_s_,
                skill->max_duration_s * dry_run_time_scale_ / static_cast<double>(count));
            auto config         = std::make_shared<BT::TestNodeConfig>();
            config->async_delay = std::chrono::milliseconds(std::llround(each_s * 1000.0));
            (*delays)[step]     = std::move(config);
        }
        auto fallback = std::make_shared<BT::TestNodeConfig>();
        fallback->async_delay =
            std::chrono::milliseconds(std::llround(dry_run_min_leaf_s_ * 1000.0));

        for (const std::string& id : leaves)
        {
            factory.registerBuilder(
                scratch.manifests().at(id),
                [delays, fallback](const std::string& name, const BT::NodeConfig& config) {
                    const std::string step  = config.path.substr(0, config.path.find('/'));
                    const auto        found = delays->find(step);
                    return std::make_unique<BT::TestNode>(
                        name,
                        config,
                        found != delays->end() ? found->second : fallback);
                });
        }
    }
    for (const std::string& id : used)
    {
        factory.registerBehaviorTreeFromText(skills.validator.library().find(id)->xml);
    }
    return factory;
}

BT::Blackboard::Ptr MissionExecutor::seededBlackboard()
{
    auto                              blackboard = BT::Blackboard::create();
    const std::lock_guard<std::mutex> lock(live_mutex_);
    const Hands&                      hands = handsInUseLocked();
    blackboard->set<std::string>("held_left", entryOf(hands.left));
    blackboard->set<std::string>("held_right", entryOf(hands.right));
    return blackboard;
}

std::optional<Diagnostic> MissionExecutor::load(
    const SkillSet& skills, BT::BehaviorTreeFactory& factory, const std::string& xml,
    const BT::Blackboard::Ptr& blackboard, BT::Tree& tree)
{
    try
    {
        tree = factory.createTreeFromText(
            withDecimalNumbers(xml, skills.validator.catalog()),
            blackboard);
        return std::nullopt;
    }
    catch (const std::exception& e)
    {
        return Diagnostic{ code::kLoadError,
                           0,
                           "",
                           std::string("The robot could not load the mission: ") + e.what() };
    }
}

bool MissionExecutor::takeAuthority(
    const SkillSet& skills, const Validation& validation, Outcome& outcome)
{
    {
        // Arms a lend gave back may still be being let go of; taking them meanwhile would race
        // the release, and finish() would release them a second time.
        std::unique_lock<std::mutex> lock(stop_mutex_);
        awaitRelease(lock);
    }
    bool base = false;
    bool arms = false;
    for (const std::string& id : validation.macros)
    {
        if (const Skill* skill = skills.validator.catalog().findByMacro(id))
        {
            base = base ||
                   std::ranges::find(skill->resources, kBaseResource) != skill->resources.end();
            arms = arms || skill->needsArms();
        }
    }
    bool held     = false;
    bool doubtful = false;
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        base_held_ = base;
        held       = arms_held_;
        doubtful   = hands_.left.unknown || hands_.right.unknown;
    }
    // Held already, by a mission that left an object in a hand: nothing to take. Except when a hand
    // may hold something nobody knows about: the arms are looked at again then, since someone may
    // have let go of them meanwhile, and that settles what the hands hold.
    if (arms && (!held || doubtful))
    {
        RCLCPP_INFO(get_logger(), "taking the arms for the mission");
        Acquired taken = Acquired::kFailed;
        try
        {
            taken = authority_->acquire();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "taking the arms threw: %s", e.what());
        }
        if (taken == Acquired::kFailed)
        {
            outcome.code   = ExecuteMission::Result::OUTCOME_ERROR;
            outcome.reason = "the arms could not be taken; is the control stack up?";
            return false;
        }
        const std::lock_guard<std::mutex> lock(live_mutex_);
        arms_held_ = true;
        if (taken == Acquired::kTaken && doubtful)
        {
            RCLCPP_INFO(
                get_logger(),
                "the arms had been let go of since: nothing can be in a hand that was not known "
                "to hold something");
            hands_.left.unknown  = false;
            hands_.right.unknown = false;
            syncBlackboardLocked();
        }
        else if (taken == Acquired::kAlready && !held && !hands_empty_on_attach_)
        {
            // Someone took the arms before this executor did, possibly one that had picked
            // something up (an executor that restarted while a hand held an object). Letting go of
            // them would drop it, so both hands count as full until a place shows otherwise.
            RCLCPP_WARN(
                get_logger(),
                "the arms were already taken: what the hands hold is not known, so the arms are "
                "kept and both hands count as holding something (hands_empty_on_attach:=true says "
                "nothing can be in a hand at start)");
            hands_.left.unknown  = hands_.left.id.empty();
            hands_.right.unknown = hands_.right.id.empty();
            syncBlackboardLocked();
        }
    }
    pushState();
    return true;
}

MissionExecutor::Outcome
MissionExecutor::execute(const ExecuteMission::Goal& goal, const std::shared_ptr<GoalHandle>& handle)
{
    const Diagnostic shutting_down{ code::kShuttingDown,
                                    0,
                                    "",
                                    "The executor is shutting down and takes no more missions." };
    if (shutting_down_)
    {
        return rejected({ shutting_down });
    }
    if (goal.mode > ExecuteMission::Goal::MODE_DRY_RUN)
    {
        return rejected(
            { { code::kBadMode, 0, "", "mode must be 0 (execute), 1 (validate) or 2 (dry run)." } });
    }
    const bool execute_mode = goal.mode == ExecuteMission::Goal::MODE_EXECUTE;
    // The deadman's clock starts at acceptance: taking the arms can take seconds. The agent
    // asks for its timeout, but a long one would be no deadman at all.
    {
        const std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        heartbeat_client_ = goal.heartbeat_client;
    }
    deadman_fired_   = false;
    unhealthy_since_ = 0;
    heartbeat_ticks_ = Clock::now().time_since_epoch().count();
    heartbeat_timeout_ticks_ =
        execute_mode && std::isfinite(goal.heartbeat_timeout_s) && goal.heartbeat_timeout_s > 0.0F ?
            inClockUnits(
                std::min(static_cast<double>(goal.heartbeat_timeout_s), max_heartbeat_timeout_s_))
                .count() :
            0;

    // The skills it is checked and built against stay its own, whatever the robot learns meanwhile.
    mission_skills_             = repertoire_->current();
    const SkillSet&  skills     = *mission_skills_;
    const Validation validation = skills.validator.validate(goal.tree_xml, goal.tree_sha256);
    if (!validation.ok())
    {
        return rejected(validation.diagnostics);
    }
    BT::BehaviorTreeFactory factory =
        makeFactory(skills, !execute_mode, validation, validation.macros);
    BT::Tree   tree;
    const auto blackboard = seededBlackboard();
    if (!execute_mode)
    {
        // A dry run's stand-ins hand nothing on, and the macros' scripts read what a later step
        // takes: each output gets its key's name as a value.
        for (const std::string& key : validation.outputs)
        {
            blackboard->set<std::string>(key, key);
        }
    }
    if (auto problem = load(skills, factory, goal.tree_xml, blackboard, tree))
    {
        return rejected({ std::move(*problem) });
    }
    if (goal.mode == ExecuteMission::Goal::MODE_VALIDATE)
    {
        return {};
    }
    if (execute_mode)
    {
        std::set<std::string> needs;
        for (const std::string& macro : validation.macros)
        {
            if (const Skill* skill = skills.validator.catalog().findByMacro(macro))
            {
                needs.insert(skill->resources.begin(), skill->resources.end());
            }
        }
        if (auto why = healthRefusal(needs))
        {
            return rejected({ { code::kRobotCannotMove, 0, "", *why } });
        }
        mission_walks_ = needs.contains(kBaseResource);
    }

    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        mission_blackboard_ = blackboard;
        mission_id_         = goal.mission_id;
        mission_step_.clear();
        note_.clear();
        dry_running_ = !execute_mode;
        dry_hands_   = hands_;
        if (execute_mode)
        {
            stopped_ = false;
        }
    }
    pushState();

    Outcome outcome;
    // Taking the arms can take seconds; not for a mission the executor is about to drop.
    if (shutting_down_)
    {
        return rejected({ shutting_down });
    }
    if (execute_mode && !takeAuthority(skills, validation, outcome))
    {
        return outcome;
    }

    const double limit_s = watchdogLimitS(
        static_cast<double>(goal.max_duration_s),
        validation.worst_case_s,
        duration_factor_,
        max_duration_cap_s_);

    std::optional<BT::Groot2Publisher> groot2;
    if (execute_mode && groot2_port_ > 0)
    {
        try
        {
            groot2.emplace(tree, static_cast<unsigned>(groot2_port_));
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(), "Groot2 cannot monitor on port %d: %s", groot2_port_, e.what());
        }
    }

    MissionObserver::Hooks hooks;
    hooks.on_step = [this](const std::string& step) {
        {
            const std::lock_guard<std::mutex> lock(live_mutex_);
            mission_step_ = step;
        }
        pushState();
    };
    // A dry run follows what its own picks and places do to a copy of the hands, so that a
    // mission that picks and then places is not refused for a hand it has not really filled.
    hooks.on_success = [this](const BT::TreeNode& node) { noteLeafSuccess(node); };
    hooks.on_halt    = [this](const BT::TreeNode& node) { noteLeafHalted(node); };
    auto observer    = std::make_unique<MissionObserver>(
        tree.rootNode(),
        get_clock(),
        event_level_,
        skillLeavesOf(factory),
        std::move(hooks));

    const auto     begin  = Clock::now();
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    Ended          ended  = Ended::kDone;
    std::string    thrown;
    deadline_ticks_ = (begin + inClockUnits(limit_s)).time_since_epoch().count();
    try
    {
        ended = tickLoop(tree, *observer, handle, limit_s, begin, status);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "the tree threw: %s", e.what());
        thrown = e.what();
        ended  = Ended::kShutdown;
    }

    // A dry run fills each output with its own key; only a real run hands anything on.
    if (execute_mode)
    {
        outcome.outputs = outputsJson(*blackboard, validation.outputs);
    }
    // Where the mission was, taken before the halt clears it.
    const std::string step = observer->currentStep();
    std::string       winding_down;
    if (ended != Ended::kDone || status != BT::NodeStatus::SUCCESS)
    {
        winding_down = halt(tree);
    }
    // As it stands: a mission stopped while paused has not gone on.
    sendFeedback(handle, *observer, begin, paused_);
    const std::optional<Failure> failure = observer->failure();
    observer.reset();
    groot2.reset();

    const auto running_step = [&](Outcome& out) {
        out.failed_node = step;
        out.failed_step = stepIdOf(step);
    };
    const std::string tail = winding_down.empty() ? "" : "; " + winding_down;
    if (!thrown.empty())
    {
        outcome.code   = ExecuteMission::Result::OUTCOME_ERROR;
        outcome.reason = "the tree threw: " + thrown;
        return outcome;
    }
    switch (ended)
    {
        case Ended::kDone:
            break;
        case Ended::kCanceled:
            outcome.code   = ExecuteMission::Result::OUTCOME_CANCELED;
            outcome.reason = "cancelled" + tail;
            running_step(outcome);
            return outcome;
        case Ended::kStopped:
        {
            const std::lock_guard<std::mutex> lock(stop_mutex_);
            outcome.code   = ExecuteMission::Result::OUTCOME_CANCELED;
            outcome.reason = "stopped: " + stop_reason_ + tail;
            outcome.cause  = stop_cause_;
            running_step(outcome);
            return outcome;
        }
        case Ended::kTimeout:
            outcome.code   = ExecuteMission::Result::OUTCOME_TIMEOUT;
            outcome.reason = std::format("the mission ran past its {:.0f} s limit", limit_s) + tail;
            running_step(outcome);
            return outcome;
        case Ended::kShutdown:
            outcome.code   = ExecuteMission::Result::OUTCOME_CANCELED;
            outcome.reason = "the executor is shutting down" + tail;
            outcome.cause  = ExecuteMission::Result::CAUSE_SHUTDOWN;
            running_step(outcome);
            return outcome;
        case Ended::kPausedTooLong:
            outcome.code   = ExecuteMission::Result::OUTCOME_CANCELED;
            outcome.reason = std::format("paused for longer than {:.0f} s", max_pause_s_) + tail;
            running_step(outcome);
            return outcome;
    }

    if (status == BT::NodeStatus::SUCCESS)
    {
        return outcome;
    }
    outcome.code = failure && failure->timed_out ? ExecuteMission::Result::OUTCOME_TIMEOUT :
                                                   ExecuteMission::Result::OUTCOME_FAILURE;
    if (failure)
    {
        outcome.failed_node = failure->path;
        outcome.failed_step = failure->step;
        outcome.reason      = failure->reason + tail;
        if (failure->server_missing)
        {
            outcome.cause = ExecuteMission::Result::CAUSE_NO_SERVER;
        }
    }
    else
    {
        outcome.reason = "the mission tree returned FAILURE" + tail;
    }
    return outcome;
}

std::optional<MissionExecutor::Ended> MissionExecutor::interruption(
    const std::shared_ptr<GoalHandle>& handle, Clock::time_point deadline) const
{
    if (shutting_down_)
    {
        return Ended::kShutdown;
    }
    if (stop_requested_)
    {
        return Ended::kStopped;
    }
    if (handle->is_canceling())
    {
        return Ended::kCanceled;
    }
    if (Clock::now() >= deadline)
    {
        return Ended::kTimeout;
    }
    return std::nullopt;
}

void MissionExecutor::watchdog()
{
    const Clock::rep deadline = deadline_ticks_.load();
    if (deadline != 0 && Clock::now().time_since_epoch().count() >= deadline)
    {
        halting_ = true;
    }
}

MissionExecutor::Ended MissionExecutor::tickLoop(
    BT::Tree& tree, MissionObserver& observer, const std::shared_ptr<GoalHandle>& handle,
    double limit_s, Clock::time_point begin, BT::NodeStatus& status)
{
    const auto period      = inClockUnits(1.0 / tick_rate_hz_);
    auto       next_report = begin;
    auto       deadline    = begin + inClockUnits(limit_s);
    Pausing    pausing;
    for (;;)
    {
        // While paused the watchdog waits: its limit is for running, and a pause has its own.
        if (const auto why =
                interruption(handle, pausing.since ? Clock::time_point::max() : deadline))
        {
            return *why;
        }
        if (pausing.since && Clock::now() - *pausing.since >= inClockUnits(max_pause_s_))
        {
            return Ended::kPausedTooLong;
        }
        const auto tick_start = Clock::now();

        const bool tick = followPause(tree, observer, pausing, deadline);
        if (tick)
        {
            status = tree.tickOnce();
        }
        if (tick_start >= next_report || (tick && status != BT::NodeStatus::RUNNING))
        {
            sendFeedback(handle, observer, begin, paused_);
            next_report = tick_start + inClockUnits(feedback_period_s_);
        }
        if (tick && status != BT::NodeStatus::RUNNING)
        {
            // Once the mission is told to wind down the leaves refuse to start anything, and a
            // tree that then fails has failed for that, not for anything it did.
            if (status == BT::NodeStatus::FAILURE)
            {
                if (const auto why = interruption(handle, deadline))
                {
                    return *why;
                }
            }
            return Ended::kDone;
        }

        // Until the next tick, serve the leaves' callbacks, but leave at once for a stop.
        const auto next_tick = tick_start + period;
        while (Clock::now() < next_tick)
        {
            skills_executor_.spin_once(
                std::min<Clock::duration>(next_tick - Clock::now(), std::chrono::milliseconds(10)));
            if (stop_requested_ || shutting_down_ || handle->is_canceling())
            {
                break;
            }
        }
    }
}

bool MissionExecutor::followPause(
    BT::Tree& tree, const MissionObserver& observer, Pausing& pausing, Clock::time_point& deadline)
{
    const auto note = [this](std::string text) {
        {
            const std::lock_guard<std::mutex> lock(live_mutex_);
            note_ = std::move(text);
        }
        pushState();
    };
    if (!pause_wanted_)
    {
        if (pausing.since || !pausing.finishing.empty())
        {
            if (pausing.since)
            {
                // The limit is for running: the pause, and what the halted step does again, are
                // given back.
                deadline += Clock::now() - *pausing.since + pausing.redone;
                deadline_ticks_ = deadline.time_since_epoch().count();
            }
            pausing = {};
            pause_gate_.open();
            paused_ = false;
            RCLCPP_INFO(get_logger(), "resumed");
            note({});
        }
        return true;
    }
    if (pausing.since)
    {
        return false;
    }
    // The running step by its name, or one named otherwise by what runs.
    std::string step  = observer.currentStep();
    StepNodes   nodes = findStep(tree.rootNode(), tree.rootNode(), step);
    const bool  by_name =
        nodes.subtree != nullptr && nodes.subtree->status() == BT::NodeStatus::RUNNING;
    if (!by_name)
    {
        nodes = findStep(tree.rootNode(), tree.rootNode(), {});
        step  = nodes.subtree != nullptr ? pathOf(*nodes.subtree) : std::string();
    }
    const bool   busy = nodes.subtree != nullptr;
    const Skill* skill =
        busy && mission_skills_ ?
            mission_skills_->validator.catalog().findByMacro(nodes.subtree->subtreeID()) :
            nullptr;
    // Halted half way, a pick or a place could leave an object in the air: the one running when
    // the pause came ends first, and the gate holds the next step until then. Until it ends the
    // robot still moves, so it is not reported paused.
    if (skill != nullptr && skill->risk == "manipulation" &&
        (pausing.finishing.empty() || pausing.finishing == step))
    {
        if (pausing.finishing.empty())
        {
            pausing.finishing = step;
            pause_gate_.close(step);
            RCLCPP_INFO(get_logger(), "pausing once %s ends", step.c_str());
            note("pausing once " + step + " ends");
        }
        return true;
    }
    // Halted from its top, the step starts afresh on resume: its timeout and retries too.
    pause_gate_.close();
    if (busy && nodes.top != nullptr)
    {
        if (by_name)
        {
            pausing.redone = Clock::now() - observer.stepStarted();
        }
        nodes.top->haltNode();
        spinLeaves(Seconds(cancel_settle_s_));
        if (const std::vector<std::string> left = sweepAfterHalt(); !left.empty())
        {
            RCLCPP_WARN(get_logger(), "still winding down after the pause: %s", join(left).c_str());
        }
    }
    pausing.finishing.clear();
    pausing.since   = Clock::now();
    paused_         = true;
    deadline_ticks_ = 0;
    RCLCPP_INFO(get_logger(), "paused%s%s", busy ? " during " : "", busy ? step.c_str() : "");
    note(busy ? "paused during " + step : std::string("paused"));
    return false;
}

void MissionExecutor::sendFeedback(
    const std::shared_ptr<GoalHandle>& handle, MissionObserver& observer, Clock::time_point begin,
    bool paused)
{
    auto feedback           = std::make_shared<ExecuteMission::Feedback>();
    feedback->elapsed_s     = static_cast<float>(Seconds(Clock::now() - begin).count());
    feedback->running_nodes = observer.running();
    feedback->events        = observer.takeEvents();
    feedback->paused        = paused;
    handle->publish_feedback(feedback);
}

void MissionExecutor::spinLeaves(Seconds span)
{
    const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(span);
    while (Clock::now() < end)
    {
        skills_executor_.spin_once(
            std::min<Clock::duration>(end - Clock::now(), std::chrono::milliseconds(10)));
    }
}

std::vector<std::string> MissionExecutor::sweepAfterHalt()
{
    // A goal sent just before the halt can have been accepted only after it, when the leaf no
    // longer had a handle to cancel it by: whatever is still in flight now is asked to stop. And
    // as for a stop, the arm's controller holds the arm where it is, which a skill server that
    // runs its trajectory to the end would not.
    if (!leaf_servers_->busy().empty())
    {
        (void)leaf_servers_->cancelAll();
    }
    (void)arm_controller_->cancelAll();
    const auto deadline  = Clock::now() + inClockUnits(stop_wait_s_);
    const auto remaining = [&deadline] {
        return Seconds(std::max(0.0, Seconds(deadline - Clock::now()).count()));
    };
    std::vector<std::string> busy = leaf_servers_->settle(remaining());
    for (std::string& server : arm_controller_->settle(remaining()))
    {
        busy.push_back(std::move(server));
    }
    return busy;
}

std::string MissionExecutor::halt(BT::Tree& tree)
{
    // haltTree cancels each leaf's goal; the spin lets those requests reach their servers and the
    // answers come back while the leaves still exist to receive them.
    tree.haltTree();
    spinLeaves(Seconds(cancel_settle_s_));
    {
        const std::lock_guard<std::mutex> lock(halted_mutex_);
        halted_ = true;
    }
    halted_cv_.notify_all();
    const std::vector<std::string> busy = sweepAfterHalt();
    if (busy.empty())
    {
        return {};
    }
    RCLCPP_WARN(get_logger(), "goals still winding down after the halt: %s", join(busy).c_str());
    return "still winding down: " + join(busy);
}

void MissionExecutor::finish(bool execute_mode)
{
    if (execute_mode)
    {
        (void)letGoOfArms();
    }
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        mission_id_.clear();
        mission_step_.clear();
        mission_blackboard_.reset();
        base_held_   = false;
        dry_running_ = false;
        note_.clear();
    }
    pushState();
}

// --- what the robot holds ----------------------------------------------------------------------

void MissionExecutor::noteLeafSuccess(const BT::TreeNode& node)
{
    const std::string& kind = node.registrationName();
    if (kind != "Pick" && kind != "Place")
    {
        return;
    }
    const std::string arm  = node.getInput<std::string>("arm").value_or("right");
    bool              real = false;
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        Hands&                            hands = handsInUseLocked();
        real                                    = !dry_running_;
        Held& hand                              = arm == "left" ? hands.left : hands.right;
        if (kind == "Pick")
        {
            hand.seen_as = node.getInput<std::string>("object_id").value_or("");
            // What the mission called it and the words the detector found it by are arguments of
            // the macro the leaf sits in; a leaf in a hand-written tree has only its own port.
            hand.id      = hand.seen_as;
            hand.phrase  = "";
            hand.unknown = false;
            if (node.config().blackboard)
            {
                (void)node.config().blackboard->get("object_id", hand.id);
                (void)node.config().blackboard->get("phrase", hand.phrase);
            }
            // A dry run's stand-in for LookFor writes no id, so nothing says what the detector
            // would have called the object; the mission's own id will do for a what-if.
            if (hand.seen_as.empty())
            {
                hand.seen_as = hand.id;
            }
        }
        else
        {
            // A place that succeeded put down whatever the hand held, or did not know it held.
            hand = Held{};
        }
        // The macros read what a hand holds from here, so it has to follow a pick or a place made
        // earlier in the same mission, not only the state the mission started from.
        syncBlackboardLocked();
    }
    if (real)
    {
        pushState();
    }
}

void MissionExecutor::noteLeafHalted(const BT::TreeNode& node)
{
    // One a pause held back never sent its goal, so it cannot have closed the hand.
    const auto* report = dynamic_cast<const LeafReport*>(&node);
    if (node.registrationName() != "Pick" || (report != nullptr && !report->goalSent()))
    {
        return;
    }
    // A pick that was halted while it ran had its goal out, and it may have closed the hand on the
    // object before the cancel arrived. Nothing reports how it ended, and letting go of the arm
    // with a full hand drops the object, so the hand counts as full until a place shows otherwise.
    const std::string arm  = node.getInput<std::string>("arm").value_or("right");
    bool              real = false;
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        Hands&                            hands = handsInUseLocked();
        real                                    = !dry_running_;
        Held& hand                              = arm == "left" ? hands.left : hands.right;
        hand.unknown                            = hand.id.empty();
        syncBlackboardLocked();
    }
    if (real)
    {
        pushState();
    }
}

void MissionExecutor::syncBlackboardLocked()
{
    if (mission_blackboard_)
    {
        const Hands& hands = handsInUseLocked();
        mission_blackboard_->set<std::string>("held_left", entryOf(hands.left));
        mission_blackboard_->set<std::string>("held_right", entryOf(hands.right));
    }
}

bool MissionExecutor::mayHoldSomethingLocked() const
{
    return std::ranges::any_of(std::array{ &hands_.left, &hands_.right }, [](const Held* hand) {
        return !hand->id.empty() || hand->unknown;
    });
}

std::string MissionExecutor::entryOf(const Held& held)
{
    if (held.seen_as.empty())
    {
        // What the macros read for a hand that may hold something they cannot name.
        return held.unknown ? kUnknownHeld : std::string{};
    }
    return held.phrase.empty() ? held.seen_as : held.seen_as + "=" + held.phrase;
}

std::string MissionExecutor::describeHandsLocked(const char* verb) const
{
    std::string text;
    const auto  describe = [&](const char* side, const Held& hand) {
        if (!hand.id.empty())
        {
            text += std::format("; {} hand {} {}", side, verb, hand.id);
        }
        else if (hand.unknown)
        {
            text += std::format("; {} hand unknown (it may hold something)", side);
        }
    };
    describe("left", hands_.left);
    describe("right", hands_.right);
    return text;
}

void MissionExecutor::pushState()
{
    // Called from the mission thread too, and from a timer at shutdown: a publish that throws
    // must not take either down.
    try
    {
        const std::lock_guard<std::mutex> lock(live_mutex_);
        RobotState                        state;
        state.stamp        = now();
        state.boot_id      = boot_id_;
        state.mission_id   = mission_id_;
        state.mission_step = mission_step_;
        state.teleop       = teleop_ && teleop_->active();
        if (base_held_ || state.teleop)
        {
            state.resources_held.emplace_back(kBaseResource);
        }
        if (health_)
        {
            const Health health      = health_->now();
            state.can_move           = health.can_walk;
            state.cannot_move_reason = health.can_walk ? std::string() : health.reason;
            state.tilt_deg           = static_cast<float>(health.tilt_deg);
        }
        else
        {
            state.can_move = true;
            state.tilt_deg = std::numeric_limits<float>::quiet_NaN();
        }
        if (arms_held_)
        {
            // Both arms and both hands are taken together, so both are held.
            state.resources_held.emplace_back(kLeftArmResource);
            state.resources_held.emplace_back(kRightArmResource);
        }
        state.holding_left  = hands_.left.id;
        state.holding_right = hands_.right.id;
        state.stopped       = stopped_;
        state.paused        = paused_;
        if (!note_.empty())
        {
            state.message = note_;
        }
        else if (stopped_)
        {
            state.message = "holding posture" + describeHandsLocked("keeps");
        }
        else if (!mission_id_.empty())
        {
            state.message = "running" +
                            (mission_step_.empty() ? std::string() : " " + mission_step_) +
                            describeHandsLocked("holds");
        }
        else
        {
            state.message = "idle" + describeHandsLocked("holds");
        }
        state_pub_->publish(state);
    }
    catch (const std::exception& e)
    {
        RCLCPP_DEBUG(get_logger(), "the robot state was not published: %s", e.what());
    }
}

}  // namespace g1_orchestration
