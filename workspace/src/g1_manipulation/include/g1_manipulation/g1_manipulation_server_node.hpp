#ifndef G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
#define G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_

/**
 * @file g1_manipulation_server_node.hpp
 * @brief Pick, place and named-posture skills, served as actions over MoveIt.
 *
 * Adds no command path: every motion goes out through the same `move_group` the RViz panel
 * uses, onto the controllers that already own the body motors and the hand topics, so the
 * one-writer rule is unaffected by this node existing.
 *
 * Takes no control authority of its own. The arm and hands must already be acquired before a
 * goal will execute, and releasing them is the caller's job.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <atomic>
#include <g1_msgs/action/pick.hpp>
#include <g1_msgs/action/place.hpp>
#include <g1_msgs/action/set_arm_posture.hpp>
#include <g1_msgs/srv/generate_grasps.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace g1_manipulation
{

/**
 * @brief The MoveIt groups and links one arm brings, resolved from a goal's `arm` field.
 */
struct ArmContext
{
    std::string arm_group;   ///< left_arm / right_arm
    std::string hand_group;  ///< left_hand / right_hand
    std::string palm_link;   ///< the arm group's tip, and what an object attaches to
    /// Where the hand actually closes. A frame in the URDF, so pose goals are given for it
    /// directly and nothing here does offset arithmetic.
    std::string grasp_frame;
    /// The two hands mirror, so the palm roll flips sign with this.
    bool is_left{ false };
};

/**
 * @brief Resolves an arm name to its group, frames and handedness.
 *
 * @param[out] out Set only when the name is recognised.
 * @return False if @p arm is neither "left" nor "right", leaving @p out untouched.
 */
bool resolveArm(const std::string& arm, ArmContext& out);

class G1ManipulationServer : public rclcpp::Node
{
public:
    explicit G1ManipulationServer(const rclcpp::NodeOptions& options);

    /**
     * @brief Waits for any goal still running on a detached thread.
     *
     * Without the wait those threads outlive the MoveGroups, the planning scene and the
     * service clients they are dereferencing.
     */
    ~G1ManipulationServer() override;

    /**
     * @brief Builds the MoveGroupInterfaces.
     *
     * Separate from the constructor because MoveGroupInterface blocks until it has the robot
     * description and the current state, which only arrive once something is spinning this
     * node; constructing one from inside the constructor deadlocks.
     */
    void initialize();

private:
    using Pick          = g1_msgs::action::Pick;
    using Place         = g1_msgs::action::Place;
    using SetArmPosture = g1_msgs::action::SetArmPosture;
    using MoveGroup     = moveit::planning_interface::MoveGroupInterface;

    template <typename ActionT>
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    /**
     * @brief Runs a pick to completion: approach, grasp, lift.
     */
    void executePick(const std::shared_ptr<GoalHandle<Pick>>& goal_handle);

    /**
     * @brief Runs a place to completion: approach, release, retreat.
     */
    void executePlace(const std::shared_ptr<GoalHandle<Place>>& goal_handle);

    /**
     * @brief Moves one planning group to a named SRDF pose.
     */
    void executeSetArmPosture(const std::shared_ptr<GoalHandle<SetArmPosture>>& goal_handle);

    /**
     * @brief Claims the arm for one goal.
     *
     * @return true if this goal may run, false if another one already holds the arm.
     */
    bool acquire();

    /**
     * @brief The /objects array frame, read under objects_mutex_.
     */
    std::string objectsFrame();

    /**
     * @brief Runs one goal body, balancing the running count and releasing the arm.
     *
     * Turns an escaping exception into an aborted goal rather than std::terminate on a
     * detached thread.
     *
     * @tparam ActionT The action this goal belongs to.
     * @tparam Body Callable holding the skill itself.
     */
    template <typename ActionT, typename Body>
    void
    runGuarded(Body&& body, const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>& handle)
    {
        goals_running_.fetch_add(1);
        try
        {
            body();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "manipulation goal threw: %s", e.what());
            auto result     = std::make_shared<typename ActionT::Result>();
            result->success = false;
            result->message = std::string("aborted on an internal error: ") + e.what();
            if (handle->is_executing() || handle->is_canceling())
            {
                handle->abort(result);
            }
        }
        busy_.store(false);
        goals_running_.fetch_sub(1);
    }

    /**
     * @brief Latest pose for @p object_id.
     *
     * @return nullopt if the object is unknown or its pose is older than the timeout.
     */
    std::optional<vision_msgs::msg::Detection3D> lookUpObject(const std::string& object_id);

    /**
     * @brief Stores the latest detection array under objects_mutex_.
     */
    void onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg);

    /**
     * @brief Whether the hand is holding something, from the fingers' own positions and efforts.
     *
     * The trajectory controller reports a blocked finger as success, so closing the hand proves
     * nothing on its own. Targets come from the SRDF `closed` posture the close was planned to.
     *
     * @param[out] why What the fingers are doing, for the result message either way.
     * @return true when the check is disabled, so a robot with no effort feedback still picks.
     */
    [[nodiscard]] bool isHolding(const ArmContext& arm, std::string& why);

    /**
     * @brief Transforms a pose into the planning frame.
     *
     * Everything a goal carries goes through here: /objects is in odom, the planner works in
     * pelvis, and the two differ by wherever the robot is standing.
     *
     * @return nullopt with the reason logged.
     */
    std::optional<geometry_msgs::msg::Pose>
    toPlanningFrame(const geometry_msgs::msg::Pose& pose, const std::string& frame_id);

    /**
     * @brief The pose to give the arm's grasp frame so the object ends up at @p object_pose.
     *
     * Position passes straight through, since the grasp frame is where the object goes, and
     * only the orientation is chosen here. The two hands hold at mirrored rolls.
     *
     * @param object_height_m The object's full height. The grasp is taken just under its top
     *        face, and never nearer its base than min_grip_height_m, which is as far past the
     *        grasp frame as the hand itself reaches.
     */
    geometry_msgs::msg::Pose graspFrameGoal(
        const geometry_msgs::msg::Pose& object_pose, double object_height_m,
        const ArmContext& arm) const;

    /**
     * @brief Moves to @p pose in a straight line, falling back to a planned path.
     *
     * For the last stretch into a grasp, where the hand is inches from a surface and a sampling
     * planner has almost no free space to sample.
     */
    bool moveAlongApproach(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /**
     * @brief Walks @p link toward @p pose in a straight line, however far it gets.
     *
     * Never plans around, deliberately: the correction callers pass 0 and take whatever the line
     * gives, because a planner free to route around arrives from a direction that sweeps the
     * object away.
     *
     * @param min_fraction Below this the line is measured and reported but not executed, leaving
     *        the arm where it was for a caller that has a fallback.
     * @return The fraction the line covered, or 0 if it could not be walked or executed.
     */
    double moveStraight(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what, double min_fraction);

    /**
     * @brief How far @p link is from @p pose right now, in the planning frame.
     *
     * @return nullopt if TF cannot answer, which is a stale or unpublished frame rather than a
     *         missed target.
     */
    std::optional<geometry_msgs::msg::Point> residualTo(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /**
     * @brief Descends onto @p grasp, re-aiming from above until the hand measures there.
     *
     * The arm settles short of its target, and by a different amount in every configuration, so
     * the error after the descent is not the error that was corrected at @p pregrasp. Each retry
     * lifts back up before re-aiming: a sideways correction at object height pushes the object
     * away instead of reaching it.
     */
    bool descendOnto(
        MoveGroup& group, const geometry_msgs::msg::Pose& pregrasp,
        const geometry_msgs::msg::Pose& grasp, const std::string& link);

    /**
     * @brief Nudges @p link onto @p pose until TF says it is there.
     *
     * The arm is position-only and settles short of its target under gravity, by more than the
     * grip is wide. Each pass re-commands the measured residual on top of the last target.
     *
     * @return false only if the arm cannot be measured or a nudge fails to execute; running out
     *         of attempts closes from wherever it got to, which the grip check then judges.
     */
    bool settleOnPose(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /// Where the hand goes and where it starts from, whichever source the grasp came from.
    struct GraspPlan
    {
        geometry_msgs::msg::Pose grasp;
        geometry_msgs::msg::Pose pregrasp;
        /// What produced it, for the result message: a generated grasp is worth naming.
        std::string origin;
    };

    /// A generated candidate the filter looked at, in the planning frame, and what it decided.
    struct ConsideredGrasp
    {
        enum class Verdict : std::uint8_t
        {
            kTilted,
            kUnreachable,
            kChosen,
        };
        geometry_msgs::msg::Pose pose;
        Verdict                  verdict{ Verdict::kTilted };
    };

    /**
     * @brief The grasp to attempt for one object, from whichever source is configured.
     *
     * With `grasp_source: fixed_top_down` this is the pose graspFrameGoal computes, reached from
     * straight above. With `generated` it asks the grasp service and keeps the best candidate
     * the arm can actually take: scored, not reaching up through the surface, and solvable.
     *
     * @param[out] why Why there is nothing to attempt, when it returns nothing. There is no
     *             fallback to the fixed grasp on purpose: a pick that quietly stops using the
     *             generator is a pick nobody knows is not using it.
     * @param[out] verdicts Every candidate the filter judged, for drawing; null skips the record.
     */
    std::optional<GraspPlan> chooseGrasp(
        const vision_msgs::msg::Detection3D& detection, const geometry_msgs::msg::Pose& object_pose,
        const ArmContext& arm, std::string& why, std::vector<ConsideredGrasp>* verdicts);

    /// Draws the candidates and the chosen grasp on ~/grasp_plan; @p plan is null when none was.
    void publishGraspPlan(const GraspPlan* plan, const std::vector<ConsideredGrasp>& verdicts);

    /// Calls the grasp service, or nothing with the reason in @p why.
    std::optional<g1_msgs::srv::GenerateGrasps::Response>
    requestGrasps(const std::string& object_id, const ArmContext& arm, std::string& why);

    /**
     * @brief Seeds the plan from the measured state, clamped into the group's URDF limits.
     *
     * MoveIt's start-state check rejects a joint that is outside by any amount at all, and a
     * joint commanded to its own limit tracks a fraction past it.
     */
    static void setStartStateInBounds(MoveGroup& group);

    /**
     * @brief Plans and executes so that @p link reaches @p pose.
     *
     * @param what Name of the step, used in the failure log.
     * @return False on either a planning or an execution failure, logged.
     */
    bool moveTo(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /**
     * @brief Plans for the group's current target, sampling again within one planning_time_s.
     *
     * A path OMPL accepts can still be rejected by the ValidateSolution adapter once it is time
     * parameterised. That rejection comes back fast, so there is budget left for another sample;
     * a planner that finds nothing spends the whole budget once and stops.
     */
    moveit::core::MoveItErrorCode planWithinBudget(MoveGroup& group, MoveGroup::Plan& plan);

    /**
     * @brief Plans and executes to a named SRDF pose.
     *
     * @return False on either a planning or an execution failure, logged.
     */
    bool moveToNamed(MoveGroup& group, const std::string& named_target);

    /**
     * @brief Puts the object into the planning scene so plans route around it.
     *
     * @param in_planning_frame The object's measured pose, already transformed.
     * @return What was built: the world copy is removed before the grasp, and the attached
     *         body then has to carry the same geometry.
     */
    moveit_msgs::msg::CollisionObject publishCollisionObject(
        const vision_msgs::msg::Detection3D& detection,
        const geometry_msgs::msg::Pose&      in_planning_frame);

    /**
     * @brief Lets the grasping hand touch the named things, or stops letting it.
     *
     * Grasping is contact, and the planner does not distinguish intended contact from a
     * collision. Two things are unavoidably in the way of a grasp and both have to be
     * exempted: the octomap, because the sensor has already seen the support surface and the
     * object as occupied space, and the target object's own collision geometry, which is
     * added precisely so the planner routes around it right up until the moment the hand is
     * supposed to close on it. Without this every grasp pose measures as "reachable but
     * collides".
     *
     * Scoped as narrowly as the problem allows, to the hand and the wrist that carries it on
     * one arm for one skill, and always restored on every failure path, so the arm is never
     * left planning against a permanently blinded scene.
     *
     * This is allowHandContact() plus the log, and is what callers use.
     *
     * @param include_links false exempts the touchables from each other only, leaving the hand
     *        and wrist collision-checked, which is what carrying an object over a surface wants.
     */
    void setHandContact(
        const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
        bool include_links = true);

    /**
     * @brief setHandContact() without the error log, for a caller that must see the failure.
     *
     * @return false if the arm has no hand group or a planning-scene service did not answer, in
     *         which case the exemption was neither applied nor restored. A silently failed
     *         restore leaves the scene blinded, and a silently failed apply reads downstream as
     *         an unreachable pose.
     */
    [[nodiscard]] bool allowHandContact(
        const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
        bool include_links = true);

    /**
     * @brief The MoveGroupInterface for a planning group.
     *
     * @return nullptr if no group of that name was built.
     */
    MoveGroup* groupFor(const std::string& name);

    std::map<std::string, std::shared_ptr<MoveGroup>>  groups_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    std::mutex                                                          objects_mutex_;
    vision_msgs::msg::Detection3DArray                                  objects_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex                                                    joint_states_mutex_;
    sensor_msgs::msg::JointState                                  joint_states_;

    /// Kept alive for as long as the node is: dropping the handle removes the callback.
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// Null unless publish_markers is set, so a run without visualization pays nothing for it.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasp_plan_pub_;

    rclcpp::Client<g1_msgs::srv::GenerateGrasps>::SharedPtr         grasps_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr   get_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;

    rclcpp_action::Server<Pick>::SharedPtr          pick_server_;
    rclcpp_action::Server<Place>::SharedPtr         place_server_;
    rclcpp_action::Server<SetArmPosture>::SharedPtr posture_server_;

    std::string planning_frame_;
    double      object_timeout_s_{ 1.0 };
    double      grasp_depth_below_top_m_{ 0.020 };
    /// How far above its own support surface an object must be gripped: the hand hangs 63 mm
    /// below its grasp frame, so below this the thumb rests on the surface, not the object.
    double min_grip_height_m_{ 0.068 };
    /// How close the grasp frame must measure to its target before the hand closes, and how many
    /// corrective nudges it gets to get there.
    double settle_tolerance_m_{ 0.010 };
    int    settle_attempts_{ 2 };
    /// How far back up the approach axis a retry starts from. Above the top of anything this hand
    /// can grip, and short enough that the retry is a line the arm can actually walk.
    double reaim_clearance_m_{ 0.08 };
    bool   grip_check_enabled_{ true };
    double grip_min_position_error_rad_{ 0.08 };
    double grip_min_effort_nm_{ 0.10 };
    double place_tolerance_m_{ 0.08 };
    double approach_height_m_{ 0.22 };
    double lift_height_m_{ 0.15 };
    double velocity_scaling_{ 0.3 };
    double planning_time_s_{ 5.0 };
    int    planning_attempts_{ 5 };
    // How the hand is held at the grasp. Where it grips is the grasp frame in the URDF; only the
    // orientation is a choice, and it is the one thing that depends on the surface rather than
    // on the hand.
    std::vector<double> grasp_rpy_;

    /// "fixed_top_down" or "generated". The default is what this server has always done.
    std::string grasp_source_;
    double      grasp_timeout_s_{ 20.0 };
    double      min_grasp_score_{ 0.5 };
    int         max_grasp_candidates_{ 20 };
    double      max_approach_tilt_rad_{ 0.0 };
    double      approach_standoff_m_{ 0.12 };
    double      ik_timeout_s_{ 0.05 };
    /// Interpolation step and how much of a straight line must be walkable to take it.
    double cartesian_step_m_{ 0.005 };
    double cartesian_min_fraction_{ 0.8 };
    /// Generator gripper frame to `<side>_hand_grasp_frame`; see grasp_filter.hpp.
    std::array<double, 6> graspgen_offset_{};

    /// One goal at a time across ALL THREE servers: MoveGroupInterface is not thread-safe, and two
    /// goals share one arm_trajectory_controller, so the second preempts the first mid-motion.
    std::atomic<bool> busy_{ false };
    std::atomic<int>  goals_running_{ 0 };
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
