#ifndef G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
#define G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_

/**
 * @file g1_manipulation_server_node.hpp
 * @brief Pick, place and named-posture skills, served as actions over MoveIt.
 *
 * Every motion goes through `move_group` onto controllers that already own the motors, so this
 * node adds no command path. It takes no control authority: the arm and hands must already be
 * acquired, and releasing them is the caller's job.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <g1_msgs/action/pick.hpp>
#include <g1_msgs/action/place.hpp>
#include <g1_msgs/action/set_arm_posture.hpp>
#include <g1_msgs/srv/generate_grasps.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <memory>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
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
    std::string arm_group;         ///< left_arm / right_arm
    std::string hand_group;        ///< left_hand / right_hand
    std::string palm_link;         ///< The arm group's tip, and what a held object attaches to.
    std::string grasp_frame;       ///< URDF frame at the point the fingers close on.
    bool        is_left{ false };  ///< The hands mirror, so the palm roll flips sign.
};

/**
 * @brief Resolves an arm name to its group, frames and handedness.
 *
 * @param[out] out Set only when the name is recognised.
 * @return False if @p arm is neither "left" nor "right", leaving @p out untouched.
 */
bool resolveArm(const std::string& arm, ArmContext& out);

/**
 * @brief Where the grasp frame goes for a top grasp of an object.
 *
 * Horizontally on the object; vertically @p depth_below_top_m under its top face, but never lower
 * than @p min_grip_height_m above its base. The roll in @p rpy is mirrored for the left hand.
 *
 * @param object_height_m The object's full height.
 * @param rpy The right hand's orientation, three angles.
 */
geometry_msgs::msg::Pose topGraspGoal(
    const geometry_msgs::msg::Pose& object_pose, double object_height_m, double depth_below_top_m,
    double min_grip_height_m, const std::vector<double>& rpy, bool is_left);

class G1ManipulationServer : public rclcpp::Node
{
public:
    explicit G1ManipulationServer(const rclcpp::NodeOptions& options);

    /**
     * @brief Waits for any goal still running on a detached thread.
     *
     * Those threads dereference the MoveGroups, the planning scene and the service clients.
     */
    ~G1ManipulationServer() override;

    /**
     * @brief Builds the MoveGroupInterfaces and starts the action servers.
     *
     * Separate from the constructor because MoveGroupInterface blocks until something spins this
     * node, which a constructor cannot.
     */
    void initialize();

private:
    using Pick          = g1_msgs::action::Pick;
    using Place         = g1_msgs::action::Place;
    using SetArmPosture = g1_msgs::action::SetArmPosture;
    using MoveGroup     = moveit::planning_interface::MoveGroupInterface;
    using Detection     = vision_msgs::msg::Detection3D;

    template <typename ActionT>
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    /// Where the hand goes and where it starts from, whichever source the grasp came from.
    struct GraspPlan
    {
        geometry_msgs::msg::Pose grasp;
        geometry_msgs::msg::Pose pregrasp;
        std::string              origin;  ///< What produced it, for the log and the markers.
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

    /// Positions and efforts of a set of joints, in the order they were asked for.
    struct JointSample
    {
        std::vector<double> position;
        std::vector<double> effort;  ///< Empty when /joint_states carries no effort.
    };

    /**
     * @brief Declares every parameter and installs the live-update callback.
     */
    void declareParameters();

    /**
     * @brief Runs a pick to completion: locate, pregrasp, descend, close, lift.
     */
    void executePick(const std::shared_ptr<GoalHandle<Pick>>& goal_handle);

    /**
     * @brief Runs a place to completion: preplace, lower, release, retreat, confirm the landing.
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
     * @brief Runs one goal body, balancing the running count and releasing the arm.
     *
     * Turns an escaping exception into an aborted goal rather than std::terminate on a detached
     * thread.
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
     * @brief Stores the latest detection array and each id's newest sighting.
     */
    void onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg);

    /**
     * @brief Latest detection of @p object_id, stamped and framed.
     *
     * @param report Log why nothing usable was found; polling loops pass false.
     * @return nullopt if the object is unknown, its pose is older than object_timeout_ms, or the
     *         pose or size is not finite.
     */
    std::optional<Detection> lookUpObject(const std::string& object_id, bool report = true);

    /**
     * @brief Polls lookUpObject() until it answers or @p timeout_s runs out.
     *
     * Only the last attempt logs.
     */
    std::optional<Detection> waitForObject(const std::string& object_id, double timeout_s);

    /**
     * @brief The last sighting of @p object_id, if within sighting_memory_s.
     *
     * For objects that stand still until the skill moves them: a pick's target and a place's
     * surface. Never for where a released object landed, which needs a new sighting.
     */
    std::optional<Detection> lastSighting(const std::string& object_id);

    /**
     * @brief Latest /joint_states values for @p joints.
     *
     * @return nullopt if any joint is missing.
     */
    std::optional<JointSample> readJoints(const std::vector<std::string>& joints);

    /**
     * @brief Whether the hand is holding something, from the fingers' positions and efforts.
     *
     * The trajectory controller reports a blocked finger as success, so a finished close proves
     * nothing. Targets are `open` to `closed` interpolated to the last commanded close fraction.
     *
     * @param[out] why What the fingers are doing, for the log and the result.
     * @param opposed Also require the thumb among the pressing fingers.
     * @return true when the grip check is disabled, so a hand without effort feedback still picks.
     */
    [[nodiscard]] bool isHolding(const ArmContext& arm, std::string& why, bool opposed = false);

    /**
     * @brief Transforms a pose into the planning frame at the latest transform.
     *
     * @return nullopt with the reason logged.
     */
    std::optional<geometry_msgs::msg::Pose>
    toPlanningFrame(const geometry_msgs::msg::Pose& pose, const std::string& frame_id);

    /**
     * @brief topGraspGoal() with this server's depth, minimum height and orientation.
     */
    geometry_msgs::msg::Pose graspFrameGoal(
        const geometry_msgs::msg::Pose& object_pose, double object_height_m,
        const ArmContext& arm) const;

    /**
     * @brief The grasp to attempt, from the configured grasp source.
     *
     * `fixed_top_down` is graspFrameGoal() approached from straight above. `generated` asks the
     * grasp service and keeps the best candidate that is scored, not reaching up through the
     * surface, and solvable. There is no fallback between the two.
     *
     * @param[out] why Why there is nothing to attempt, when it returns nothing.
     * @param[out] verdicts Every candidate the filter judged, for drawing; null skips the record.
     */
    std::optional<GraspPlan> chooseGrasp(
        const Detection& detection, const geometry_msgs::msg::Pose& object_pose,
        const ArmContext& arm, std::string& why, std::vector<ConsideredGrasp>* verdicts);

    /// Draws the candidates and the chosen grasp on ~/grasp_plan; @p plan is null when none was.
    void publishGraspPlan(const GraspPlan* plan, const std::vector<ConsideredGrasp>& verdicts);

    /// Calls the grasp service, or returns nothing with the reason in @p why.
    std::optional<g1_msgs::srv::GenerateGrasps::Response>
    requestGrasps(const std::string& object_id, const ArmContext& arm, std::string& why);

    /**
     * @brief Seeds the next plan from the measured state, clamped into the group's URDF limits.
     *
     * MoveIt rejects a start joint outside its limit by any amount, and a joint commanded onto its
     * limit tracks a little past it.
     */
    static void setStartStateInBounds(MoveGroup& group);

    /**
     * @brief Plans and executes so that @p link reaches @p pose.
     *
     * @param what Name of the step, for the log.
     * @return False on a planning or an execution failure, logged.
     */
    bool moveTo(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /**
     * @brief Plans for the group's current target, sampling again within one planning_time_s.
     *
     * ValidateSolution can reject a path OMPL accepted, and that rejection returns fast enough to
     * leave budget for another sample.
     */
    moveit::core::MoveItErrorCode planWithinBudget(MoveGroup& group, MoveGroup::Plan& plan);

    /**
     * @brief Plans and executes to a named SRDF pose.
     *
     * @return False on a planning or an execution failure, logged.
     */
    bool moveToNamed(MoveGroup& group, const std::string& named_target);

    /**
     * @brief Walks @p link toward @p pose in a straight line, however far it gets.
     *
     * Waits settle_wait_s after executing, since the controller reports done before the arm stops.
     *
     * @param min_fraction Below this the line is not executed, leaving the arm where it was.
     * @return The fraction the line covered, or 0 if it could not be walked or executed.
     */
    double moveStraight(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what, double min_fraction);

    /**
     * @brief The point reaim_clearance_m back up the approach axis from @p grasp.
     *
     * @return @p grasp itself when the two poses coincide.
     */
    geometry_msgs::msg::Pose stagingPose(
        const geometry_msgs::msg::Pose& pregrasp, const geometry_msgs::msg::Pose& grasp) const;

    /**
     * @brief How far @p link is from @p pose right now, in the planning frame.
     *
     * @return nullopt if TF cannot answer.
     */
    std::optional<geometry_msgs::msg::Point> residualTo(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /**
     * @brief Nudges @p link onto @p pose until TF measures it there.
     *
     * The arm is position-controlled and settles short of its target under gravity, so each pass
     * re-commands the measured residual on top of the last target.
     *
     * @return false only if the arm cannot be measured; running out of attempts is not a failure.
     */
    bool settleOnPose(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);

    /**
     * @brief Descends onto @p grasp from above, re-aiming until the hand measures there.
     *
     * Every attempt starts from the staging point on the @p pregrasp axis and walks the last
     * stretch as a straight line: a sideways correction at object height pushes the object away.
     *
     * @param max_offset_m Still this far off after the last attempt, the descent is refused.
     */
    bool descendOnto(
        MoveGroup& group, const geometry_msgs::msg::Pose& pregrasp,
        const geometry_msgs::msg::Pose& grasp, const std::string& link, double max_offset_m);

    /**
     * @brief Backs the grasp frame reaim_clearance_m straight up from wherever it is.
     *
     * For an exit from low over a surface, before the collision exemptions are restored.
     */
    void backOff(const ArmContext& arm, MoveGroup& group);

    /**
     * @brief Executes a two-point trajectory from the fingers' measured positions to @p targets.
     *
     * Commanded, not planned: closing on an object is deliberate contact, and at the grasp the
     * hand starts inside the octomap, which the planner refuses. The duration scales with the
     * largest normalised finger travel, from 0.15 s up to hand_close_s.
     *
     * @param targets A position for every active joint of @p hand.
     */
    bool commandHand(
        MoveGroup& hand, const std::map<std::string, double>& targets, const std::string& what);

    /// Drives the hand to a named SRDF posture through commandHand().
    bool moveHandTo(MoveGroup& hand, const std::string& named_target);

    /**
     * @brief Drives the hand to @p fraction of the way from `open` to `closed`.
     *
     * @param fraction Clamped to [0, 1]; not finite is refused.
     */
    bool moveHandToFraction(MoveGroup& hand, double fraction, const std::string& what);

    /**
     * @brief Closes onto an object of a given width by creeping in until the thumb opposes a
     *        finger.
     *
     * `closed` is a pose, not a grip: commanded at an object, the fingers stall short and push at
     * kp times an error they can never close, which squeezes the object out.
     *
     * @param widest_m The wider horizontal side of the measured box, where the close starts.
     * @param narrowest_m The narrower side, which bounds how far the close may go.
     * @return false only if the hand would not move; no contact is left to the caller's grip check.
     */
    bool closeHandOn(MoveGroup& hand, const ArmContext& arm, double widest_m, double narrowest_m);

    /**
     * @brief Re-commands each finger to where it stalled plus grip_hold_bias_rad.
     *
     * Equal bias means equal force, kp times the bias, so no finger out-pushes the others and walks
     * the object out. Logs and leaves the close's targets standing if it cannot.
     */
    void holdCurrentGrip(MoveGroup& hand);

    /**
     * @brief Waits until no finger moves faster than grip_settled_speed_rad_s.
     *
     * @return false if they are still moving after hand_close_s, or cannot be read.
     */
    bool waitForHandToSettle(MoveGroup& hand);

    /**
     * @brief Puts the object into the planning scene as its measured box.
     *
     * @param in_planning_frame The object's measured pose, already transformed.
     * @return What was added, so the attach can rebuild the same geometry.
     */
    moveit_msgs::msg::CollisionObject publishCollisionObject(
        const Detection& detection, const geometry_msgs::msg::Pose& in_planning_frame);

    /**
     * @brief Detaches @p object_id if attached, and removes it from the world.
     *
     * Detaching leaves a world copy at the hand, which nothing would ever move again.
     */
    void removeFromScene(const std::string& object_id);

    /**
     * @brief Drops the octomap, then exempts the arm's hand from @p touchables.
     *
     * The clear deletes the octomap's allowed-collision entries with it, so the exemption has to
     * follow it. Waits octomap_rebuild_wait_s for the sensor to put the visible surfaces back.
     */
    void clearOctomapKeeping(const ArmContext& arm, const std::vector<std::string>& touchables);

    /**
     * @brief Lets the arm's hand and wrist touch @p touchables, or stops letting it; logs a failure.
     *
     * Grasping is contact, and the planner cannot tell intended contact from a collision: the
     * octomap already holds the surface and the object as occupied, and the object's own
     * collision box is there to be avoided until the hand closes on it.
     *
     * @param include_links false exempts the touchables from each other only, leaving the hand and
     *        wrist checked. That is what a carried object needs against the voxels it casts.
     */
    void setHandContact(
        const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
        bool include_links = true);

    /**
     * @brief setHandContact() without the log.
     *
     * @return false if the arm has no hand group or a planning-scene service did not answer, in
     *         which case the exemption was neither applied nor restored.
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
    /// Each id's newest detection, stamped and framed like the array it came in.
    std::map<std::string, Detection> sightings_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    std::mutex                                                    joint_states_mutex_;
    sensor_msgs::msg::JointState                                  joint_states_;

    /// Kept alive for as long as the node is: dropping the handle removes the callback.
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
    /// Tunables every goal reads afresh, so a set_parameters between goals takes effect.
    std::map<std::string, double*> live_doubles_;
    std::map<std::string, int*>    live_ints_;

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// Null unless publish_markers is set.
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasp_plan_pub_;

    rclcpp::Client<g1_msgs::srv::GenerateGrasps>::SharedPtr         grasps_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr   get_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr                 clear_octomap_;

    rclcpp_action::Server<Pick>::SharedPtr          pick_server_;
    rclcpp_action::Server<Place>::SharedPtr         place_server_;
    rclcpp_action::Server<SetArmPosture>::SharedPtr posture_server_;

    std::string planning_frame_;

    // Defaults for the parameters of the same name; config/g1_manipulation_server.yaml documents
    // each one.
    double              object_timeout_s_{ 1.0 };
    double              approach_height_m_{ 0.22 };
    double              place_approach_height_m_{ 0.15 };
    double              grasp_depth_below_top_m_{ 0.020 };
    double              min_grip_height_m_{ 0.080 };
    double              settle_tolerance_m_{ 0.010 };
    int                 settle_attempts_{ 2 };
    double              max_grasp_offset_m_{ 0.020 };
    double              grasp_refresh_max_shift_m_{ 0.050 };
    double              reaim_clearance_m_{ 0.08 };
    double              settle_wait_s_{ 0.8 };
    double              lift_height_m_{ 0.15 };
    int                 lift_attempts_{ 3 };
    double              hand_close_s_{ 1.5 };
    double              hand_span_open_m_{ 0.124 };
    double              hand_span_closed_m_{ 0.058 };
    double              grip_preload_m_{ 0.002 };
    double              grip_start_margin_m_{ 0.010 };
    double              grip_max_width_m_{ 0.075 };
    double              grip_search_step_m_{ 0.002 };
    double              grip_search_beyond_m_{ 0.006 };
    double              grip_settled_speed_rad_s_{ 0.1 };
    double              grip_hold_bias_rad_{ 0.08 };
    bool                grip_check_enabled_{ true };
    double              grip_min_position_error_rad_{ 0.08 };
    double              grip_min_effort_nm_{ 0.03 };
    double              place_tolerance_m_{ 0.08 };
    double              place_footprint_margin_m_{ 0.02 };
    double              place_confirm_timeout_s_{ 4.0 };
    double              sighting_memory_s_{ 30.0 };
    double              octomap_rebuild_wait_s_{ 0.8 };
    double              velocity_scaling_{ 0.3 };
    double              planning_time_s_{ 5.0 };
    int                 planning_attempts_{ 5 };
    std::vector<double> grasp_rpy_;
    std::string         grasp_source_;
    double              grasp_timeout_s_{ 20.0 };
    double              min_grasp_score_{ 0.5 };
    int                 max_grasp_candidates_{ 20 };
    double              max_approach_tilt_rad_{ 0.0 };
    double              approach_standoff_m_{ 0.12 };
    double              ik_timeout_s_{ 0.05 };
    double              cartesian_step_m_{ 0.005 };
    /// Generator gripper frame to `<side>_hand_grasp_frame`; see grasp_filter.hpp.
    std::array<double, 6> graspgen_offset_{};

    /// Where the last close was commanded to, as a fraction of `open` to `closed`. The grip check
    /// measures against this, since a close onto a measured width stops short of `closed`.
    double last_close_fraction_{ 1.0 };

    /// One goal at a time across all three servers: MoveGroupInterface is not thread-safe, and two
    /// goals would share one arm_trajectory_controller.
    std::atomic<bool> busy_{ false };
    std::atomic<int>  goals_running_{ 0 };
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
