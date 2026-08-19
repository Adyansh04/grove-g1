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

#include <atomic>
#include <g1_msgs/action/pick.hpp>
#include <g1_msgs/action/place.hpp>
#include <g1_msgs/action/set_arm_posture.hpp>
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
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

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
     *        face, not at its centre, or the fingers close through whatever it stands on.
     */
    geometry_msgs::msg::Pose graspFrameGoal(
        const geometry_msgs::msg::Pose& object_pose, double object_height_m,
        const ArmContext& arm) const;

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
     * @brief setHandContact() without the logging, for a caller that must see the failure.
     *
     * @return false if the planning-scene service did not answer, in which case the exemption
     *         was neither applied nor restored. A silently failed restore leaves the scene
     *         blinded, and a silently failed apply reads downstream as an unreachable pose.
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

    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr   get_scene_;
    rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_scene_;

    rclcpp_action::Server<Pick>::SharedPtr          pick_server_;
    rclcpp_action::Server<Place>::SharedPtr         place_server_;
    rclcpp_action::Server<SetArmPosture>::SharedPtr posture_server_;

    std::string planning_frame_;
    double      object_timeout_s_{ 1.0 };
    double      grasp_height_above_top_m_{ 0.010 };
    double      place_tolerance_m_{ 0.08 };
    double      approach_height_m_{ 0.22 };
    double      lift_height_m_{ 0.15 };
    double      velocity_scaling_{ 0.3 };
    double      planning_time_s_{ 5.0 };
    // How the hand is held at the grasp. Where it grips is the grasp frame in the URDF; only the
    // orientation is a choice, and it is the one thing that depends on the surface rather than
    // on the hand.
    std::vector<double> grasp_rpy_;

    /// One goal at a time across ALL THREE servers. MoveGroupInterface is not thread-safe and
    /// carries mutable start-state and plan state, and two goals on different groups still drive
    /// overlapping joints through the one arm_trajectory_controller, so the second trajectory
    /// preempts the first mid-motion, possibly with an object in the hand.
    std::atomic<bool> busy_{ false };
    std::atomic<int>  goals_running_{ 0 };
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
