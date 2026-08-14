#ifndef G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
#define G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_

/**
 * @file g1_manipulation_server_node.hpp
 * @brief Pick, place and named-posture skills, served as actions over MoveIt.
 *
 * Adds no command path: every motion goes out through the same `move_group` the RViz panel
 * uses, onto the controllers that already own `rt/arm_sdk` and the hand topics. So the
 * one-writer rule in the control-mode rules is unaffected by this node existing.
 *
 * It also takes no control authority of its own. The arm and hands must already be acquired
 * (g1_bringup's activate_arm) before a goal will execute, and releasing them is the caller's
 * job -- for the mission that is g1_orchestration's executor, which brackets the whole run.
 * A skill that acquired authority per goal would hand it back mid-mission and drop whatever
 * the hand was holding.
 */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

/// The MoveIt groups and links one arm brings, resolved from the `arm` field of a goal.
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

/// False if `arm` is neither "left" nor "right", leaving `out` untouched.
bool resolveArm(const std::string& arm, ArmContext& out);

class G1ManipulationServer : public rclcpp::Node
{
public:
    explicit G1ManipulationServer(const rclcpp::NodeOptions& options);

    /**
     * @brief Builds the MoveGroupInterfaces.
     *
     * Separate from the constructor because MoveGroupInterface blocks until it has the robot
     * description and the current state, which only arrive once something is spinning this
     * node -- constructing one from inside the constructor deadlocks.
     */
    void initialize();

private:
    using Pick          = g1_msgs::action::Pick;
    using Place         = g1_msgs::action::Place;
    using SetArmPosture = g1_msgs::action::SetArmPosture;
    using MoveGroup     = moveit::planning_interface::MoveGroupInterface;

    template <typename ActionT>
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    void executePick(const std::shared_ptr<GoalHandle<Pick>>& goal_handle);
    void executePlace(const std::shared_ptr<GoalHandle<Place>>& goal_handle);
    void executeSetArmPosture(const std::shared_ptr<GoalHandle<SetArmPosture>>& goal_handle);

    /// Latest pose for `object_id`, or nullopt if it is unknown or older than the timeout.
    std::optional<vision_msgs::msg::Detection3D> lookUpObject(const std::string& object_id);
    void onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg);

    /// Into the planning frame, or nullopt with the reason logged. Everything a goal carries
    /// goes through here: /objects is in odom, the planner works in pelvis, and the two differ
    /// by wherever the robot is standing.
    std::optional<geometry_msgs::msg::Pose>
    toPlanningFrame(const geometry_msgs::msg::Pose& pose, const std::string& frame_id);

    /// The pose to give the arm's grasp frame so the object ends up at `object_pose`. Position
    /// passes straight through -- the grasp frame IS where the object goes -- and only the
    /// orientation is chosen here. Handed: the two hands hold at mirrored rolls.
    ///
    /// `object_height_m` is the object's FULL height: the grasp is taken just under its top
    /// face, not at its centre, or the fingers close through whatever the object is standing on.
    geometry_msgs::msg::Pose graspFrameGoal(
        const geometry_msgs::msg::Pose& object_pose, double object_height_m,
        const ArmContext& arm) const;

    /// Plans and executes so that `link` reaches `pose`. False on either failure, logged.
    bool moveTo(
        MoveGroup& group, const geometry_msgs::msg::Pose& pose, const std::string& link,
        const std::string& what);
    bool moveToNamed(MoveGroup& group, const std::string& named_target);

    /// Puts the object into the planning scene at its measured pose, already transformed into
    /// the planning frame, so plans route around it. Returns what it built: the world copy is
    /// removed before the grasp, and the attached body then has to carry the same geometry.
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
     * Scoped as narrowly as the problem allows -- the hand and the wrist that carries it, one
     * arm, one skill -- and always restored, including on every failure path, so the arm is
     * never left planning against a permanently blinded scene.
     */
    /// @param include_links  false exempts the touchables from each other only, leaving the hand
    ///        and wrist collision-checked -- what carrying an object over a surface wants.
    bool allowHandContact(
        const ArmContext& arm, const std::vector<std::string>& touchables, bool allowed,
        bool include_links = true);

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
    // How the hand is held at the grasp. The WHERE is the grasp frame in the URDF; only the
    // orientation is a choice, and it is the one thing that depends on the surface rather than
    // on the hand.
    std::vector<double> grasp_rpy_;
};

}  // namespace g1_manipulation

#endif  // G1_MANIPULATION__G1_MANIPULATION_SERVER_NODE_HPP_
