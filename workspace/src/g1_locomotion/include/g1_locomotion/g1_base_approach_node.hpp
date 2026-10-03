/**
 * @file g1_base_approach_node.hpp
 * @brief Walks the base into arm's reach of a measured object, and backs it out again.
 *
 * Nav2 parks within 0.5 m of its goal and the arm's reach window is about 0.11 m wide, so this
 * closes the gap against the measured object. It writes /cmd_vel directly, as Nav2 does; the
 * mission tree never runs the two together. The control law is in approach_planner.
 */

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <chrono>
#include <g1_msgs/action/approach_object.hpp>
#include <g1_msgs/action/retreat.hpp>
#include <g1_msgs/action/step_clear.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_locomotion/approach_planner.hpp"
#include "g1_locomotion/step_clear.hpp"

namespace g1_locomotion
{

using ApproachObject      = g1_msgs::action::ApproachObject;
using Retreat             = g1_msgs::action::Retreat;
using GoalHandleApproach  = rclcpp_action::ServerGoalHandle<ApproachObject>;
using GoalHandleRetreat   = rclcpp_action::ServerGoalHandle<Retreat>;
using StepClear           = g1_msgs::action::StepClear;
using GoalHandleStepClear = rclcpp_action::ServerGoalHandle<StepClear>;

class BaseApproachNode : public rclcpp::Node
{
public:
    BaseApproachNode();

    ~BaseApproachNode() override;

private:
    /// One goal at a time across every action: an approach and a retreat running together would
    /// interleave a forward and a reverse command on the same channel.
    bool acquire();

    /// Runs one goal body: releases the busy flag, balances the running count its caller took
    /// before the thread started, and turns an escaping exception into a stopped base and an
    /// aborted goal.
    template <typename ActionT, typename Body>
    void
    runGuarded(Body&& body, const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>& handle);

    std::chrono::duration<double> tickPeriod() const;

    void publish(double vx, double vy, double yaw_rate);

    /// Holds zero while the gait finishes its stride, so the next measurement is of a stopped robot.
    void settle();

    /// Holds a velocity for `seconds`, then stops.
    void pulse(double vx, double vy, double seconds);

    /// The base's pose in @p frame, or nothing if TF has not caught up.
    std::optional<geometry_msgs::msg::PoseStamped> basePose(const std::string& frame = "odom");

    /// The named object's position in the base frame, or nothing if it is missing or stale.
    std::optional<geometry_msgs::msg::PointStamped> objectInBase(const std::string& object_id);

    /// The reach window for this goal: mirrored for the left arm, widened for objects that have
    /// to be reached over rather than onto.
    ApproachLimits limitsFor(const ApproachObject::Goal& goal) const;

    void runApproach(const std::shared_ptr<GoalHandleApproach>& handle);

    void runRetreat(const std::shared_ptr<GoalHandleRetreat>& handle);

    /// What surrounds the base: lethal costmap cells and the base pose, in the costmap's frame.
    struct Surroundings
    {
        std::vector<ObstaclePoint> obstacles;
        double                     x          = 0.0;
        double                     y          = 0.0;
        double                     yaw        = 0.0;
        double                     resolution = 0.0;
    };

    std::optional<Surroundings> surroundings();

    void runStepClear(const std::shared_ptr<GoalHandleStepClear>& handle);

    /// @p a wrapped to [-pi, pi].
    static double wrap(double a);

    /// In steady_clock's own duration type, so deadlines compare without templates.
    static std::chrono::steady_clock::time_point deadlineIn(double seconds);

    std::string cmd_topic_;
    std::string base_frame_;
    double      object_timeout_ms_ = 1500.0;
    double      retreat_speed_mps_ = 0.30;
    double      settle_s_          = 1.0;
    int         max_nudges_        = 12;
    double      cmd_rate_hz_       = 20.0;
    double      lookup_grace_s_    = 3.0;
    double      default_timeout_s_ = 900.0;

    StepClearLimits step_limits_;
    double          step_speed_mps_             = 0.30;
    double          step_heading_tolerance_rad_ = 0.15;
    double          step_timeout_s_             = 20.0;
    std::string     costmap_topic_;

    ApproachLimits           limits_;
    GaitLimits               gait_;
    std::vector<std::string> standoff_ids_;
    std::vector<double>      standoff_target_x_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    rclcpp_action::Server<ApproachObject>::SharedPtr                    approach_server_;
    rclcpp_action::Server<Retreat>::SharedPtr                           retreat_server_;
    rclcpp_action::Server<StepClear>::SharedPtr                         step_clear_server_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr       costmap_sub_;

    std::mutex                                   costmap_mutex_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;

    std::mutex objects_mutex_;
    /// Each object id's newest reported position, stamped when it was seen.
    std::map<std::string, geometry_msgs::msg::PointStamped> sightings_;
    tf2_ros::Buffer                                         tf_buffer_;
    tf2_ros::TransformListener                              tf_listener_;

    /// One goal across both actions: two would be two writers on /cmd_vel.
    std::atomic<bool> busy_{ false };
    /// Set by the destructor so a running goal leaves its loop.
    std::atomic<bool> stopping_{ false };
    std::atomic<int>  goals_running_{ 0 };
};

}  // namespace g1_locomotion
