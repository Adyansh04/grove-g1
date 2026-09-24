/**
 * @file g1_base_approach_node.cpp
 * @brief Walks the base into arm's reach of a measured object, and backs it out again.
 *
 * Nav2 parks within 0.5 m of its goal and the arm's reach window is about 0.11 m wide, so this
 * closes the gap against the measured object. It writes /cmd_vel directly, as Nav2 does; the
 * mission tree never runs the two together. The control law is in approach_planner.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <g1_msgs/action/approach_object.hpp>
#include <g1_msgs/action/retreat.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <stdexcept>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vector>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_locomotion/approach_planner.hpp"

namespace g1_locomotion
{

/// A retreat only has to clear a surface; anything longer is a malformed goal.
constexpr double kMaxRetreatDistanceM = 2.0;

using ApproachObject     = g1_msgs::action::ApproachObject;
using Retreat            = g1_msgs::action::Retreat;
using GoalHandleApproach = rclcpp_action::ServerGoalHandle<ApproachObject>;
using GoalHandleRetreat  = rclcpp_action::ServerGoalHandle<Retreat>;

namespace
{

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

/// In steady_clock's own duration type, so deadlines compare without templates.
std::chrono::steady_clock::time_point deadlineIn(double seconds)
{
    return std::chrono::steady_clock::now() +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double>(seconds));
}

}  // namespace

class BaseApproachNode : public rclcpp::Node
{
public:
    BaseApproachNode()
      : rclcpp::Node("g1_base_approach")
      , tf_buffer_(get_clock())
      , tf_listener_(tf_buffer_)
    {
        cmd_topic_         = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        base_frame_        = declare_parameter<std::string>("base_frame", "base_footprint");
        object_timeout_ms_ = declare_parameter<double>("object_timeout_ms", 1500.0);

        const GaitLimits g;
        gait_.min_speed_x_mps  = declare_parameter<double>("min_speed_x_mps", g.min_speed_x_mps);
        gait_.min_speed_y_mps  = declare_parameter<double>("min_speed_y_mps", g.min_speed_y_mps);
        gait_.max_speed_x_mps  = declare_parameter<double>("max_speed_x_mps", g.max_speed_x_mps);
        gait_.max_speed_y_mps  = declare_parameter<double>("max_speed_y_mps", g.max_speed_y_mps);
        gait_.max_yaw_rate_rps = declare_parameter<double>("max_yaw_rate_rps", g.max_yaw_rate_rps);
        gait_.speed_per_m      = declare_parameter<double>("speed_per_m", g.speed_per_m);
        gait_.yaw_rate_per_rad = declare_parameter<double>("yaw_rate_per_rad", g.yaw_rate_per_rad);

        retreat_speed_mps_ = declare_parameter<double>("retreat_speed_mps", 0.30);
        settle_s_          = declare_parameter<double>("settle_s", 1.0);
        cmd_rate_hz_       = declare_parameter<double>("cmd_rate_hz", 20.0);

        const ApproachLimits d;
        limits_.target_x_m = declare_parameter<double>("target_x_m", d.target_x_m);
        limits_.target_y_m = declare_parameter<double>("target_y_m", d.target_y_m);
        limits_.forward_tolerance_m =
            declare_parameter<double>("forward_tolerance_m", d.forward_tolerance_m);
        limits_.lateral_tolerance_m =
            declare_parameter<double>("lateral_tolerance_m", d.lateral_tolerance_m);
        limits_.min_forward_m = declare_parameter<double>("min_forward_m", d.min_forward_m);
        limits_.heading_tolerance_rad =
            declare_parameter<double>("heading_tolerance_rad", d.heading_tolerance_rad);

        standoff_ids_ = declare_parameter<std::vector<std::string>>(
            "standoff_object_ids",
            std::vector<std::string>{});
        standoff_target_x_ =
            declare_parameter<std::vector<double>>("standoff_target_x_m", std::vector<double>{});
        if (standoff_ids_.size() != standoff_target_x_.size())
        {
            throw std::runtime_error(
                "g1_base_approach: standoff_object_ids and standoff_target_x_m must be the same "
                "length");
        }

        lookup_grace_s_    = declare_parameter<double>("lookup_grace_s", 3.0);
        default_timeout_s_ = declare_parameter<double>("default_timeout_s", 900.0);

        if (!limitsAreUsable(limits_))
        {
            throw std::runtime_error(
                "g1_base_approach: the configured reach window is not one the planner can aim "
                "at; check target_x_m against min_forward_m and the tolerances");
        }
        if (!gaitLimitsAreUsable(gait_))
        {
            throw std::runtime_error(
                "g1_base_approach: the configured speeds are unusable; every floor must be "
                "positive and no greater than its ceiling");
        }

        cmd_pub_     = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 1);
        objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
            "objects",
            rclcpp::SensorDataQoS(),
            [this](const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg) {
                const std::lock_guard<std::mutex> lock(objects_mutex_);
                for (const auto& detection : msg->detections)
                {
                    if (detection.results.empty())
                    {
                        continue;
                    }
                    geometry_msgs::msg::PointStamped& seen =
                        sightings_[detection.results.front().hypothesis.class_id];
                    seen.header = msg->header;
                    seen.point  = detection.results.front().pose.pose.position;
                }
                // Track ids come and go, so aged-out sightings are dropped.
                const rclcpp::Time newest(msg->header.stamp);
                std::erase_if(sightings_, [&](const auto& entry) {
                    return (newest - rclcpp::Time(entry.second.header.stamp)).seconds() * 1e3 >
                           object_timeout_ms_;
                });
            });

        approach_server_ = rclcpp_action::create_server<ApproachObject>(
            this,
            "~/approach_object",
            [this](const rclcpp_action::GoalUUID&, const ApproachObject::Goal::ConstSharedPtr&) {
                return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                                   rclcpp_action::GoalResponse::REJECT;
            },
            [](const std::shared_ptr<GoalHandleApproach>&) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleApproach>& handle) {
                std::thread{ [this, handle] {
                    runGuarded([&] { runApproach(handle); }, handle);
                } }.detach();
            });

        retreat_server_ = rclcpp_action::create_server<Retreat>(
            this,
            "~/retreat",
            [this](const rclcpp_action::GoalUUID&, const Retreat::Goal::ConstSharedPtr& goal) {
                if (goal->distance_m <= 0.0 || goal->distance_m > kMaxRetreatDistanceM)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "rejecting a retreat of %.3f m: must be within (0, %.1f]",
                        goal->distance_m,
                        kMaxRetreatDistanceM);
                    return rclcpp_action::GoalResponse::REJECT;
                }
                return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                                   rclcpp_action::GoalResponse::REJECT;
            },
            [](const std::shared_ptr<GoalHandleRetreat>&) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleRetreat>& handle) {
                std::thread{ [this, handle] {
                    runGuarded([&] { runRetreat(handle); }, handle);
                } }.detach();
            });

        RCLCPP_INFO(
            get_logger(),
            "base approach ready: object wanted at (%.3f, %.3f) in %s, window +/-%.3f forward "
            "and +/-%.3f lateral, publishing on %s",
            limits_.target_x_m,
            limits_.target_y_m,
            base_frame_.c_str(),
            limits_.forward_tolerance_m,
            limits_.lateral_tolerance_m,
            cmd_topic_.c_str());
    }

    ~BaseApproachNode() override
    {
        // Goal threads are detached and use this node's members: stop them, wait, then leave
        // /cmd_vel at zero rather than at the last command.
        stopping_.store(true);
        while (goals_running_.load() > 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        try
        {
            if (cmd_pub_)
            {
                publish(0.0, 0.0, 0.0);
            }
        }
        catch (const std::exception& e)
        {
            // The C logger: this runs during destruction, which must not throw.
            RCUTILS_LOG_ERROR_NAMED(
                "g1_base_approach",
                "could not stop the base on shutdown: %s",
                e.what());
        }
    }

private:
    /// One goal at a time across BOTH actions: an approach and a retreat running together would
    /// interleave a forward and a reverse command on the same channel.
    bool acquire()
    {
        bool expected = false;
        if (!busy_.compare_exchange_strong(expected, true))
        {
            RCLCPP_WARN(get_logger(), "rejecting a goal: another one is already running");
            return false;
        }
        return true;
    }

    /// Runs one goal body: releases the busy flag, balances the running count, and turns an
    /// escaping exception into a stopped base and an aborted goal.
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
            RCLCPP_ERROR(get_logger(), "goal threw, stopping the base: %s", e.what());
            publish(0.0, 0.0, 0.0);
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

    std::chrono::duration<double> tickPeriod() const
    {
        return std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
    }

    void publish(double vx, double vy, double yaw_rate)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = vx;
        twist.linear.y  = vy;
        twist.angular.z = yaw_rate;
        cmd_pub_->publish(twist);
    }

    /// Holds zero while the gait finishes its stride, so the next measurement is of a stopped robot.
    void settle()
    {
        const auto until = std::chrono::steady_clock::now() +
                           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(settle_s_));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
        {
            publish(0.0, 0.0, 0.0);
            std::this_thread::sleep_for(tickPeriod());
        }
    }

    /// The base's pose in odom, or nothing if TF has not caught up.
    std::optional<geometry_msgs::msg::PoseStamped> basePose()
    {
        try
        {
            const auto tf = tf_buffer_.lookupTransform(
                "odom",
                base_frame_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.5));
            geometry_msgs::msg::PoseStamped pose;
            pose.header           = tf.header;
            pose.pose.position.x  = tf.transform.translation.x;
            pose.pose.position.y  = tf.transform.translation.y;
            pose.pose.position.z  = tf.transform.translation.z;
            pose.pose.orientation = tf.transform.rotation;
            return pose;
        }
        catch (const tf2::TransformException& e)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "no odom to %s: %s",
                base_frame_.c_str(),
                e.what());
            return std::nullopt;
        }
    }

    /// The named object's position in the base frame, or nothing if it is missing or stale.
    std::optional<geometry_msgs::msg::PointStamped> objectInBase(const std::string& object_id)
    {
        // The last sighting, not the newest message: a detector pass that misses a standing prop
        // says nothing about where it went.
        geometry_msgs::msg::PointStamped seen;
        {
            const std::lock_guard<std::mutex> lock(objects_mutex_);
            const auto                        found = sightings_.find(object_id);
            if (found == sightings_.end())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "'%s' has not been reported on /objects",
                    object_id.c_str());
                return std::nullopt;
            }
            seen = found->second;
        }

        const double age_ms = (now() - rclcpp::Time(seen.header.stamp)).seconds() * 1e3;
        if (age_ms > object_timeout_ms_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "'%s' was last seen %.0f ms ago",
                object_id.c_str(),
                age_ms);
            return std::nullopt;
        }

        try
        {
            // Latest transform: a standing object's odom pose stays valid, and what matters is
            // where it is from the base now.
            seen.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
            return tf_buffer_.transform(seen, base_frame_, tf2::durationFromSec(0.5));
        }
        catch (const tf2::TransformException& e)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "cannot transform the object pose: %s",
                e.what());
            return std::nullopt;
        }
    }

    /// The reach window for this goal: mirrored for the left arm, widened for objects that have
    /// to be reached over rather than onto.
    ApproachLimits limitsFor(const ApproachObject::Goal& goal) const
    {
        ApproachLimits limits = limits_;
        // The window mirrors with the arm, exactly as the grasp offset does: the right hand
        // works to the pelvis's -y, the left to its +y.
        if (goal.arm == ApproachObject::Goal::ARM_LEFT)
        {
            limits.target_y_m = -limits.target_y_m;
        }
        for (std::size_t i = 0; i < standoff_ids_.size(); ++i)
        {
            if (standoff_ids_[i] == goal.object_id)
            {
                limits.target_x_m = standoff_target_x_[i];
                RCLCPP_INFO(
                    get_logger(),
                    "'%s' is approached to %.3f rather than the default %.3f",
                    goal.object_id.c_str(),
                    limits.target_x_m,
                    limits_.target_x_m);
                break;
            }
        }
        return limits;
    }

    void runApproach(const std::shared_ptr<GoalHandleApproach>& handle)
    {
        const auto goal   = handle->get_goal();
        auto       result = std::make_shared<ApproachObject::Result>();

        if (goal->arm != ApproachObject::Goal::ARM_LEFT &&
            goal->arm != ApproachObject::Goal::ARM_RIGHT)
        {
            result->message = "locating: arm must be 'left' or 'right', got '" + goal->arm + "'";
            handle->abort(result);
            return;
        }
        const ApproachLimits limits = limitsFor(*goal);

        const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
        const auto   deadline  = deadlineIn(timeout_s);
        auto         feedback  = std::make_shared<ApproachObject::Feedback>();

        // Fixed up front: chasing the object's bearing would never arrive square to anything.
        const auto start_pose = basePose();
        if (!start_pose)
        {
            result->message = "locating: no base pose";
            handle->abort(result);
            return;
        }
        const double working_yaw = goal->use_current_heading ?
                                       tf2::getYaw(start_pose->pose.orientation) :
                                       goal->working_yaw;

        // Feedback at about 2 Hz, not every tick.
        const int  feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
        int        tick           = 0;
        auto       last_measured  = std::chrono::steady_clock::now();
        const auto blind_for      = [&last_measured] {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - last_measured)
                .count();
        };

        while (rclcpp::ok())
        {
            if (handle->is_canceling())
            {
                publish(0.0, 0.0, 0.0);
                result->message = "closing: cancelled";
                handle->canceled(result);
                return;
            }
            if (std::chrono::steady_clock::now() > deadline)
            {
                publish(0.0, 0.0, 0.0);
                result->message = "closing: gave up after " + std::to_string(timeout_s) + " s";
                handle->abort(result);
                return;
            }

            const auto object = objectInBase(goal->object_id);
            const auto here   = basePose();
            if (!object || !here)
            {
                // Stand still without a measurement, for up to lookup_grace_s: TF and detections
                // both drop out briefly.
                publish(0.0, 0.0, 0.0);
                if (blind_for() > lookup_grace_s_)
                {
                    result->message = "locating: no fresh pose for '" + goal->object_id +
                                      "' on /objects after " + std::to_string(lookup_grace_s_) +
                                      " s";
                    handle->abort(result);
                    return;
                }
                std::this_thread::sleep_for(tickPeriod());
                continue;
            }
            last_measured = std::chrono::steady_clock::now();

            // Judged in the base frame, which is what the arm's reach is defined in.
            const double heading_error = wrap(working_yaw - tf2::getYaw(here->pose.orientation));
            const auto   command =
                planApproach(object->point.x, object->point.y, heading_error, limits, gait_);

            result->final_x_m         = object->point.x;
            result->final_y_m         = object->point.y;
            feedback->forward_error_m = command.forward_error_m;
            feedback->lateral_error_m = command.lateral_error_m;

            if (command.state == ApproachState::kOvershot)
            {
                publish(0.0, 0.0, 0.0);
                result->message = "closing: the object is " + std::to_string(object->point.x) +
                                  " m ahead, under the robot's own footprint; re-stage "
                                  "through Nav2";
                handle->abort(result);
                return;
            }
            if (command.state == ApproachState::kInvalid)
            {
                publish(0.0, 0.0, 0.0);
                result->message = "locating: the configured reach window is unusable";
                handle->abort(result);
                return;
            }

            if (command.state == ApproachState::kArrived)
            {
                // Stop, let the stride finish, then re-judge: the robot coasts, and a coast out of
                // the window has to be closed again.
                feedback->phase = ApproachObject::Feedback::PHASE_VERIFYING;
                handle->publish_feedback(feedback);
                settle();

                const auto settled = objectInBase(goal->object_id);
                if (!settled)
                {
                    continue;
                }
                const auto verdict =
                    planApproach(settled->point.x, settled->point.y, 0.0, limits, gait_);
                result->final_x_m = settled->point.x;
                result->final_y_m = settled->point.y;
                if (verdict.state != ApproachState::kArrived)
                {
                    RCLCPP_INFO(
                        get_logger(),
                        "coasted back out of the window to (%.3f, %.3f); closing again",
                        settled->point.x,
                        settled->point.y);
                    continue;
                }

                result->success = true;
                result->message = "the object is at (" + std::to_string(settled->point.x) + ", " +
                                  std::to_string(settled->point.y) + ")";
                RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
                handle->succeed(result);
                return;
            }

            if (tick % feedback_every == 0)
            {
                feedback->phase = ApproachObject::Feedback::PHASE_CLOSING;
                handle->publish_feedback(feedback);
                RCLCPP_INFO(
                    get_logger(),
                    "closing: forward %+.3f, lateral %+.3f, heading %+.1f deg -> "
                    "(%.2f, %.2f, %.2f)",
                    command.forward_error_m,
                    command.lateral_error_m,
                    heading_error * 180.0 / M_PI,
                    command.vx_mps,
                    command.vy_mps,
                    command.yaw_rate_rps);
            }
            ++tick;

            publish(command.vx_mps, command.vy_mps, command.yaw_rate_rps);
            std::this_thread::sleep_for(tickPeriod());
        }

        publish(0.0, 0.0, 0.0);
        result->message = "closing: shutting down";
        handle->abort(result);
    }

    void runRetreat(const std::shared_ptr<GoalHandleRetreat>& handle)
    {
        const auto goal     = handle->get_goal();
        auto       result   = std::make_shared<Retreat::Result>();
        auto       feedback = std::make_shared<Retreat::Feedback>();

        const auto start = basePose();
        if (!start)
        {
            result->message = "backing_off: no base pose to retreat from";
            handle->abort(result);
            return;
        }

        const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
        const auto   deadline  = deadlineIn(timeout_s);

        // Straight back only: turning beside a surface swings the robot and what it holds across
        // it. The navigation goal that follows does the turning.
        feedback->phase = Retreat::Feedback::PHASE_BACKING_OFF;
        handle->publish_feedback(feedback);

        // nullopt on a TF outage, not 0.0, or the loop would reverse blind until the deadline.
        const auto travelled = [&]() -> std::optional<double> {
            const auto here = basePose();
            if (!here)
            {
                return std::nullopt;
            }
            return std::hypot(
                here->pose.position.x - start->pose.position.x,
                here->pose.position.y - start->pose.position.y);
        };
        double     last_travelled = 0.0;
        const auto blind_until =
            [&,
             grace = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                 std::chrono::duration<double>(lookup_grace_s_))] {
                return std::chrono::steady_clock::now() + grace;
            };
        auto blind_deadline = blind_until();

        const int feedback_every = std::max(1, static_cast<int>(cmd_rate_hz_ / 2.0));
        int       tick           = 0;
        while (rclcpp::ok() && !stopping_.load() && last_travelled < goal->distance_m &&
               std::chrono::steady_clock::now() < deadline)
        {
            if (handle->is_canceling())
            {
                publish(0.0, 0.0, 0.0);
                result->travelled_m = last_travelled;
                result->message     = "backing_off: cancelled";
                handle->canceled(result);
                return;
            }

            if (const auto measured = travelled())
            {
                last_travelled = *measured;
                blind_deadline = blind_until();
            }
            else
            {
                // As in the approach: stop, and give TF lookup_grace_s to come back.
                publish(0.0, 0.0, 0.0);
                if (std::chrono::steady_clock::now() > blind_deadline)
                {
                    result->success     = false;
                    result->travelled_m = last_travelled;
                    result->message     = "backing_off: lost the base pose";
                    handle->abort(result);
                    return;
                }
                std::this_thread::sleep_for(tickPeriod());
                continue;
            }

            publish(-retreat_speed_mps_, 0.0, 0.0);
            if (tick++ % feedback_every == 0)
            {
                feedback->travelled_m = last_travelled;
                handle->publish_feedback(feedback);
            }
            std::this_thread::sleep_for(tickPeriod());
        }
        settle();

        // The last good reading if TF is still down, or a completed retreat reads as 0 m.
        const double backed = travelled().value_or(last_travelled);
        result->travelled_m = backed;
        result->success     = backed >= goal->distance_m;
        result->message     = result->success ? "reversed " + std::to_string(backed) + " m clear" :
                                                "backing_off: only made " + std::to_string(backed) +
                                                " m of " + std::to_string(goal->distance_m);
        RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
        if (result->success)
        {
            handle->succeed(result);
        }
        else
        {
            handle->abort(result);
        }
    }

    std::string cmd_topic_;
    std::string base_frame_;
    double      object_timeout_ms_ = 1500.0;
    double      retreat_speed_mps_ = 0.30;
    double      settle_s_          = 1.0;
    double      cmd_rate_hz_       = 20.0;
    double      lookup_grace_s_    = 3.0;
    double      default_timeout_s_ = 900.0;

    ApproachLimits           limits_;
    GaitLimits               gait_;
    std::vector<std::string> standoff_ids_;
    std::vector<double>      standoff_target_x_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    rclcpp_action::Server<ApproachObject>::SharedPtr                    approach_server_;
    rclcpp_action::Server<Retreat>::SharedPtr                           retreat_server_;

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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        // Multi-threaded, so /objects, TF and cancels keep flowing while a goal loop runs.
        rclcpp::executors::MultiThreadedExecutor executor;
        auto node = std::make_shared<g1_locomotion::BaseApproachNode>();
        executor.add_node(node);
        executor.spin();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("g1_base_approach"), "fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
