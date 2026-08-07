/**
 * @file g1_base_approach_node.cpp
 * @brief Walks the base into arm's reach of a measured object, and backs it out again.
 *
 * The missing step between navigation and manipulation. Nav2 parks the robot within 0.5 m of a
 * goal it chose from a map; the arm reaches 0.28 to 0.33 m and its usable window is about
 * 0.12 m wide. Nothing bridges that today, which is why navigate-then-pick does not work
 * (docs/notes/m9-grasp-reachability.md).
 *
 * Lives in g1_locomotion, not in g1_manipulation, on one principle: everything that writes a
 * velocity command belongs to the package that owns the velocity path. A manipulation package
 * publishing into locomotion's channel is the shape of bug docs/CONTROL_MODES.md exists to
 * prevent, even when the topic itself is harmless.
 *
 * Publishes on its own topic rather than writing /cmd_vel alongside Nav2. g1_gait_shaper
 * subscribes to both and gives this one priority while it is publishing, so ownership of the
 * velocity channel is declared rather than left to the behaviour tree's sequencing.
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <g1_msgs/action/approach_object.hpp>
#include <g1_msgs/action/retreat.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <utility>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "g1_locomotion/approach_planner.hpp"

namespace g1_locomotion
{

using ApproachObject     = g1_msgs::action::ApproachObject;
using Retreat            = g1_msgs::action::Retreat;
using GoalHandleApproach = rclcpp_action::ServerGoalHandle<ApproachObject>;
using GoalHandleRetreat  = rclcpp_action::ServerGoalHandle<Retreat>;

namespace
{

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

/// steady_clock's own rep, not the double-based one `now() + duration<double>` produces --
/// otherwise every function that takes a deadline needs its own template parameter.
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
        cmd_topic_         = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_approach");
        base_frame_        = declare_parameter<std::string>("base_frame", "base_footprint");
        object_timeout_ms_ = declare_parameter<double>("object_timeout_ms", 1500.0);

        // Both must clear g1_gait_shaper's engage thresholds (0.45 forward, 1.20 yaw) or the
        // shaper zeroes them and the robot stands still while this node reports it is walking.
        pulse_vx_   = declare_parameter<double>("pulse_vx", 0.45);
        pulse_vyaw_ = declare_parameter<double>("pulse_vyaw", 1.30);
        // 0.30 s is the pulse measured to be repeatable to a millimetre on a settled robot.
        step_pulse_s_     = declare_parameter<double>("step_pulse_s", 0.30);
        turn_pulse_s_     = declare_parameter<double>("turn_pulse_s", 0.22);
        min_turn_pulse_s_ = declare_parameter<double>("min_turn_pulse_s", 0.12);
        // The gait keeps stepping after the command stops. Measuring before it settles reports
        // the command plus whatever of the stride was still in flight.
        settle_s_    = declare_parameter<double>("settle_s", 2.5);
        cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 20.0);

        limits_.target_range_m        = declare_parameter<double>("target_range_m", 0.344);
        limits_.target_bearing_rad    = declare_parameter<double>("target_bearing_rad", -0.620);
        limits_.range_tolerance_m     = declare_parameter<double>("range_tolerance_m", 0.055);
        limits_.bearing_tolerance_rad = declare_parameter<double>("bearing_tolerance_rad", 0.200);
        limits_.min_range_m           = declare_parameter<double>("min_range_m", 0.215);
        limits_.pulse_advance_m       = declare_parameter<double>("pulse_advance_m", 0.35);
        limits_.max_oblique_rad       = declare_parameter<double>("max_oblique_rad", 1.309);

        max_pulses_        = declare_parameter<int>("max_pulses", 30);
        default_timeout_s_ = declare_parameter<double>("default_timeout_s", 180.0);

        if (!limitsAreUsable(limits_))
        {
            throw std::runtime_error(
                "g1_base_approach: the configured reach window is not one the planner can aim "
                "at; check target_range_m against min_range_m and the tolerances");
        }

        cmd_pub_     = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 1);
        objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
            "objects",
            rclcpp::SensorDataQoS(),
            [this](vision_msgs::msg::Detection3DArray::SharedPtr msg) {
                const std::lock_guard<std::mutex> lock(objects_mutex_);
                objects_ = std::move(msg);
            });

        approach_server_ = rclcpp_action::create_server<ApproachObject>(
            this,
            "~/approach_object",
            [](const rclcpp_action::GoalUUID&, ApproachObject::Goal::ConstSharedPtr) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<GoalHandleApproach>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleApproach> handle) {
                std::thread{ [this, handle] { runApproach(handle); } }.detach();
            });

        retreat_server_ = rclcpp_action::create_server<Retreat>(
            this,
            "~/retreat",
            [](const rclcpp_action::GoalUUID&, Retreat::Goal::ConstSharedPtr) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<GoalHandleRetreat>) {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleRetreat> handle) {
                std::thread{ [this, handle] { runRetreat(handle); } }.detach();
            });

        RCLCPP_INFO(
            get_logger(),
            "base approach ready: target %.3f m at %.1f deg, window +/-%.3f m, publishing on %s",
            limits_.target_range_m,
            limits_.target_bearing_rad * 180.0 / M_PI,
            limits_.range_tolerance_m,
            cmd_topic_.c_str());
    }

private:
    /// Command a velocity for `duration_s`, then hold zero while the gait settles.
    ///
    /// Zeros are published rather than merely stopping: the shaper hands the channel back to
    /// Nav2 when this source goes quiet, so an idle gap mid-skill would surrender priority.
    void pulse(double vx, double vyaw, double duration_s)
    {
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));

        geometry_msgs::msg::Twist moving;
        moving.linear.x       = vx;
        moving.angular.z      = vyaw;
        const auto move_until = std::chrono::steady_clock::now() +
                                std::chrono::duration<double>(std::max(0.0, duration_s));
        while (rclcpp::ok() && std::chrono::steady_clock::now() < move_until)
        {
            cmd_pub_->publish(moving);
            std::this_thread::sleep_for(period);
        }

        const geometry_msgs::msg::Twist stop;
        const auto                      settle_until =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_s_);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < settle_until)
        {
            cmd_pub_->publish(stop);
            std::this_thread::sleep_for(period);
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
        vision_msgs::msg::Detection3DArray::SharedPtr snapshot;
        {
            const std::lock_guard<std::mutex> lock(objects_mutex_);
            snapshot = objects_;
        }
        if (snapshot == nullptr)
        {
            return std::nullopt;
        }

        const double age_ms = (now() - rclcpp::Time(snapshot->header.stamp)).seconds() * 1e3;
        if (age_ms > object_timeout_ms_)
        {
            RCLCPP_WARN(get_logger(), "object poses are %.0f ms old", age_ms);
            return std::nullopt;
        }

        for (const auto& detection : snapshot->detections)
        {
            if (detection.results.empty() ||
                detection.results.front().hypothesis.class_id != object_id)
            {
                continue;
            }
            geometry_msgs::msg::PointStamped in_source;
            in_source.header = snapshot->header;
            in_source.point  = detection.results.front().pose.pose.position;
            try
            {
                // Latest available rather than the message stamp: both sides are ground truth
                // on this track, and insisting on an exact stamp match fails while the robot
                // walks for no accuracy gained.
                in_source.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
                return tf_buffer_.transform(in_source, base_frame_, tf2::durationFromSec(0.5));
            }
            catch (const tf2::TransformException& e)
            {
                RCLCPP_WARN(get_logger(), "cannot transform the object pose: %s", e.what());
                return std::nullopt;
            }
        }
        return std::nullopt;
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

        // The window mirrors with the arm, exactly as the grasp offset does: the right hand
        // works to the pelvis's -y, the left to its +y.
        ApproachLimits limits = limits_;
        if (goal->arm == ApproachObject::Goal::ARM_LEFT)
        {
            limits.target_bearing_rad = -limits.target_bearing_rad;
        }

        const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
        const auto   deadline  = deadlineIn(timeout_s);

        auto   feedback        = std::make_shared<ApproachObject::Feedback>();
        double turn_pulse      = turn_pulse_s_;
        double last_align_sign = 0.0;
        int    pulses          = 0;

        while (rclcpp::ok())
        {
            if (handle->is_canceling())
            {
                result->message = "closing: cancelled";
                handle->canceled(result);
                return;
            }
            if (std::chrono::steady_clock::now() > deadline)
            {
                result->message = "closing: gave up after " + std::to_string(timeout_s) + " s";
                handle->abort(result);
                return;
            }
            if (pulses >= max_pulses_)
            {
                result->message =
                    "closing: " + std::to_string(max_pulses_) + " pulses without converging";
                handle->abort(result);
                return;
            }

            const auto object = objectInBase(goal->object_id);
            if (!object)
            {
                result->message =
                    "locating: no fresh pose for '" + goal->object_id + "' on /objects";
                handle->abort(result);
                return;
            }

            const double range   = std::hypot(object->point.x, object->point.y);
            const double bearing = std::atan2(object->point.y, object->point.x);
            const auto   command = planApproach(range, bearing, limits);

            result->final_range_m     = range;
            result->final_bearing_rad = bearing;

            feedback->range_m     = range;
            feedback->bearing_rad = bearing;
            feedback->pulses      = pulses;

            switch (command.move)
            {
                case ApproachMove::kDone:
                    feedback->phase = ApproachObject::Feedback::PHASE_VERIFYING;
                    handle->publish_feedback(feedback);
                    result->success = true;
                    result->message = "the object is at " + std::to_string(range) + " m, " +
                                      std::to_string(bearing * 180.0 / M_PI) + " deg after " +
                                      std::to_string(pulses) + " pulses";
                    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
                    handle->succeed(result);
                    return;

                case ApproachMove::kOvershot:
                    result->message =
                        "closing: the object is " + std::to_string(range) +
                        " m away, inside the arm's working range, and this gait cannot reverse";
                    handle->abort(result);
                    return;

                case ApproachMove::kInvalid:
                    result->message = "locating: the configured reach window is unusable";
                    handle->abort(result);
                    return;

                case ApproachMove::kTurn:
                {
                    feedback->phase = ApproachObject::Feedback::PHASE_FACING;
                    handle->publish_feedback(feedback);

                    // A yaw pulse turns 0 to 18 degrees, which straddles the bearing tolerance,
                    // so a fixed pulse can oscillate about the window forever. Halving after an
                    // overshoot is what makes it converge.
                    const double sign = command.turn_rad > 0.0 ? 1.0 : -1.0;
                    if (last_align_sign != 0.0 && sign != last_align_sign)
                    {
                        turn_pulse = std::max(min_turn_pulse_s_, turn_pulse * 0.5);
                    }
                    last_align_sign = sign;

                    RCLCPP_INFO(
                        get_logger(),
                        "turn %+.1f deg (oblique %.1f), range %.3f, remaining %.3f",
                        command.turn_rad * 180.0 / M_PI,
                        command.oblique_rad * 180.0 / M_PI,
                        range,
                        command.remaining_m);
                    pulse(0.0, sign * pulse_vyaw_, turn_pulse);
                    ++pulses;
                    break;
                }

                case ApproachMove::kStep:
                {
                    feedback->phase = ApproachObject::Feedback::PHASE_CLOSING;
                    handle->publish_feedback(feedback);

                    const auto before = basePose();
                    RCLCPP_INFO(
                        get_logger(),
                        "step: range %.3f, remaining %.3f, quantum %.3f",
                        range,
                        command.remaining_m,
                        limits.pulse_advance_m);
                    pulse(pulse_vx_, 0.0, step_pulse_s_);
                    ++pulses;

                    const auto after = basePose();
                    if (before && after)
                    {
                        const double moved = std::hypot(
                            after->pose.position.x - before->pose.position.x,
                            after->pose.position.y - before->pose.position.y);
                        limits.pulse_advance_m = maxObservedAdvance(limits.pulse_advance_m, moved);
                    }
                    // Turning resumes from a full pulse: the halving above is about settling
                    // one heading, not a permanent property of the gait.
                    turn_pulse      = turn_pulse_s_;
                    last_align_sign = 0.0;
                    break;
                }
            }
        }

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
            result->message = "turning: no base pose to retreat from";
            handle->abort(result);
            return;
        }
        const double start_yaw = tf2::getYaw(start->pose.orientation);
        const double away_yaw  = wrap(start_yaw + M_PI);

        const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
        const auto   deadline  = deadlineIn(timeout_s);

        // Turn to face away. Reverse is not available: g1_gait_shaper compares forward speed
        // signed, so a negative command is zeroed at any magnitude.
        feedback->phase = Retreat::Feedback::PHASE_TURNING;
        handle->publish_feedback(feedback);
        if (!turnTo(handle, away_yaw, deadline))
        {
            result->message = "turning: could not face away";
            handle->abort(result);
            return;
        }

        feedback->phase  = Retreat::Feedback::PHASE_WALKING;
        double travelled = 0.0;
        int    pulses    = 0;
        while (rclcpp::ok() && travelled < goal->distance_m && pulses < max_pulses_)
        {
            if (handle->is_canceling())
            {
                result->travelled_m = travelled;
                result->message     = "walking: cancelled";
                handle->canceled(result);
                return;
            }
            if (std::chrono::steady_clock::now() > deadline)
            {
                break;
            }
            pulse(pulse_vx_, 0.0, step_pulse_s_);
            ++pulses;

            const auto here = basePose();
            if (here)
            {
                travelled = std::hypot(
                    here->pose.position.x - start->pose.position.x,
                    here->pose.position.y - start->pose.position.y);
            }
            feedback->travelled_m = travelled;
            handle->publish_feedback(feedback);
        }

        if (goal->restore_heading)
        {
            feedback->phase = Retreat::Feedback::PHASE_RESTORING;
            handle->publish_feedback(feedback);
            turnTo(handle, start_yaw, deadline);
        }

        result->travelled_m = travelled;
        result->success     = travelled >= goal->distance_m;
        result->message     = result->success ? "backed off " + std::to_string(travelled) + " m" :
                                                "walking: only made " + std::to_string(travelled) +
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

    /// Yaw-pulse until the base faces `target_yaw`. Shares the halving the approach uses.
    template <typename HandleT>
    bool turnTo(
        const std::shared_ptr<HandleT>& handle, double target_yaw,
        std::chrono::steady_clock::time_point deadline)
    {
        double turn_pulse = turn_pulse_s_;
        double last_sign  = 0.0;
        for (int i = 0; i < max_pulses_ && rclcpp::ok(); ++i)
        {
            if (handle->is_canceling() || std::chrono::steady_clock::now() > deadline)
            {
                return false;
            }
            const auto here = basePose();
            if (!here)
            {
                return false;
            }
            const double error = wrap(target_yaw - tf2::getYaw(here->pose.orientation));
            if (std::abs(error) <= limits_.bearing_tolerance_rad)
            {
                return true;
            }
            const double sign = error > 0.0 ? 1.0 : -1.0;
            if (last_sign != 0.0 && sign != last_sign)
            {
                turn_pulse = std::max(min_turn_pulse_s_, turn_pulse * 0.5);
            }
            last_sign = sign;
            pulse(0.0, sign * pulse_vyaw_, turn_pulse);
        }
        return false;
    }

    std::string cmd_topic_;
    std::string base_frame_;
    double      object_timeout_ms_ = 1500.0;
    double      pulse_vx_          = 0.45;
    double      pulse_vyaw_        = 1.30;
    double      step_pulse_s_      = 0.30;
    double      turn_pulse_s_      = 0.22;
    double      min_turn_pulse_s_  = 0.12;
    double      settle_s_          = 2.5;
    double      cmd_rate_hz_       = 20.0;
    int         max_pulses_        = 30;
    double      default_timeout_s_ = 180.0;

    ApproachLimits limits_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;
    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    rclcpp_action::Server<ApproachObject>::SharedPtr                    approach_server_;
    rclcpp_action::Server<Retreat>::SharedPtr                           retreat_server_;

    std::mutex                                    objects_mutex_;
    vision_msgs::msg::Detection3DArray::SharedPtr objects_;
    tf2_ros::Buffer                               tf_buffer_;
    tf2_ros::TransformListener                    tf_listener_;
};

}  // namespace g1_locomotion

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // Multi-threaded: a goal executes on its own thread and blocks on gait pulses for seconds
    // at a time, while /objects, TF and cancellation all have to keep flowing underneath it.
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<g1_locomotion::BaseApproachNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
