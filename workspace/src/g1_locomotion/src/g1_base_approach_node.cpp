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
        pulse_vyaw_ = declare_parameter<double>("pulse_vyaw", 1.50);
        pulse_vy_   = declare_parameter<double>("pulse_vy", 0.50);
        // Must clear g1_gait_shaper's rev_engage, which is higher than fwd_engage on purpose.
        // The policy measures -0.247 m/s here and exactly nothing at -0.40.
        pulse_vrev_ = declare_parameter<double>("pulse_vrev", 0.60);
        // Measured durations, one per primitive. They are not interchangeable: forward is
        // irreducible at ~0.29 m however short the pulse, while yaw and lateral both have a
        // small-response mode that only survives at short durations.
        step_pulse_s_     = declare_parameter<double>("step_pulse_s", 0.30);
        turn_pulse_ccw_s_ = declare_parameter<double>("turn_pulse_ccw_s", 0.15);
        turn_pulse_cw_s_  = declare_parameter<double>("turn_pulse_cw_s", 0.60);
        strafe_pulse_s_   = declare_parameter<double>("strafe_pulse_s", 0.15);
        // A pulse that the gait ignores is lengthened and tried again, up to these ceilings.
        // The gait's response depends on how recently it moved: an isolated 0.15 s yaw pulse
        // turns 3.8 deg, and the same pulse inside this loop has been measured turning 0.5 deg.
        // Rather than pick one duration and be wrong half the time, grow it until it bites.
        // Ceilings on one continuous drive, so a goal that is never reached still ends.
        max_turn_s_   = declare_parameter<double>("max_turn_s", 8.0);
        max_step_s_   = declare_parameter<double>("max_step_s", 6.0);
        max_strafe_s_ = declare_parameter<double>("max_strafe_s", 5.0);
        // Stop commanding this far short of the target: the gait keeps going after the command
        // stops, and these are what it coasts.
        turn_lead_rad_  = declare_parameter<double>("turn_lead_rad", 0.30);
        step_lead_m_    = declare_parameter<double>("step_lead_m", 0.22);
        lateral_lead_m_ = declare_parameter<double>("lateral_lead_m", 0.055);
        // Kept under forward_tolerance_m; see the reverse case for why that ordering matters.
        reverse_lead_m_   = declare_parameter<double>("reverse_lead_m", 0.045);
        max_aim_attempts_ = declare_parameter<int>("max_aim_attempts", 8);
        // The gait keeps stepping after the command stops. Measuring before it settles reports
        // the command plus whatever of the stride was still in flight.
        settle_s_    = declare_parameter<double>("settle_s", 2.5);
        cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 20.0);

        limits_.target_x_m          = declare_parameter<double>("target_x_m", 0.280);
        limits_.target_y_m          = declare_parameter<double>("target_y_m", -0.200);
        limits_.forward_tolerance_m = declare_parameter<double>("forward_tolerance_m", 0.045);
        limits_.lateral_tolerance_m = declare_parameter<double>("lateral_tolerance_m", 0.050);
        limits_.min_forward_m       = declare_parameter<double>("min_forward_m", 0.180);
        limits_.step_threshold_m    = declare_parameter<double>("step_threshold_m", 0.32);

        // Consumed only by the aim before a forward drive; the planner itself never turns, so
        // this is not part of the reach window.
        heading_tolerance_rad_ = declare_parameter<double>("heading_tolerance_rad", 0.350);
        // How long a missing object pose or base transform is tolerated before the goal fails.
        lookup_grace_s_ = declare_parameter<double>("lookup_grace_s", 3.0);

        max_pulses_        = declare_parameter<int>("max_pulses", 90);
        default_timeout_s_ = declare_parameter<double>("default_timeout_s", 420.0);

        if (!limitsAreUsable(limits_))
        {
            throw std::runtime_error(
                "g1_base_approach: the configured reach window is not one the planner can aim "
                "at; check target_x_m against min_forward_m and the tolerances");
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
            "base approach ready: object wanted at (%.3f, %.3f) in %s, window +/-%.3f forward "
            "and +/-%.3f lateral, publishing on %s",
            limits_.target_x_m,
            limits_.target_y_m,
            base_frame_.c_str(),
            limits_.forward_tolerance_m,
            limits_.lateral_tolerance_m,
            cmd_topic_.c_str());
    }

private:
    /// Command a velocity for `duration_s`, then hold zero while the gait settles.
    ///
    /// Zeros are published rather than merely stopping: the shaper hands the channel back to
    /// Nav2 when this source goes quiet, so an idle gap mid-skill would surrender priority.
    /// Hold a velocity until `done()` returns true, then stop and let the gait settle.
    ///
    /// CONTINUOUS, not pulsed, and that is the whole point. The original design commanded fixed
    /// pulses and re-measured between them, which fights the walking policy: repeated
    /// command-and-stop cycles wind it down, measured decaying from 8.3 degrees of yaw on the
    /// first pulse to 0.3 by the eighth and to nothing by the twentieth. Every duration this
    /// file was tuned against came from a probe that fired two or three pulses in a row, so it
    /// measured a rested gait every time and the loop never saw those numbers.
    ///
    /// Nav2 drives this robot perfectly well with a continuous velocity stream. So does this.
    template <typename DoneT>
    bool driveUntil(
        double vx, double vy, double vyaw, DoneT done, double max_s,
        std::chrono::steady_clock::time_point deadline)
    {
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
        geometry_msgs::msg::Twist moving;
        moving.linear.x  = vx;
        moving.linear.y  = vy;
        moving.angular.z = vyaw;

        const auto give_up = std::chrono::steady_clock::now() +
                             std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                 std::chrono::duration<double>(max_s));
        bool reached = false;
        while (rclcpp::ok() && std::chrono::steady_clock::now() < give_up &&
               std::chrono::steady_clock::now() < deadline)
        {
            if (done())
            {
                reached = true;
                break;
            }
            cmd_pub_->publish(moving);
            std::this_thread::sleep_for(period);
        }

        settle();
        return reached;
    }

    /// Zeros while the gait finishes the stride it is in, then silence so the velocity gate
    /// falls idle rather than re-issuing SetVelocity(0) at 5 Hz.
    void settle()
    {
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
        const geometry_msgs::msg::Twist stop;
        const auto                      settle_until =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_s_);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < settle_until)
        {
            cmd_pub_->publish(stop);
            std::this_thread::sleep_for(period);
        }
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(quiet_s_)));
    }

    void pulse(double vx, double vy, double vyaw, double duration_s)
    {
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));

        geometry_msgs::msg::Twist moving;
        moving.linear.x       = vx;
        moving.linear.y       = vy;
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

        // Then say nothing at all for a moment. This is the one thing that differed between the
        // probe that produced every number in the config and this loop, and it is worth more
        // than any of them: the same 0.60 s clockwise pulse turns 4 to 6 degrees when measured
        // with a silent gap after it, and 0.5 degrees when the zeros never stop. Going quiet
        // lets g1_loco_bridge's velocity gate fall idle instead of re-issuing SetVelocity(0) at
        // 5 Hz, and the gait starts the next pulse from rest rather than from whatever the
        // re-issue stream left it in.
        //
        // Nothing takes the channel during the gap: the shaper's override lapses after
        // override_timeout_s, but Nav2 publishes nothing between goals, so the output is silence
        // either way.
        std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(quiet_s_)));
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
        // The one path out of here that used to say nothing at all, and the abort it causes
        // reads identically to a stale or untransformable pose. Worth a line: a live run
        // aborted here with /objects publishing at 10 Hz and left no evidence of which.
        RCLCPP_WARN(
            get_logger(),
            "'%s' is not among the %zu objects being reported",
            object_id.c_str(),
            snapshot->detections.size());
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
            limits.target_y_m = -limits.target_y_m;
        }

        const double timeout_s = goal->timeout_s > 0.0 ? goal->timeout_s : default_timeout_s_;
        const auto   deadline  = deadlineIn(timeout_s);

        auto feedback = std::make_shared<ApproachObject::Feedback>();
        int  pulses   = 0;

        // The heading held for the whole approach. Fixed up front rather than recomputed from
        // the object each iteration: the object moves in the base frame as the robot walks, and
        // chasing it is what made the previous version arrive square to nothing.
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

            // Re-read rather than give up on the first miss. Both of these can fail for a
            // moment for reasons that are not the skill's problem: a TF buffer that has not
            // caught up after the base moved, or a sample arriving late. Aborting on one miss
            // threw away an otherwise healthy approach mid-mission with /objects publishing at
            // 10 Hz throughout. Bounded, so a genuinely absent object still ends the goal.
            std::optional<geometry_msgs::msg::PointStamped> object;
            std::optional<geometry_msgs::msg::PoseStamped>  here;
            const auto give_up_looking = deadlineIn(lookup_grace_s_);
            while (rclcpp::ok() && std::chrono::steady_clock::now() < give_up_looking)
            {
                object = objectInBase(goal->object_id);
                here   = basePose();
                if (object && here)
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (!object || !here)
            {
                result->message = "locating: no fresh pose for '" + goal->object_id + "' on " +
                                  "/objects after " + std::to_string(lookup_grace_s_) + " s";
                handle->abort(result);
                return;
            }

            // Judged in the RAW base frame, because that is the frame the ARM works in. Where
            // the object sits relative to the robot is the whole of reachability; which way the
            // room faces is not part of it. The working heading only steers the drives, in
            // aimAt -- it is not an input to the plan.
            const auto command = planApproach(object->point.x, object->point.y, limits);

            result->final_x_m = object->point.x;
            result->final_y_m = object->point.y;

            feedback->forward_error_m = command.forward_error_m;
            feedback->lateral_error_m = command.lateral_error_m;
            feedback->pulses          = pulses;

            switch (command.move)
            {
                case ApproachMove::kDone:
                    feedback->phase = ApproachObject::Feedback::PHASE_VERIFYING;
                    handle->publish_feedback(feedback);
                    result->success = true;
                    result->message = "the object is at (" + std::to_string(object->point.x) +
                                      ", " + std::to_string(object->point.y) + ") after " +
                                      std::to_string(pulses) + " pulses";
                    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
                    handle->succeed(result);
                    return;

                case ApproachMove::kOvershot:
                    result->message = "closing: the object is " + std::to_string(object->point.x) +
                                      " m ahead, under the robot's own footprint; re-stage "
                                      "through Nav2";
                    handle->abort(result);
                    return;

                case ApproachMove::kInvalid:
                    result->message = "locating: the configured reach window is unusable";
                    handle->abort(result);
                    return;

                case ApproachMove::kStep:
                {
                    feedback->phase = ApproachObject::Feedback::PHASE_CLOSING;
                    handle->publish_feedback(feedback);

                    // Aim first, ALWAYS. A forward pulse yaws the robot 8 degrees every time, so
                    // without re-aiming before each one the approach walks an arc; that is what
                    // the spiral in the first probe run was.
                    RCLCPP_INFO(
                        get_logger(),
                        "step: forward error %.3f, lateral %.3f",
                        command.forward_error_m,
                        command.lateral_error_m);
                    if (!aimAt(handle, working_yaw, deadline, pulses))
                    {
                        result->message = "facing: could not hold the working heading";
                        handle->abort(result);
                        return;
                    }

                    // Drive forward continuously and stop short of the window, leaving the
                    // gait's coast to carry the rest. Whatever it does not close, the creep
                    // finishes at a few centimetres a pulse.
                    const auto close_enough = [&] {
                        const auto object_now = objectInBase(goal->object_id);
                        if (!object_now)
                        {
                            return true;
                        }
                        const double lead = command.coarse ? step_lead_m_ : 0.0;
                        return object_now->point.x - limits.target_x_m <= lead;
                    };
                    driveUntil(pulse_vx_, 0.0, 0.0, close_enough, max_step_s_, deadline);
                    ++pulses;
                    break;
                }

                case ApproachMove::kReverse:
                {
                    feedback->phase = ApproachObject::Feedback::PHASE_CLOSING;
                    handle->publish_feedback(feedback);
                    RCLCPP_INFO(
                        get_logger(),
                        "reverse: forward error %.3f, lateral %.3f",
                        command.forward_error_m,
                        command.lateral_error_m);
                    // The lead here MUST stay under forward_tolerance_m, and getting that
                    // backwards livelocks rather than degrades. It used step_lead_m_ (0.22)
                    // against a tolerance of 0.110, so every error between the two left the
                    // planner demanding a reverse and driveUntil reporting itself already
                    // arrived: the same "-0.138" logged every 2.7 s, forever, with no time
                    // spent driving. Exactly the dead band the heading correction hit.
                    const double lead = std::min(reverse_lead_m_, 0.5 * limits.forward_tolerance_m);
                    const auto   back_far_enough = [&] {
                        const auto object_now = objectInBase(goal->object_id);
                        if (!object_now)
                        {
                            return true;
                        }
                        return object_now->point.x - limits.target_x_m >= -lead;
                    };
                    driveUntil(-pulse_vrev_, 0.0, 0.0, back_far_enough, max_step_s_, deadline);
                    ++pulses;
                    break;
                }

                case ApproachMove::kStrafe:
                {
                    feedback->phase = ApproachObject::Feedback::PHASE_SIDESTEPPING;
                    handle->publish_feedback(feedback);

                    // Continuous while there is real distance to cover, one short pulse for the
                    // last few centimetres. Same hybrid the heading correction uses, and for the
                    // same reason: neither primitive covers both ends.
                    //
                    // This was a fixed 0.15 s pulse for every correction, and it was the slowest
                    // thing in the mission by a wide margin. Each one moved about 12 mm and then
                    // spent settle + quiet standing still, so 3.9 s per centimetre and a half:
                    // one live approach spent 54 of them closing a gap a single drive covers.
                    // Forward and reverse had been continuous since the pulse decay was
                    // measured; lateral was simply left behind.
                    const bool far = std::abs(command.lateral_error_m) > lateral_lead_m_;
                    RCLCPP_INFO(
                        get_logger(),
                        "strafe %s: lateral error %.3f, %s",
                        command.lateral_sign > 0.0 ? "left" : "right",
                        command.lateral_error_m,
                        far ? "driving" : "nudging");
                    if (far)
                    {
                        // Closed on the live measurement, stopping a lead short because the gait
                        // keeps sliding after the command ends.
                        const auto close_enough = [&] {
                            const auto object_now = objectInBase(goal->object_id);
                            if (!object_now)
                            {
                                return true;
                            }
                            const double error = object_now->point.y - limits.target_y_m;
                            // Stop on reaching the lead band OR on crossing zero, so a fast
                            // slide cannot run away when it passes the target between ticks.
                            return std::abs(error) <= lateral_lead_m_ ||
                                   error * command.lateral_sign < 0.0;
                        };
                        driveUntil(
                            0.0,
                            command.lateral_sign * pulse_vy_,
                            0.0,
                            close_enough,
                            max_strafe_s_,
                            deadline);
                    }
                    else
                    {
                        pulse(0.0, command.lateral_sign * pulse_vy_, 0.0, strafe_pulse_s_);
                    }
                    ++pulses;
                    break;
                }
            }
        }

        result->message = "closing: shutting down";
        handle->abort(result);
    }

    /// How long to hold a yaw command, which depends on which way it turns.
    ///
    /// This gait's yaw is ASYMMETRIC, measured: a 0.15 s pulse turns +3.8 deg counter-clockwise
    /// but only -1.1 deg clockwise, and clockwise does not reach -3.5 deg until 0.60 s. That
    /// matters more than it sounds, because a forward step yaws +8 deg every time, so every
    /// correction the approach needs is in the weak direction -- the first live run spent
    /// fourteen pulses failing to take out 7 degrees.
    double turnPulseFor(double error_rad) const
    {
        return error_rad > 0.0 ? turn_pulse_ccw_s_ : turn_pulse_cw_s_;
    }

    /// Yaw-pulse until the base is within the heading tolerance of `target_yaw`.
    ///
    /// A closed loop rather than one pulse per planner tick, because a 3.8 degree pulse would
    /// otherwise cost a whole re-plan and a fresh object lookup per degree of correction.
    template <typename HandleT>
    bool aimAt(
        const std::shared_ptr<HandleT>& handle, double target_yaw,
        std::chrono::steady_clock::time_point deadline, int& pulses)
    {
        // One continuous turn per attempt, closed on the measured heading, rather than a train
        // of pulses. The lead angle stops the command early because the gait keeps rotating
        // after it does: 1.5 rad/s commanded delivers about 1.08, and it coasts.
        const auto reached = [&] {
            const auto here = basePose();
            if (!here)
            {
                return true;
            }
            return std::abs(wrap(target_yaw - tf2::getYaw(here->pose.orientation))) <=
                   turn_lead_rad_;
        };

        for (int attempt = 0; attempt < max_aim_attempts_ && rclcpp::ok(); ++attempt)
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
            if (std::abs(error) <= heading_tolerance_rad_)
            {
                return true;
            }
            // Continuous for the big swing, one short pulse for the last few degrees.
            //
            // Neither alone works. A continuous turn cannot stop inside the tolerance because
            // the gait coasts about 17 degrees after the command ends, wider than the 8 degree
            // window; and a train of pulses winds the policy down to nothing, which is what the
            // fully pulsed design failed on. So drive while the error is bigger than the coast,
            // and pulse once it is not.
            //
            // Getting this wrong deadlocks rather than degrades. With the stopping lead larger
            // than the tolerance, every error between the two left the planner demanding a turn
            // and driveUntil declaring itself already arrived, logged as the same +9.2 deg
            // forever, 3.7 s apart, with no time spent driving.
            const bool far = std::abs(error) > turn_lead_rad_;
            RCLCPP_INFO(
                get_logger(),
                "aim %d/%d: heading off by %+.1f deg, %s",
                attempt + 1,
                max_aim_attempts_,
                error * 180.0 / M_PI,
                far ? "driving" : "nudging");
            if (far)
            {
                driveUntil(
                    0.0,
                    0.0,
                    std::copysign(pulse_vyaw_, error),
                    reached,
                    max_turn_s_,
                    deadline);
            }
            else
            {
                pulse(0.0, 0.0, std::copysign(pulse_vyaw_, error), turnPulseFor(error));
            }
            ++pulses;
        }

        const auto settled = basePose();
        return settled && std::abs(wrap(target_yaw - tf2::getYaw(settled->pose.orientation))) <
                              2.0 * heading_tolerance_rad_;
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

        // Reverse, and that is the whole skill. No turn, no walk.
        //
        // It used to turn 180 degrees and walk the remaining distance, which was doing Nav2's job
        // badly: a navigation goal follows immediately and is far better at going somewhere than a
        // hand-rolled pulse controller. And the turn was actively harmful -- taken beside a
        // workbench it swings the robot and whatever it is holding across the table, which is
        // exactly what this exists to prevent.
        //
        // Reverse is real but only just: the policy measures -0.247 m/s at a commanded -0.60 and
        // exactly nothing at -0.40, and g1_gait_shaper has a rev_engage above a planner's backup
        // speeds so a deliberate command like this one gets through and a stray one does not.
        feedback->phase = Retreat::Feedback::PHASE_BACKING_OFF;
        handle->publish_feedback(feedback);

        const auto travelled = [&] {
            const auto here = basePose();
            return here ? std::hypot(
                              here->pose.position.x - start->pose.position.x,
                              here->pose.position.y - start->pose.position.y) :
                          0.0;
        };
        const auto far_enough = [&] { return travelled() >= goal->distance_m; };

        while (rclcpp::ok() && !far_enough() && std::chrono::steady_clock::now() < deadline)
        {
            if (handle->is_canceling())
            {
                result->travelled_m = travelled();
                result->message     = "backing_off: cancelled";
                handle->canceled(result);
                return;
            }
            driveUntil(-pulse_vrev_, 0.0, 0.0, far_enough, max_step_s_, deadline);
            feedback->travelled_m = travelled();
            handle->publish_feedback(feedback);
        }

        const double backed = travelled();
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
    double      object_timeout_ms_     = 1500.0;
    double      pulse_vx_              = 0.45;
    double      pulse_vyaw_            = 1.50;
    double      pulse_vy_              = 0.50;
    double      pulse_vrev_            = 0.60;
    double      step_pulse_s_          = 0.30;
    double      turn_pulse_ccw_s_      = 0.15;
    double      turn_pulse_cw_s_       = 0.60;
    double      strafe_pulse_s_        = 0.15;
    double      max_turn_s_            = 8.0;
    double      max_step_s_            = 6.0;
    double      max_strafe_s_          = 5.0;
    double      turn_lead_rad_         = 0.30;
    double      step_lead_m_           = 0.22;
    double      lateral_lead_m_        = 0.055;
    double      reverse_lead_m_        = 0.045;
    double      heading_tolerance_rad_ = 0.350;
    double      lookup_grace_s_        = 3.0;
    int         max_aim_attempts_      = 8;
    double      settle_s_              = 2.5;
    double      quiet_s_               = 1.2;
    double      cmd_rate_hz_           = 20.0;
    int         max_pulses_            = 90;
    double      default_timeout_s_     = 420.0;

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
