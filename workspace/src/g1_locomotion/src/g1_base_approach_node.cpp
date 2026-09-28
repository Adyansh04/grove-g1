/**
 * @file g1_base_approach_node.cpp
 * @brief The base approach node: its parameters, action servers, and the base and object lookups.
 */

#include "g1_locomotion/g1_base_approach_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace g1_locomotion
{

namespace
{

/// A retreat only has to clear a surface; anything longer is a malformed goal.
constexpr double kMaxRetreatDistanceM = 2.0;

/// The 3 x 3 m rolling costmap's half width less the 0.8 m longest step: beyond it the step would
/// end where the grid cannot see, and cells outside the window read as free.
constexpr double kMaxStepClearanceM = 0.7;

}  // namespace

double BaseApproachNode::wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

std::chrono::steady_clock::time_point BaseApproachNode::deadlineIn(double seconds)
{
    // A day stands in for no limit: casting an infinite or huge double to the clock's integer
    // ticks is undefined, and lands the deadline in the past.
    constexpr double kLongest = 86400.0;
    const double bounded = std::isfinite(seconds) ? std::clamp(seconds, 0.0, kLongest) : kLongest;
    return std::chrono::steady_clock::now() +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double>(bounded));
}

template <typename ActionT, typename Body>
void BaseApproachNode::runGuarded(
    Body&& body, const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>& handle)
{
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

BaseApproachNode::BaseApproachNode()
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

    const StepClearLimits c;
    step_limits_.body_radius_m =
        declare_parameter<double>("step_clear.body_radius_m", c.body_radius_m);
    step_limits_.max_step_m = declare_parameter<double>("step_clear.max_step_m", c.max_step_m);
    step_limits_.turn_cost_m_per_rad =
        declare_parameter<double>("step_clear.turn_cost_m_per_rad", c.turn_cost_m_per_rad);
    step_speed_mps_ = declare_parameter<double>("step_clear.speed_mps", 0.30);
    step_heading_tolerance_rad_ =
        declare_parameter<double>("step_clear.heading_tolerance_rad", 0.15);
    step_timeout_s_ = declare_parameter<double>("step_clear.timeout_s", 20.0);
    costmap_topic_ =
        declare_parameter<std::string>("step_clear.costmap_topic", "/local_costmap/costmap");
    settle_s_    = declare_parameter<double>("settle_s", 1.0);
    cmd_rate_hz_ = declare_parameter<double>("cmd_rate_hz", 20.0);

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

    // The AGILE controller's profile: reliable so the closing zero arrives, never latched.
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        cmd_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
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

    // Nav2 latches its costmap, so the subscription must too or nothing arrives until the
    // next change.
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_,
        rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
            const std::lock_guard<std::mutex> lock(costmap_mutex_);
            costmap_ = msg;
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
            goals_running_.fetch_add(1);
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
            goals_running_.fetch_add(1);
            std::thread{ [this, handle] {
                runGuarded([&] { runRetreat(handle); }, handle);
            } }.detach();
        });

    step_clear_server_ = rclcpp_action::create_server<StepClear>(
        this,
        "~/step_clear",
        [this](const rclcpp_action::GoalUUID&, const StepClear::Goal::ConstSharedPtr& goal) {
            if (goal->clearance_m <= 0.0 || goal->clearance_m > kMaxStepClearanceM)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "rejecting a step clear to %.3f m: must be within (0, %.1f]",
                    goal->clearance_m,
                    kMaxStepClearanceM);
                return rclcpp_action::GoalResponse::REJECT;
            }
            return acquire() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
                               rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<GoalHandleStepClear>&) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<GoalHandleStepClear>& handle) {
            goals_running_.fetch_add(1);
            std::thread{ [this, handle] {
                runGuarded([&] { runStepClear(handle); }, handle);
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

BaseApproachNode::~BaseApproachNode()
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

bool BaseApproachNode::acquire()
{
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true))
    {
        RCLCPP_WARN(get_logger(), "rejecting a goal: another one is already running");
        return false;
    }
    return true;
}

std::chrono::duration<double> BaseApproachNode::tickPeriod() const
{
    return std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_hz_));
}

void BaseApproachNode::publish(double vx, double vy, double yaw_rate)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = vx;
    twist.linear.y  = vy;
    twist.angular.z = yaw_rate;
    cmd_pub_->publish(twist);
}

void BaseApproachNode::settle()
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

std::optional<geometry_msgs::msg::PoseStamped> BaseApproachNode::basePose(const std::string& frame)
{
    try
    {
        const auto tf =
            tf_buffer_
                .lookupTransform(frame, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
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
            "no %s to %s: %s",
            frame.c_str(),
            base_frame_.c_str(),
            e.what());
        return std::nullopt;
    }
}

std::optional<geometry_msgs::msg::PointStamped>
BaseApproachNode::objectInBase(const std::string& object_id)
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

}  // namespace g1_locomotion
