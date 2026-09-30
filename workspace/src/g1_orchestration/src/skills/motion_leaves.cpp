#include "g1_orchestration/skills/motion_leaves.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace g1_orchestration
{

namespace
{

// The catalog bounds the arguments; these repeat them for trees that bypass the validator.
constexpr double kMinDistanceM = 0.1;
constexpr double kMaxDistanceM = 2.0;
constexpr double kMaxTurnDeg   = 180.0;

}  // namespace

// --- DriveStraight ------------------------------------------------------------------------

DriveStraight::DriveStraight(
    const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, std::move(context), "/drive_on_heading")
{}

BT::PortsList DriveStraight::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<std::string>("direction", "forward or backward, relative to the heading."),
        BT::InputPort<double>("distance", "How far to walk, m, from 0.1 to 2.0."),
        BT::InputPort<double>(
            "speed",
            0.3,
            "Commanded speed, m/s, kept within 0.2 to 0.4: the gait barely moves below, and above "
            "the costmap's 3 m window makes the collision check abort."),
        BT::InputPort<double>(
            "slack_s",
            8.0,
            "Seconds the allowance adds for the walk to start and settle."),
        BT::InputPort<double>(
            "coast_m",
            0.1,
            "How far the gait carries on after Nav2 stops it, taken off the distance asked for: "
            "measured at 0.3 m/s in the simulator, a calibration for the real robot."),
    });
}

bool DriveStraight::fillGoal(Goal& goal)
{
    const std::string direction = getInput<std::string>("direction").value_or("");
    const auto        distance  = getInput<double>("distance");
    if ((direction != "forward" && direction != "backward") || !distance ||
        !std::isfinite(*distance) || *distance < kMinDistanceM || *distance > kMaxDistanceM)
    {
        setFailureText(std::format(
            "a straight walk needs direction forward or backward and a distance from {} to {} m",
            kMinDistanceM,
            kMaxDistanceM));
        return false;
    }
    const double sign  = direction == "backward" ? -1.0 : 1.0;
    const double speed = std::clamp(getInput<double>("speed").value_or(0.3), 0.2, 0.4);
    const double slack = std::max(0.0, getInput<double>("slack_s").value_or(8.0));
    // A short walk barely reaches speed, so it coasts less: at most half of it is taken off.
    const double coast = std::clamp(getInput<double>("coast_m").value_or(0.1), 0.0, *distance / 2);
    goal.target.x      = sign * (*distance - coast);
    goal.speed         = static_cast<float>(sign * speed);
    // Half the commanded speed is what the gait still delivers on a bad day. Never zero: Nav2
    // reads a zero allowance as no limit.
    goal.time_allowance = rclcpp::Duration::from_seconds(slack + (*distance / (0.5 * speed)));
    return true;
}

BT::NodeStatus DriveStraight::judgeResult(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }
    using Result = nav2_msgs::action::DriveOnHeading::Result;
    // Nav2 1.3 leaves error_msg empty, so the codes are worded here.
    switch (result.result ? result.result->error_code : Result::UNKNOWN)
    {
        case Result::COLLISION_AHEAD:
            setFailureText(
                "something is in the way, inside the 0.45 m circle Nav2 keeps around the robot; "
                "it stopped where it was, part of the way");
            break;
        case Result::TIMEOUT:
            setFailureText("the walk did not cover the distance in time");
            break;
        case Result::TF_ERROR:
            setFailureText("the walk had no pose to measure the distance by");
            break;
        case Result::INVALID_INPUT:
            setFailureText("Nav2 refused the walk as invalid");
            break;
        default:
            setFailureText("the walk did not finish: " + describeResultCode(result.code));
    }
    RCLCPP_WARN(node_->get_logger(), "[%s] %s", name().c_str(), failureText().c_str());
    return BT::NodeStatus::FAILURE;
}

// --- TurnBy -------------------------------------------------------------------------------

TurnBy::TurnBy(const std::string& name, const BT::NodeConfig& config, RosContext context)
  : RosActionNode(name, config, std::move(context), "/spin")
{}

BT::PortsList TurnBy::providedPorts()
{
    return providedBasicPorts({
        BT::InputPort<double>("degrees", "How far to turn, from -180 to 180; positive is left."),
        BT::InputPort<double>(
            "yaw_rate",
            0.4,
            "Yaw rate, rad/s, the gait holds while turning. Sizes the time allowance."),
        BT::InputPort<double>(
            "slack_s",
            8.0,
            "Seconds the allowance adds for the turn to start and settle."),
    });
}

bool TurnBy::fillGoal(Goal& goal)
{
    const auto degrees = getInput<double>("degrees");
    if (!degrees || !std::isfinite(*degrees) || std::abs(*degrees) > kMaxTurnDeg)
    {
        setFailureText(std::format("a turn needs an angle from -{0} to {0} degrees", kMaxTurnDeg));
        return false;
    }
    const double radians = *degrees * M_PI / 180.0;
    goal.target_yaw      = static_cast<float>(radians);
    const double rate    = std::max(0.05, getInput<double>("yaw_rate").value_or(0.4));
    const double slack   = std::max(0.0, getInput<double>("slack_s").value_or(8.0));
    goal.time_allowance  = rclcpp::Duration::from_seconds(slack + (std::abs(radians) / rate));
    return true;
}

BT::NodeStatus TurnBy::judgeResult(const WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        return BT::NodeStatus::SUCCESS;
    }
    using Result = nav2_msgs::action::Spin::Result;
    switch (result.result ? result.result->error_code : Result::UNKNOWN)
    {
        case Result::COLLISION_AHEAD:
            setFailureText(
                "something is within 0.45 m of the robot's centre, so it would not turn; step "
                "clear first");
            break;
        case Result::TIMEOUT:
            setFailureText("the turn did not finish in time");
            break;
        case Result::TF_ERROR:
            setFailureText("the turn had no heading to measure by");
            break;
        default:
            setFailureText("the turn did not finish: " + describeResultCode(result.code));
    }
    RCLCPP_WARN(node_->get_logger(), "[%s] %s", name().c_str(), failureText().c_str());
    return BT::NodeStatus::FAILURE;
}

}  // namespace g1_orchestration
