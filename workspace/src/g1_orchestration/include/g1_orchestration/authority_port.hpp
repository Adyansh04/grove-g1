#ifndef G1_ORCHESTRATION__AUTHORITY_PORT_HPP_
#define G1_ORCHESTRATION__AUTHORITY_PORT_HPP_

/**
 * @file authority_port.hpp
 * @brief How the mission executor takes and gives back the arms, behind a seam for tests.
 */

#include <atomic>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

#include "g1_orchestration/arm_authority.hpp"

namespace g1_orchestration
{

/// Whether each hand's fingers stand closed: on something, for all the executor can tell.
struct ClosedHands
{
    bool left  = false;
    bool right = false;
};

/**
 * @brief Which hands stand closed in @p joints.
 *
 * @param finger_joints The joints that close each hand, named left_* or right_*.
 * @param closed_rad A hand is closed once any of its finger joints stands past this.
 * @return A hand whose finger joint is missing from @p joints, or reads NaN, counts as closed.
 */
[[nodiscard]] ClosedHands closedHands(
    const sensor_msgs::msg::JointState& joints, const std::vector<std::string>& finger_joints,
    double closed_rad);

/**
 * @brief Taking and releasing the arms and hands.
 *
 * Both calls block: they make controller_manager calls on a node of their own, so neither may run
 * on a thread an executor is using for the same node.
 */
class AuthorityPort
{
public:
    AuthorityPort()                                = default;
    AuthorityPort(const AuthorityPort&)            = delete;
    AuthorityPort& operator=(const AuthorityPort&) = delete;
    AuthorityPort(AuthorityPort&&)                 = delete;
    AuthorityPort& operator=(AuthorityPort&&)      = delete;
    virtual ~AuthorityPort()                       = default;

    /// Takes the arms and both hands. kFailed when the arm could not be taken. kAlready when it
    /// was taken before this call, by someone who may have left an object in a hand.
    virtual Acquired acquire() = 0;

    /// Hands them back. Never throws. False when the arms could not be handed back: still held.
    [[nodiscard]] virtual bool release() = 0;

    /// Which hands stand closed, from one reading of the fingers; both when none comes. A released
    /// hand goes limp, so one closed by someone the arms were lent to would drop what it holds.
    virtual ClosedHands closedHands() = 0;
};

/**
 * @brief The real thing: controller_manager, through acquireArm and releaseArm.
 */
class ControllerManagerAuthority final : public AuthorityPort
{
public:
    /// How the fingers are read: see closedHands().
    struct Fingers
    {
        std::vector<std::string> joints;
        double                   closed_rad = 0.0;
        /// How long to wait for /joint_states, s.
        double timeout_s = 0.0;
    };

    /**
     * @param logger Where progress and failures are reported.
     * @param timeout_s Per-step service budget.
     * @param fingers How the fingers are read.
     * @param stopped Whether a stop came, which cuts an acquisition's settle short.
     */
    ControllerManagerAuthority(
        rclcpp::Logger logger, double timeout_s, Fingers fingers,
        std::function<bool()> stopped = {});

    Acquired    acquire() override;
    bool        release() override;
    ClosedHands closedHands() override;

private:
    rclcpp::Logger        logger_;
    double                timeout_s_;
    Fingers               fingers_;
    std::function<bool()> stopped_;
};

/**
 * @brief The arms one run of g1_bt_executor took, and how to read the fingers before letting go.
 */
struct ArmClaim
{
    /// Set when this run's AcquireArm took the arms from the freeze controller, cleared once it
    /// let go of them. Arms someone else took are never this run's to release.
    std::atomic<bool>                   taken{ false };
    ControllerManagerAuthority::Fingers fingers;
};

/**
 * @brief Lets go of the arms and hands unless a hand stands closed, since a released hand drops
 *        what it holds.
 *
 * @param logger Where the decision is reported.
 * @param timeout_s Per-step budget for the controller_manager calls, s.
 * @param fingers How the fingers are read; with no reading both hands count as closed.
 * @return True if the arms were released; false if they were kept, or could not be handed back.
 */
bool releaseUnlessHolding(
    const rclcpp::Logger& logger, double timeout_s,
    const ControllerManagerAuthority::Fingers& fingers);

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__AUTHORITY_PORT_HPP_
