#ifndef G1_ORCHESTRATION__AUTHORITY_PORT_HPP_
#define G1_ORCHESTRATION__AUTHORITY_PORT_HPP_

/**
 * @file authority_port.hpp
 * @brief How the mission executor takes and gives back the arms, behind a seam for tests.
 */

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
 * @return A hand whose finger joint is missing from @p joints counts as closed.
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

    /// Hands them back. Best effort, and never throws.
    virtual void release() = 0;

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
     */
    ControllerManagerAuthority(rclcpp::Logger logger, double timeout_s, Fingers fingers);

    Acquired    acquire() override;
    void        release() override;
    ClosedHands closedHands() override;

private:
    rclcpp::Logger logger_;
    double         timeout_s_;
    Fingers        fingers_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__AUTHORITY_PORT_HPP_
