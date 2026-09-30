#ifndef G1_ORCHESTRATION__AUTHORITY_PORT_HPP_
#define G1_ORCHESTRATION__AUTHORITY_PORT_HPP_

/**
 * @file authority_port.hpp
 * @brief How the mission executor takes and gives back the arms, behind a seam for tests.
 */

#include <rclcpp/rclcpp.hpp>

#include "g1_orchestration/arm_authority.hpp"

namespace g1_orchestration
{

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
};

/**
 * @brief The real thing: controller_manager, through acquireArm and releaseArm.
 */
class ControllerManagerAuthority final : public AuthorityPort
{
public:
    /**
     * @param logger Where progress and failures are reported.
     * @param timeout_s Per-step service budget.
     */
    ControllerManagerAuthority(rclcpp::Logger logger, double timeout_s);

    Acquired acquire() override;
    void     release() override;

private:
    rclcpp::Logger logger_;
    double         timeout_s_;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__AUTHORITY_PORT_HPP_
