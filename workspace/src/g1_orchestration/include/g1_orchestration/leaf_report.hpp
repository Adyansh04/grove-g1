#ifndef G1_ORCHESTRATION__LEAF_REPORT_HPP_
#define G1_ORCHESTRATION__LEAF_REPORT_HPP_

/**
 * @file leaf_report.hpp
 * @brief What the mission executor reads off a leaf besides its status.
 */

#include <string>
#include <utility>

namespace g1_orchestration
{

/**
 * @brief A leaf's account of why it failed and which action server it drives.
 *
 * The status alone says a leaf failed; the reason is the skill server's own words, which a planner
 * can act on. The executor finds this on a node with a dynamic_cast when a leaf reports FAILURE.
 */
class LeafReport
{
public:
    LeafReport()                             = default;
    LeafReport(const LeafReport&)            = delete;
    LeafReport& operator=(const LeafReport&) = delete;
    LeafReport(LeafReport&&)                 = delete;
    LeafReport& operator=(LeafReport&&)      = delete;
    virtual ~LeafReport()                    = default;

    /// Why the leaf last failed, in the server's words. Empty until it has.
    [[nodiscard]] const std::string& failureText() const { return failure_text_; }

    /// The action server this leaf sends goals to, or empty for a leaf that calls a service.
    [[nodiscard]] virtual std::string actionServer() const { return {}; }

    /// Whether its latest start sent a goal: not while a pause held it back.
    [[nodiscard]] bool goalSent() const { return goal_sent_; }

    /// Whether it last failed because its server was not up: no test of the skill.
    [[nodiscard]] bool serverMissing() const { return server_missing_; }

protected:
    void setFailureText(std::string text) { failure_text_ = std::move(text); }
    void setGoalSent(bool sent) { goal_sent_ = sent; }
    void setServerMissing(bool missing) { server_missing_ = missing; }

private:
    std::string failure_text_;
    bool        goal_sent_      = false;
    bool        server_missing_ = false;
};

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__LEAF_REPORT_HPP_
