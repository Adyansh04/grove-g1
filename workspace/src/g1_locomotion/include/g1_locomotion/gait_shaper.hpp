#ifndef G1_LOCOMOTION__GAIT_SHAPER_HPP_
#define G1_LOCOMOTION__GAIT_SHAPER_HPP_

/**
 * @file gait_shaper.hpp
 * @brief Pure, ROS-free reduction of a planner's Twist onto the gait's achievable motions.
 */

namespace g1_locomotion
{

/**
 * @brief Reduces an arbitrary commanded velocity to one this gait can actually produce.
 *
 * The sim walking policy has a hard gait-initiation deadband with no hysteresis: measured, it
 * produces no motion at or below 0.35 m/s forward and 0.60 rad/s yaw, and steps only from 0.40
 * and 1.50. Commanding anything in between does nothing at all. Worse, combined commands come
 * out badly wrong -- a commanded (0.50, 0, 0.50) measured (0.337, 0.299, 0.390), a third of a
 * metre per second of lateral nobody asked for.
 *
 * A planner samples a continuous velocity space and has no way to express any of that. This
 * class collapses its output onto the four motions that exist: stop, drive straight, strafe,
 * turn in place.
 *
 * Strafe was not always here. It was dropped originally because Nav2's controller cannot command
 * lateral on this robot anyway and the gait already produces uncommanded lateral drift, so
 * asking for more looked pointless. Milestone 9's base approach is the caller that changes
 * that: the mobile-manipulation answer to "park precisely enough for the arm to reach" is a
 * holonomic base, and this gait IS holonomic at the velocity level -- the policy measures
 * lateral motion from 0.50 m/s. Throwing that away meant every lateral correction cost a turn,
 * a step and a turn back.
 *
 * **Subtractive only.** Every output is the input unchanged, the input clamped smaller, or
 * zero -- never larger. That is the invariant the whole design rests on: turning a small
 * command into a large motion is exactly what this stack's control-mode rules exist to
 * prevent, and it is asserted as a property test rather than left as a comment.
 *
 * Lives here rather than in Nav2 configuration because the deadband is a property of *this sim
 * walking policy*, not of the real G1's onboard MPC gait controller, which has no such dead
 * zone. Encoding it into planner tuning would mean un-tuning the navigation layer at hardware
 * bring-up. `g1_loco_bridge.yaml`'s `axis_sign` and `max_velocity` are here for the same
 * reason.
 */
class GaitShaper
{
public:
    /// See config/g1_gait_shaper.yaml for values and their provenance.
    struct Config
    {
        double fwd_engage{ 0.45 };
        double yaw_engage{ 1.20 };
        double yaw_clamp{ 1.57 };
        double lat_engage{ 0.50 };
        double lat_clamp{ 0.50 };
    };

    struct Command
    {
        double vx{ 0.0 };
        double vy{ 0.0 };
        double vyaw{ 0.0 };
    };

    /**
     * @brief Constructs a shaper, rejecting a config it could not honour.
     *
     * @throws std::invalid_argument if `fwd_engage < 0`, `yaw_engage <= 0`, `yaw_clamp < 0`,
     *         `yaw_clamp < yaw_engage`, `lat_engage <= 0`, `lat_clamp < 0`, or
     *         `lat_clamp < lat_engage`.
     *
     * The class owns this rather than whoever reads the YAML. `shape()` clamps against
     * `yaw_clamp`, which is undefined for inverted bounds, and the subtractive-only invariant
     * above is a property of the class *and* its configuration.
     */
    explicit GaitShaper(const Config& config);

    /**
     * @brief Reduces one command to a producible motion.
     *
     * Yaw is tested first, so a command carrying both becomes a pure turn: the measured
     * combined-command response is the worst case, and rotate-then-drive is what the caller's
     * shim controller is trying to do anyway.
     *
     * Yaw compares on magnitude and keeps its sign -- turning either way is proven. Forward
     * compares *signed*, so any negative vx becomes zero at any magnitude. That asymmetry is
     * deliberate: reverse is measured at -0.247 m/s for a commanded -0.60 and exactly 0.000
     * for -0.40, so a planner's usual backup speeds sit entirely inside the dead zone. It is
     * also the reason a misconfigured recovery behaviour cannot produce a reverse lurch.
     *
     * Lateral is tested LAST, so it only survives a command carrying nothing else. That keeps
     * the primitives mutually exclusive, which the measured combined-command response demands:
     * (0.50, 0, 0.50) came out (0.337, 0.299, 0.390). A caller wanting to strafe must ask for
     * strafe alone.
     *
     * Like yaw, lateral compares on magnitude and keeps its sign. Unlike forward, where the
     * signed comparison is a deliberate backstop against a reverse lurch, there is no reason to
     * prefer one side: the policy was measured stepping laterally and nothing suggests the two
     * directions differ.
     *
     * Non-finite inputs need no special case: NaN fails both comparisons and falls through to
     * a stop, and an infinite yaw clamps.
     */
    [[nodiscard]] Command shape(const Command& in) const;

private:
    Config config_;
};

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__GAIT_SHAPER_HPP_
