#ifndef G1_ORCHESTRATION__SKILL_NODES_HPP_
#define G1_ORCHESTRATION__SKILL_NODES_HPP_

/**
 * @file skill_nodes.hpp
 * @brief Umbrella over every leaf a mission tree can use.
 *
 * Each leaf is a thin client with its own file under skills/. The tree decides what happens and
 * in what order; the skills decide how. Nothing here plans, moves a joint, or takes control
 * authority: Nav2 and g1_manipulation own all of that, and this package only sequences them.
 */

#include "g1_orchestration/port_types.hpp"
#include "g1_orchestration/registration.hpp"
#include "g1_orchestration/skills/approach_object.hpp"
#include "g1_orchestration/skills/arm_authority_leaves.hpp"
#include "g1_orchestration/skills/clear_costmaps.hpp"
#include "g1_orchestration/skills/clear_octomap.hpp"
#include "g1_orchestration/skills/navigate_to_pose.hpp"
#include "g1_orchestration/skills/pick.hpp"
#include "g1_orchestration/skills/place.hpp"
#include "g1_orchestration/skills/retreat.hpp"
#include "g1_orchestration/skills/set_arm_posture.hpp"

#endif  // G1_ORCHESTRATION__SKILL_NODES_HPP_
