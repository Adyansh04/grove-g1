#ifndef G1_ORCHESTRATION__SKILL_NODES_HPP_
#define G1_ORCHESTRATION__SKILL_NODES_HPP_

/**
 * @file skill_nodes.hpp
 * @brief Umbrella over the skill leaves and their registration.
 *
 * Leaves are thin clients: Nav2 and g1_manipulation plan and move, and the tree sequences them.
 * The only authority this package takes itself is the arm bracket in arm_authority.hpp.
 */

#include "g1_orchestration/port_types.hpp"
#include "g1_orchestration/registration.hpp"
#include "g1_orchestration/skills/approach_object.hpp"
#include "g1_orchestration/skills/arm_authority_leaves.hpp"
#include "g1_orchestration/skills/clear_costmaps.hpp"
#include "g1_orchestration/skills/clear_octomap.hpp"
#include "g1_orchestration/skills/grasp.hpp"
#include "g1_orchestration/skills/look_for.hpp"
#include "g1_orchestration/skills/navigate_to_pose.hpp"
#include "g1_orchestration/skills/pick.hpp"
#include "g1_orchestration/skills/place.hpp"
#include "g1_orchestration/skills/retreat.hpp"
#include "g1_orchestration/skills/set_arm_posture.hpp"

#endif  // G1_ORCHESTRATION__SKILL_NODES_HPP_
