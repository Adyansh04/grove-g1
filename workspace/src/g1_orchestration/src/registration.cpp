#include "g1_orchestration/registration.hpp"

#include <behaviortree_cpp/xml_parsing.h>

#include <string>

#include "g1_orchestration/skills/approach_object.hpp"
#include "g1_orchestration/skills/arm_authority_leaves.hpp"
#include "g1_orchestration/skills/clear_costmaps.hpp"
#include "g1_orchestration/skills/clear_octomap.hpp"
#include "g1_orchestration/skills/navigate_to_pose.hpp"
#include "g1_orchestration/skills/pick.hpp"
#include "g1_orchestration/skills/place.hpp"
#include "g1_orchestration/skills/retreat.hpp"
#include "g1_orchestration/skills/set_arm_posture.hpp"

namespace g1_orchestration
{

void registerSkillNodes(BT::BehaviorTreeFactory& factory, const RosContext& context)
{
    registerLeaf<NavigateToPose>(factory, "NavigateToPose", context);
    registerLeaf<ApproachObject>(factory, "ApproachObject", context);
    registerLeaf<Retreat>(factory, "Retreat", context);
    registerLeaf<Pick>(factory, "Pick", context);
    registerLeaf<Place>(factory, "Place", context);
    registerLeaf<SetArmPosture>(factory, "SetArmPosture", context);
    registerLeaf<ClearCostmaps>(factory, "ClearCostmaps", context);
    registerLeaf<ClearOctomap>(factory, "ClearOctomap", context);
    registerLeaf<AcquireArm>(factory, "AcquireArm", context);
    registerLeaf<ReleaseArm>(factory, "ReleaseArm", context);
}

std::string nodeModelXml(const BT::BehaviorTreeFactory& factory)
{
    // Our own nodes only: Groot2 ships the built-ins.
    return BT::writeTreeNodesModelXML(factory, false);
}

}  // namespace g1_orchestration
