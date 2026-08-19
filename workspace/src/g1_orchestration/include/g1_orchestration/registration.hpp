#ifndef G1_ORCHESTRATION__REGISTRATION_HPP_
#define G1_ORCHESTRATION__REGISTRATION_HPP_

/**
 * @file registration.hpp
 * @brief Binding leaf classes to the names a tree XML uses.
 */

#include <behaviortree_cpp/bt_factory.h>

#include <memory>
#include <string>

#include "g1_orchestration/ros_action_node.hpp"

namespace g1_orchestration
{

/**
 * @brief Registers one leaf under the name a tree XML uses for it.
 *
 * The builder form, not registerNodeType<T>(): every leaf needs the ROS node, and the plain
 * form can only construct from (name, config).
 *
 * @tparam LeafT The leaf class to build.
 * @param factory Factory to register into.
 * @param id The name the tree XML uses.
 * @param context Copied into every instance the factory builds.
 */
template <typename LeafT>
void registerLeaf(BT::BehaviorTreeFactory& factory, const std::string& id, const RosContext& context)
{
    factory.registerBuilder<LeafT>(
        id,
        [context](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<LeafT>(name, config, context);
        });
}

/**
 * @brief Registers every leaf this package provides.
 *
 * One place, so the executor, the tests and the Groot2 model generator cannot see different
 * node sets.
 */
void registerSkillNodes(BT::BehaviorTreeFactory& factory, const RosContext& context);

/**
 * @brief The Groot2 palette for a factory.
 *
 * Shared by the generator and its drift test so the two cannot disagree about how the model is
 * produced.
 */
std::string nodeModelXml(const BT::BehaviorTreeFactory& factory);

}  // namespace g1_orchestration

#endif  // G1_ORCHESTRATION__REGISTRATION_HPP_
