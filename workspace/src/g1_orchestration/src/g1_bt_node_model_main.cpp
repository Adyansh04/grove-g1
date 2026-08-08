/**
 * @file g1_bt_node_model_main.cpp
 * @brief Writes the Groot2 palette for every leaf this package registers.
 *
 * Groot2 needs a node model to OFFER nodes for editing; the Groot2Publisher alone only lets it
 * watch. Generated from the same factory the executor builds, so the editor cannot show a leaf
 * with ports it does not have. test_node_model catches drift from the checked-in copy.
 *
 *   ros2 run g1_orchestration g1_bt_node_model > trees/g1_orchestration_nodes.xml
 */

#include <behaviortree_cpp/bt_factory.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "g1_orchestration/skill_nodes.hpp"

int main(int argc, char** argv)
{
    // A node is built only because the leaf builders take one; registration never dereferences
    // it, and nothing here reaches the graph.
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("g1_bt_node_model");

    BT::BehaviorTreeFactory      factory;
    g1_orchestration::RosContext context{ node };
    g1_orchestration::registerSkillNodes(factory, context);

    const std::string models = g1_orchestration::nodeModelXml(factory);

    if (argc > 1)
    {
        std::ofstream out(argv[1]);
        if (!out)
        {
            std::cerr << "cannot write " << argv[1] << "\n";
            rclcpp::shutdown();
            return 1;
        }
        out << models;
    }
    else
    {
        std::cout << models;
    }

    rclcpp::shutdown();
    return 0;
}
