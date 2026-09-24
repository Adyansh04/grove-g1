/**
 * @file g1_bt_node_model_main.cpp
 * @brief Writes the Groot2 palette for every leaf this package registers.
 *
 * Groot2 needs this model to offer nodes for editing; the Groot2Publisher only lets it watch.
 * Writes to the path given as the first argument, else to stdout:
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
    // Only because the leaf builders take a node; registration never uses it.
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
