/**
 * @file service_leaf.cpp
 * @brief The client node a blocking service call runs on.
 */

#include "g1_orchestration/service_leaf.hpp"

#include <memory>
#include <string>

namespace g1_orchestration
{

rclcpp::Node::SharedPtr makeClientNode(const std::string& name)
{
    return std::make_shared<rclcpp::Node>(name);
}

}  // namespace g1_orchestration
