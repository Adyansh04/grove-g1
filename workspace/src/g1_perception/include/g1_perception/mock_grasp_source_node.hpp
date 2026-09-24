#ifndef G1_PERCEPTION__MOCK_GRASP_SOURCE_NODE_HPP_
#define G1_PERCEPTION__MOCK_GRASP_SOURCE_NODE_HPP_

/**
 * @file mock_grasp_source_node.hpp
 * @brief A grasp generator stand-in that needs no GPU or model.
 *
 * Serves the real adapter's service from /objects alone: three top-down grasps and one from
 * under the table, so a grasp filter that refuses nothing fails a test.
 */

#include <g1_msgs/srv/generate_grasps.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace g1_perception
{

class G1MockGraspSource : public rclcpp::Node
{
public:
    explicit G1MockGraspSource(const rclcpp::NodeOptions& options);

private:
    void onObjects(vision_msgs::msg::Detection3DArray::ConstSharedPtr objects);
    void onRequest(
        const std::shared_ptr<g1_msgs::srv::GenerateGrasps::Request>&  request,
        const std::shared_ptr<g1_msgs::srv::GenerateGrasps::Response>& response);

    rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr objects_sub_;
    rclcpp::Service<g1_msgs::srv::GenerateGrasps>::SharedPtr            service_;

    vision_msgs::msg::Detection3DArray::ConstSharedPtr objects_;
    double                                             approach_height_m_{ 0.10 };
};

}  // namespace g1_perception

#endif  // G1_PERCEPTION__MOCK_GRASP_SOURCE_NODE_HPP_
