/**
 * @file mock_grasp_source_node.cpp
 * @brief Fixed grasp candidates around an object on /objects, one of them deliberately bad.
 */

#include "g1_perception/mock_grasp_source_node.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

namespace g1_perception
{
namespace
{

rclcpp::QoS objectsQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

/// A pose above @p centre, approaching along -z of the world, rotated by @p yaw about it.
geometry_msgs::msg::Pose
approachFromAbove(const geometry_msgs::msg::Point& centre, double height, double yaw)
{
    geometry_msgs::msg::Pose pose;
    pose.position = centre;
    pose.position.z += height;
    // Generator convention: +z is the approach direction, so a downward grasp points z at the
    // floor.
    tf2::Quaternion rotation;
    rotation.setRPY(M_PI, 0.0, yaw);
    pose.orientation = tf2::toMsg(rotation);
    return pose;
}

}  // namespace

G1MockGraspSource::G1MockGraspSource(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_mock_grasp_source", options)
{
    declare_parameter<std::string>("hand", "right");
    declare_parameter<bool>("only_from_below", false);
    approach_height_m_ = declare_parameter<double>("approach_height_m", 0.10);

    objects_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
        "objects",
        objectsQos(),
        [this](vision_msgs::msg::Detection3DArray::ConstSharedPtr objects) {
            onObjects(std::move(objects));
        });
    service_ = create_service<g1_msgs::srv::GenerateGrasps>(
        "~/generate_grasps",
        [this](
            const std::shared_ptr<g1_msgs::srv::GenerateGrasps::Request>&  request,
            const std::shared_ptr<g1_msgs::srv::GenerateGrasps::Response>& response) {
            onRequest(request, response);
        });
}

void G1MockGraspSource::onObjects(vision_msgs::msg::Detection3DArray::ConstSharedPtr objects)
{
    objects_ = std::move(objects);
}

void G1MockGraspSource::onRequest(
    const std::shared_ptr<g1_msgs::srv::GenerateGrasps::Request>&  request,
    const std::shared_ptr<g1_msgs::srv::GenerateGrasps::Response>& response)
{
    // Read per request, so one run can switch them between calls.
    const std::string hand            = get_parameter("hand").as_string();
    const bool        only_from_below = get_parameter("only_from_below").as_bool();

    response->ok = false;
    if (!request->hand.empty() && request->hand != hand)
    {
        response->message = "this stand-in generates grasps for the " + hand + " hand only";
        return;
    }
    if (objects_ == nullptr || objects_->detections.empty())
    {
        response->message = "nothing on /objects yet";
        return;
    }

    const auto match = std::find_if(
        objects_->detections.begin(),
        objects_->detections.end(),
        [&request](const vision_msgs::msg::Detection3D& detection) {
            return !detection.results.empty() &&
                   detection.results.front().hypothesis.class_id == request->object_id;
        });
    if (match == objects_->detections.end())
    {
        std::string known;
        for (const vision_msgs::msg::Detection3D& detection : objects_->detections)
        {
            if (!detection.results.empty())
            {
                known +=
                    (known.empty() ? "" : ", ") + detection.results.front().hypothesis.class_id;
            }
        }
        response->message =
            "nothing called '" + request->object_id + "' is on /objects; there is " + known;
        return;
    }

    const geometry_msgs::msg::Point& centre = match->bbox.center.position;
    const double                     top    = 0.5 * match->bbox.size.z;

    response->ok      = true;
    response->message = "stand-in grasps for '" + request->object_id + "'";
    response->header  = objects_->header;
    response->grasps.clear();
    if (!only_from_below)
    {
        response->grasps = { approachFromAbove(centre, top + approach_height_m_, 0.0),
                             approachFromAbove(centre, top + approach_height_m_, M_PI_4),
                             approachFromAbove(centre, top + approach_height_m_, M_PI_2) };
    }
    // Last and worst: from underneath, through the table, for the filter to refuse.
    geometry_msgs::msg::Pose from_below = approachFromAbove(centre, -top - approach_height_m_, 0.0);
    from_below.orientation              = geometry_msgs::msg::Quaternion();
    from_below.orientation.w            = 1.0;
    response->grasps.push_back(from_below);
    // Descending: a generator answers best first and the filter stops at the first under the bar.
    response->scores.resize(response->grasps.size());
    for (std::size_t i = 0; i < response->scores.size(); ++i)
    {
        response->scores[i] = 0.9F - (0.1F * static_cast<float>(i));
    }
}

}  // namespace g1_perception
