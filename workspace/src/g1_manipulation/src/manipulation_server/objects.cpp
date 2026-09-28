/**
 * @file objects.cpp
 * @brief Where the objects are: the /objects stream, lookups, and poses in the planning frame.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <initializer_list>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "g1_manipulation/g1_manipulation_server_node.hpp"

namespace g1_manipulation
{

namespace
{

constexpr auto kObjectPollPeriod = std::chrono::milliseconds(200);

/// Whether a detection's pose and box can be planned against.
bool isFinite(const vision_msgs::msg::Detection3D& detection)
{
    const geometry_msgs::msg::Pose&    pose = detection.results.front().pose.pose;
    const geometry_msgs::msg::Vector3& size = detection.bbox.size;
    return std::ranges::all_of(
        std::initializer_list<double>{ pose.position.x,
                                       pose.position.y,
                                       pose.position.z,
                                       pose.orientation.x,
                                       pose.orientation.y,
                                       pose.orientation.z,
                                       pose.orientation.w,
                                       size.x,
                                       size.y,
                                       size.z },
        [](double value) { return std::isfinite(value); });
}

}  // namespace

const std::string& G1ManipulationServer::idOf(const vision_msgs::msg::Detection3D& detection)
{
    return detection.results.front().hypothesis.class_id;
}

std::optional<geometry_msgs::msg::Pose> G1ManipulationServer::toPlanningFrame(
    const geometry_msgs::msg::Pose& pose, const std::string& frame_id)
{
    if (frame_id.empty() || frame_id == planning_frame_)
    {
        return pose;
    }

    geometry_msgs::msg::PoseStamped in;
    in.header.frame_id = frame_id;
    // Latest transform, not the pose's stamp: a standing object's odom pose stays valid, and the
    // question is where it is from where the robot stands now.
    in.header.stamp = rclcpp::Time(0);
    in.pose         = pose;

    try
    {
        return tf_buffer_.transform(in, planning_frame_, std::chrono::milliseconds(500)).pose;
    }
    catch (const tf2::TransformException& e)
    {
        RCLCPP_ERROR(
            get_logger(),
            "cannot transform '%s' into the planning frame '%s': %s",
            frame_id.c_str(),
            planning_frame_.c_str(),
            e.what());
        return std::nullopt;
    }
}

void G1ManipulationServer::onObjects(const vision_msgs::msg::Detection3DArray::ConstSharedPtr& msg)
{
    const std::lock_guard<std::mutex> lock(objects_mutex_);
    objects_ = *msg;
    for (const Detection& detection : msg->detections)
    {
        if (!detection.results.empty())
        {
            Detection& seen = sightings_[idOf(detection)];
            seen            = detection;
            seen.header     = msg->header;
        }
    }
    // Track ids are numbered afresh as tracks come and go, so aged-out ones are dropped.
    const rclcpp::Time newest(msg->header.stamp);
    std::erase_if(sightings_, [&](const auto& entry) {
        return (newest - rclcpp::Time(entry.second.header.stamp)).seconds() > sighting_memory_s_;
    });
}

std::optional<G1ManipulationServer::Detection>
G1ManipulationServer::lookUpObject(const std::string& object_id, bool report)
{
    std::optional<Detection> found;
    std_msgs::msg::Header    header;
    std::size_t              known = 0;
    {
        const std::lock_guard<std::mutex> lock(objects_mutex_);
        header = objects_.header;
        known  = objects_.detections.size();
        for (const Detection& detection : objects_.detections)
        {
            if (!detection.results.empty() && idOf(detection) == object_id)
            {
                found = detection;
                break;
            }
        }
    }

    // Judged here rather than at the source: only the skill about to commit the arm knows how old
    // is too old, which is why the pose source forwards the capture stamp.
    const double age = (now() - rclcpp::Time(header.stamp)).seconds();
    if (known == 0 || age > object_timeout_s_)
    {
        if (report)
        {
            RCLCPP_ERROR(
                get_logger(),
                "No usable object poses: %zu known, newest %.2f s old (limit %.2f). Is "
                "g1_object_pose_source active?",
                known,
                age,
                object_timeout_s_);
        }
        return std::nullopt;
    }
    if (!found)
    {
        if (report)
        {
            RCLCPP_ERROR(get_logger(), "No object called '%s' is being reported", object_id.c_str());
        }
        return std::nullopt;
    }
    if (!isFinite(*found))
    {
        if (report)
        {
            RCLCPP_ERROR(get_logger(), "'%s' has a non-finite pose or size", object_id.c_str());
        }
        return std::nullopt;
    }
    found->header = header;
    return found;
}

std::optional<G1ManipulationServer::Detection>
G1ManipulationServer::waitForObject(const std::string& object_id, double timeout_s)
{
    const rclcpp::Time deadline = now() + rclcpp::Duration::from_seconds(timeout_s);
    while (now() < deadline)
    {
        if (auto found = lookUpObject(object_id, /*report=*/false))
        {
            return found;
        }
        rclcpp::sleep_for(kObjectPollPeriod);
    }
    return lookUpObject(object_id);
}

std::optional<G1ManipulationServer::Detection>
G1ManipulationServer::lastSighting(const std::string& object_id)
{
    Detection seen;
    {
        const std::lock_guard<std::mutex> lock(objects_mutex_);
        const auto                        found = sightings_.find(object_id);
        if (found == sightings_.end())
        {
            return std::nullopt;
        }
        seen = found->second;
    }
    const double age = (now() - rclcpp::Time(seen.header.stamp)).seconds();
    if (age > sighting_memory_s_ || !isFinite(seen))
    {
        return std::nullopt;
    }
    RCLCPP_INFO(
        get_logger(),
        "'%s' is not in the newest detections; using where it was seen %.1f s ago",
        object_id.c_str(),
        age);
    return seen;
}

}  // namespace g1_manipulation
