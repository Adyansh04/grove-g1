/**
 * @file world_model_node.cpp
 * @brief ROS plumbing around the world model: inputs, services, publishing and persistence.
 */

#include "g1_world_model/world_model_node.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace g1_world_model
{

namespace
{

rclcpp::QoS sensorQos()
{
    return rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile();
}

/// Latched: a subscriber that starts late still gets the current rooms and objects.
rclcpp::QoS latchedQos() { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

double seconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) + (1e-9 * stamp.nanosec);
}

std::string lowered(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool containsWord(const std::string& haystack, const std::string& needle)
{
    return !needle.empty() && lowered(haystack).find(lowered(needle)) != std::string::npos;
}

std::string objectId(int id) { return "O" + std::to_string(id); }

/// @p area of an rgb8 or bgr8 image as BGR, or empty for any other encoding.
cv::Mat bgrArea(const sensor_msgs::msg::Image& image, cv::Rect area)
{
    area &= cv::Rect(0, 0, static_cast<int>(image.width), static_cast<int>(image.height));
    if ((image.encoding != "rgb8" && image.encoding != "bgr8") || area.empty() ||
        image.data.size() < static_cast<std::size_t>(image.step) * image.height)
    {
        return {};
    }
    // Copied row by row: the message is const, and the area is all that is kept.
    cv::Mat out(area.height, area.width, CV_8UC3);
    for (int row = 0; row < area.height; ++row)
    {
        std::memcpy(
            out.ptr<std::uint8_t>(row),
            image.data.data() + (static_cast<std::size_t>(area.y + row) * image.step) +
                (static_cast<std::size_t>(area.x) * 3U),
            static_cast<std::size_t>(area.width) * 3U);
    }
    if (image.encoding == "rgb8")
    {
        cv::cvtColor(out, out, cv::COLOR_RGB2BGR);
    }
    return out;
}

/// "room A" .. "room Z", then "room 27" and on: short names people can say.
std::string roomAlias(int number)
{
    if (number >= 1 && number <= 26)
    {
        return std::string("room ") + static_cast<char>('A' + number - 1);
    }
    return "room " + std::to_string(number);
}

geometry_msgs::msg::Pose poseOf(double x, double y, double z, double yaw)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x    = x;
    pose.position.y    = y;
    pose.position.z    = z;
    pose.orientation.z = std::sin(0.5 * yaw);
    pose.orientation.w = std::cos(0.5 * yaw);
    return pose;
}

}  // namespace

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("g1_world_model", options)
  , depth_history_(declare_parameter<double>("frame_history_s", 6.0), 0.02, 64)
  , color_history_(get_parameter("frame_history_s").as_double(), 0.02, 64)
{
    map_frame_          = declare_parameter<std::string>("map_frame", "map");
    base_frame_         = declare_parameter<std::string>("base_frame", "base_footprint");
    world_dir_          = declare_parameter<std::string>("world_dir", "");
    require_stillness_  = declare_parameter<bool>("require_stillness", true);
    still_linear_       = declare_parameter<double>("still_linear", 0.05);
    still_angular_      = declare_parameter<double>("still_angular", 0.05);
    settle_s_           = declare_parameter<double>("settle_s", 1.0);
    tf_wait_s_          = declare_parameter<double>("tf_wait_s", 0.2);
    resegment_period_s_ = declare_parameter<double>("resegment_period_s", 5.0);
    autosave_period_s_  = declare_parameter<double>("autosave_period_s", 60.0);
    describe_           = declare_parameter<bool>("describe", false);
    describe_after_     = static_cast<int>(declare_parameter<int>("describe_after", 3));
    describe_retry_s_   = declare_parameter<double>("describe_retry_s", 60.0);
    describe_min_crop_ =
        static_cast<int>(declare_parameter<int>("describe_min_crop", describe_min_crop_));
    describe_room_views_ =
        static_cast<int>(declare_parameter<int>("describe_room_views", describe_room_views_));
    describe_rooms_below_ =
        declare_parameter<double>("describe_rooms_below", describe_rooms_below_);
    describe_room_coverage_ =
        declare_parameter<double>("describe_room_coverage", describe_room_coverage_);
    structure_enabled_   = declare_parameter<bool>("structure.enabled", true);
    structure_min_z_     = declare_parameter<double>("structure.min_z", 1.4);
    structure_max_z_     = declare_parameter<double>("structure.max_z", 2.2);
    structure_min_hits_  = static_cast<int>(declare_parameter<int>("structure.min_hits", 3));
    structure_dilate_    = declare_parameter<double>("structure.dilate", 0.30);
    structure_min_cells_ = static_cast<int>(declare_parameter<int>("structure.min_cells", 200));
    structure_min_clear_ = static_cast<int>(declare_parameter<int>("structure.min_clear", 5));

    CoverageParams coverage;
    coverage.min_range  = declare_parameter<double>("coverage.min_range", coverage.min_range);
    coverage.good_range = declare_parameter<double>("coverage.good_range", coverage.good_range);
    coverage.max_range  = declare_parameter<double>("coverage.max_range", coverage.max_range);
    coverage.well_seen  = declare_parameter<double>("coverage.well_seen", coverage.well_seen);
    coverage.pixel_stride =
        static_cast<int>(declare_parameter<int>("coverage.pixel_stride", coverage.pixel_stride));
    coverage.floor_incidence_limit =
        declare_parameter<double>("coverage.floor_incidence_limit", coverage.floor_incidence_limit);
    coverage.face_incidence_limit =
        declare_parameter<double>("coverage.face_incidence_limit", coverage.face_incidence_limit);
    coverage.surface_incidence_limit = declare_parameter<double>(
        "coverage.surface_incidence_limit",
        coverage.surface_incidence_limit);
    coverage_ = CoverageMap(coverage);

    PlannerParams planner;
    planner.robot_radius = declare_parameter<double>("planner.robot_radius", planner.robot_radius);
    planner.clearance_margin =
        declare_parameter<double>("planner.clearance_margin", planner.clearance_margin);
    planner.travel_margin =
        declare_parameter<double>("planner.travel_margin", planner.travel_margin);
    planner.candidate_spacing =
        declare_parameter<double>("planner.candidate_spacing", planner.candidate_spacing);
    planner.max_headings =
        static_cast<int>(declare_parameter<int>("planner.max_headings", planner.max_headings));
    planner.min_heading_gain =
        declare_parameter<double>("planner.min_heading_gain", planner.min_heading_gain);
    planner.min_viewpoint_gain =
        declare_parameter<double>("planner.min_viewpoint_gain", planner.min_viewpoint_gain);
    planner.travel_speed = declare_parameter<double>("planner.travel_speed", planner.travel_speed);
    planner.turn_speed   = declare_parameter<double>("planner.turn_speed", planner.turn_speed);
    planner.dwell_time   = declare_parameter<double>("planner.dwell_time", planner.dwell_time);
    planner.goal_overhead =
        declare_parameter<double>("planner.goal_overhead", planner.goal_overhead);
    planner.room_switch_factor =
        declare_parameter<double>("planner.room_switch_factor", planner.room_switch_factor);
    planner.max_candidates =
        static_cast<int>(declare_parameter<int>("planner.max_candidates", planner.max_candidates));
    planner.min_frontier_size =
        declare_parameter<double>("planner.min_frontier_size", planner.min_frontier_size);
    planner.max_attempts =
        static_cast<int>(declare_parameter<int>("planner.max_attempts", planner.max_attempts));
    planner.max_room_failures = static_cast<int>(
        declare_parameter<int>("planner.max_room_failures", planner.max_room_failures));
    planner.max_failures_in_a_row = static_cast<int>(
        declare_parameter<int>("planner.max_failures_in_a_row", planner.max_failures_in_a_row));
    planner_ = ViewpointPlanner(planner);

    ObjectMapParams objects;
    objects.voxel     = declare_parameter<double>("objects.voxel", objects.voxel);
    objects.max_depth = declare_parameter<double>("objects.max_depth", objects.max_depth);
    objects.min_score = declare_parameter<double>("objects.min_score", objects.min_score);
    objects.min_match = declare_parameter<double>("objects.min_match", objects.min_match);
    objects.position_tolerance =
        declare_parameter<double>("objects.position_tolerance", objects.position_tolerance);
    objects.stale_after =
        static_cast<int>(declare_parameter<int>("objects.stale_after", objects.stale_after));
    objects.remove_after =
        static_cast<int>(declare_parameter<int>("objects.remove_after", objects.remove_after));
    objects.merge_gap = declare_parameter<double>("objects.merge_gap", objects.merge_gap);
    objects.support_merge_gap =
        declare_parameter<double>("objects.support_merge_gap", objects.support_merge_gap);
    objects.max_merged_extent =
        declare_parameter<double>("objects.max_merged_extent", objects.max_merged_extent);
    objects.min_observations = static_cast<int>(
        declare_parameter<int>("objects.min_observations", objects.min_observations));
    objects.confirm_s      = declare_parameter<double>("objects.confirm_s", objects.confirm_s);
    objects.support_labels = declare_parameter<std::vector<std::string>>(
        "objects.support_labels",
        objects.support_labels);
    objects_ = ObjectMap(objects);

    segmentation_params_.min_persistence =
        declare_parameter<double>("rooms.min_persistence", segmentation_params_.min_persistence);
    segmentation_params_.min_peak =
        declare_parameter<double>("rooms.min_peak", segmentation_params_.min_peak);
    segmentation_params_.max_doorway_width =
        declare_parameter<double>("rooms.max_doorway_width", segmentation_params_.max_doorway_width);
    segmentation_params_.max_pinch_ratio =
        declare_parameter<double>("rooms.max_pinch_ratio", segmentation_params_.max_pinch_ratio);
    segmentation_params_.min_room_area =
        declare_parameter<double>("rooms.min_room_area", segmentation_params_.min_room_area);

    approach_params_.robot_radius = planner.robot_radius;
    approach_params_.standoff =
        declare_parameter<double>("approach.standoff", approach_params_.standoff);
    approach_params_.max_standoff =
        declare_parameter<double>("approach.max_standoff", approach_params_.max_standoff);

    const std::string types = declare_parameter<std::string>("room_types_file", "");
    if (!types.empty())
    {
        try
        {
            room_types_ = RoomTypeTable::fromYaml(types);
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(
                get_logger(),
                "room types from '%s' failed: %s",
                types.c_str(),
                error.what());
        }
    }

    if (!world_dir_.empty())
    {
        std::string error;
        pending_restore_ = loadWorld(world_dir_, error);
        if (pending_restore_)
        {
            RCLCPP_INFO(
                get_logger(),
                "loaded a world from %s; applied once its map arrives",
                world_dir_.c_str());
        }
        else
        {
            RCLCPP_INFO(
                get_logger(),
                "no world to resume in %s (%s); starting empty",
                world_dir_.c_str(),
                error.c_str());
        }
    }

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        latchedQos(),
        [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map) { onMap(map); });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "depth/image_raw",
        sensorQos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr depth) { onDepth(std::move(depth)); });
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "color/image_raw",
        sensorQos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr color) { onColor(std::move(color)); });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info",
        sensorQos(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) { onCameraInfo(info); });
    // Reliable, as both the real and the mock detector publish them.
    masks_sub_ = create_subscription<g1_msgs::msg::InstanceMaskArray>(
        "instance_masks",
        rclcpp::QoS(rclcpp::KeepLast(4)).reliable(),
        [this](const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks) { onMasks(masks); });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr& odometry) { onOdometry(odometry); });
    if (structure_enabled_)
    {
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud",
            sensorQos(),
            [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) { onCloud(cloud); });
    }
    description_sub_ = create_subscription<g1_msgs::msg::Description>(
        "descriptions",
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const g1_msgs::msg::Description::ConstSharedPtr& description) {
            onDescription(description);
        });

    rooms_pub_    = create_publisher<g1_msgs::msg::RoomArray>("~/rooms", latchedQos());
    objects_pub_  = create_publisher<g1_msgs::msg::WorldObjectArray>("~/objects", latchedQos());
    coverage_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/coverage", latchedQos());
    markers_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    describe_pub_ = create_publisher<g1_msgs::msg::DescribeRequest>(
        "~/describe_requests",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    using NextViewpointSrv   = g1_msgs::srv::NextViewpoint;
    using ReportViewpointSrv = g1_msgs::srv::ReportViewpoint;
    using FindObjectsSrv     = g1_msgs::srv::FindObjects;
    using ApproachSrv        = g1_msgs::srv::GetApproachPose;
    using TriggerSrv         = std_srvs::srv::Trigger;
    next_viewpoint_srv_      = create_service<NextViewpointSrv>(
        "~/next_viewpoint",
        [this](
            const NextViewpointSrv::Request::SharedPtr&  request,
            const NextViewpointSrv::Response::SharedPtr& response) {
            onNextViewpoint(request, response);
        });
    report_viewpoint_srv_ = create_service<ReportViewpointSrv>(
        "~/report_viewpoint",
        [this](
            const ReportViewpointSrv::Request::SharedPtr&  request,
            const ReportViewpointSrv::Response::SharedPtr& response) {
            onReportViewpoint(request, response);
        });
    find_objects_srv_ = create_service<FindObjectsSrv>(
        "~/find_objects",
        [this](
            const FindObjectsSrv::Request::SharedPtr&  request,
            const FindObjectsSrv::Response::SharedPtr& response) {
            onFindObjects(request, response);
        });
    approach_srv_ = create_service<ApproachSrv>(
        "~/get_approach_pose",
        [this](
            const ApproachSrv::Request::SharedPtr&  request,
            const ApproachSrv::Response::SharedPtr& response) {
            onGetApproachPose(request, response);
        });
    save_srv_ = create_service<TriggerSrv>(
        "~/save",
        [this](
            const TriggerSrv::Request::SharedPtr&  request,
            const TriggerSrv::Response::SharedPtr& response) { onSave(request, response); });

    integrate_timer_ =
        create_wall_timer(std::chrono::milliseconds(50), [this] { integratePendingDepth(); });
    publish_timer_  = create_wall_timer(std::chrono::seconds(1), [this] { publishState(); });
    describe_timer_ = create_wall_timer(std::chrono::seconds(2), [this] { requestDescriptions(); });
    if (!world_dir_.empty() && autosave_period_s_ > 0.0)
    {
        autosave_timer_ =
            create_wall_timer(std::chrono::duration<double>(autosave_period_s_), [this] {
                if (dirty_)
                {
                    const std::string failure = saveNow();
                    if (!failure.empty())
                    {
                        RCLCPP_WARN(get_logger(), "autosave failed: %s", failure.c_str());
                    }
                }
            });
    }
}

WorldModelNode::~WorldModelNode()
{
    if (!world_dir_.empty() && dirty_)
    {
        saveNow();
    }
}

// --- inputs ---------------------------------------------------------------------------------

void WorldModelNode::onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map)
{
    const GridGeometry geometry{ map->info.resolution,
                                 map->info.origin.position.x,
                                 map->info.origin.position.y,
                                 static_cast<int>(map->info.width),
                                 static_cast<int>(map->info.height) };
    if (geometry.cellCount() != map->data.size() || geometry.cellCount() == 0)
    {
        RCLCPP_WARN(get_logger(), "map data does not match its size; ignored");
        return;
    }
    cv::Mat cells = classifyOccupancy(map->data.data(), geometry);
    if (!cells_.empty() && geometry == geometry_ && cv::countNonZero(cells != cells_) == 0)
    {
        return;  // The map server republishing the same map.
    }
    if (!(geometry == geometry_) && !room_labels_.empty())
    {
        room_labels_ = resampleLabels(room_labels_, geometry_, geometry);
    }
    cells_    = std::move(cells);
    geometry_ = geometry;
    coverage_.setMap(cells_, geometry_);
    coverage_.setSurfaces(objects_.surfaces());
    map_changed_ = true;
    applyRestore();
    if ((now() - last_segmented_).seconds() >= resegment_period_s_ || rooms_.empty())
    {
        resegment();
    }
}

void WorldModelNode::onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth)
{
    ++frames_seen_;
    depth_history_.push(depth);
    pending_depth_.push_back(std::move(depth));
    while (pending_depth_.size() > 16)
    {
        pending_depth_.pop_front();
    }
}

void WorldModelNode::onColor(sensor_msgs::msg::Image::ConstSharedPtr color)
{
    color_history_.push(std::move(color));
}

void WorldModelNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
    if (info->k[0] <= 0.0 || info->k[4] <= 0.0)
    {
        return;
    }
    const Intrinsics intrinsics{ info->k[0],
                                 info->k[4],
                                 info->k[2],
                                 info->k[5],
                                 static_cast<int>(info->width),
                                 static_cast<int>(info->height) };
    if (!intrinsics_ || !(*intrinsics_ == intrinsics))
    {
        intrinsics_   = intrinsics;
        camera_known_ = false;
    }
}

void WorldModelNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry)
{
    const auto& twist  = odometry->twist.twist;
    const bool  moving = std::hypot(twist.linear.x, twist.linear.y) > still_linear_ ||
                        std::abs(twist.angular.z) > still_angular_;
    const double stamp = seconds(odometry->header.stamp);
    motion_.emplace_back(stamp, moving);
    while (!motion_.empty() && motion_.front().first < stamp - 30.0)
    {
        motion_.pop_front();
    }
}

void WorldModelNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
    // Held until its transform exists, like depth. One at a time: walls do not move, so the
    // sweeps that arrive meanwhile are dropped at no cost.
    if (pending_cloud_ == nullptr)
    {
        pending_cloud_ = cloud;
    }
}

void WorldModelNode::integrateCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
    const auto pose = mapFrom(cloud.header.frame_id, cloud.header.stamp);
    if (!pose)
    {
        return;
    }
    if (structure_hits_.size() != geometry_.cellCount())
    {
        structure_hits_.assign(geometry_.cellCount(), 0);
        band_clear_.assign(geometry_.cellCount(), 0);
        structure_cells_ = 0;
        cleared_cells_   = 0;
    }
    const Eigen::Isometry3f                      map_from_sensor = pose->cast<float>();
    const Eigen::Vector3f                        origin          = map_from_sensor.translation();
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    const auto                                   low  = static_cast<float>(structure_min_z_);
    const auto                                   high = static_cast<float>(structure_max_z_);
    const auto step  = static_cast<float>(0.5 * geometry_.resolution);
    const auto limit = static_cast<std::uint16_t>(std::numeric_limits<std::uint16_t>::max());
    for (; x != x.end(); ++x, ++y, ++z)
    {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
        {
            continue;
        }
        const Eigen::Vector3f point = map_from_sensor * Eigen::Vector3f(*x, *y, *z);
        const bool            wall  = point.z() >= low && point.z() <= high;
        if (wall)
        {
            const CellIndex cell = geometry_.toCell(point.x(), point.y());
            if (geometry_.contains(cell))
            {
                auto& hits = structure_hits_[static_cast<std::size_t>(geometry_.index(cell))];
                if (hits < limit)
                {
                    ++hits;
                    structure_cells_ += static_cast<int>(hits == structure_min_hits_);
                }
            }
        }

        // Where the ray runs through the band, nothing stood there: that is what tells a table
        // the ray passed over from a wall it has not reached yet.
        const Eigen::Vector3f ray   = point - origin;
        float                 enter = 0.0F;
        float                 leave = 1.0F;
        if (std::abs(ray.z()) > 1e-4F)
        {
            const float a = (low - origin.z()) / ray.z();
            const float b = (high - origin.z()) / ray.z();
            enter         = std::max(0.0F, std::min(a, b));
            leave         = std::min(1.0F, std::max(a, b));
        }
        else if (origin.z() < low || origin.z() > high)
        {
            continue;
        }
        const float across = ray.head<2>().norm();
        if (leave <= enter || across < 1e-3F)
        {
            continue;
        }
        // Stop short of the return itself, which may be the wall.
        const float stop =
            (leave * across) - (wall ? static_cast<float>(geometry_.resolution) : 0.0F);
        const float start = enter * across;
        const int   steps = static_cast<int>((stop - start) / step);
        int         last  = -1;
        for (int n = 0; n < steps; ++n)
        {
            const float     share = (start + (static_cast<float>(n) * step)) / across;
            const CellIndex cell =
                geometry_.toCell(origin.x() + (share * ray.x()), origin.y() + (share * ray.y()));
            if (!geometry_.contains(cell))
            {
                break;
            }
            const int index = geometry_.index(cell);
            if (index == last)
            {
                continue;
            }
            last        = index;
            auto& clear = band_clear_[static_cast<std::size_t>(index)];
            if (clear < limit)
            {
                ++clear;
                cleared_cells_ += static_cast<int>(clear == structure_min_clear_);
            }
        }
    }
    if (structure_cells_ >= structure_min_cells_)
    {
        RCLCPP_INFO_ONCE(get_logger(), "walls seen by the LiDAR: rooms are segmented on them now");
    }
}

cv::Mat WorldModelNode::structureCells() const
{
    if (!structure_enabled_ || structure_cells_ < structure_min_cells_ ||
        structure_hits_.size() != geometry_.cellCount())
    {
        return cells_;
    }
    cv::Mat walls(geometry_.height, geometry_.width, CV_8UC1, cv::Scalar(0));
    for (std::size_t index = 0; index < structure_hits_.size(); ++index)
    {
        if (structure_hits_[index] >= structure_min_hits_)
        {
            walls.ptr<std::uint8_t>(0)[index] = 255;
        }
    }
    // A wall's returns scatter a cell or two either side of the line the scan map drew.
    const int reach = std::max(1, geometry_.cellsFor(structure_dilate_));
    cv::dilate(
        walls,
        walls,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size((2 * reach) + 1, (2 * reach) + 1)));
    // Only cells the LiDAR has looked across at wall height: an unseen wall stays a wall.
    cv::Mat cleared(geometry_.height, geometry_.width, CV_8UC1, cv::Scalar(0));
    for (std::size_t index = 0; index < band_clear_.size(); ++index)
    {
        if (band_clear_[index] >= structure_min_clear_)
        {
            cleared.ptr<std::uint8_t>(0)[index] = 255;
        }
    }
    cv::Mat rooms = cells_.clone();
    rooms.setTo(kFree, (cells_ == kOccupied) & (walls == 0) & (cleared != 0));
    return rooms;
}

bool WorldModelNode::wasStill(double stamp) const
{
    if (!require_stillness_)
    {
        return true;
    }
    // Every sample over the settle window before the frame was still, and there were samples.
    bool covered = false;
    for (const auto& [time, moving] : motion_)
    {
        if (time < stamp - settle_s_ || time > stamp + 0.05)
        {
            continue;
        }
        if (moving)
        {
            return false;
        }
        covered = covered || time <= stamp - (0.8 * settle_s_);
    }
    return covered;
}

std::optional<Eigen::Isometry3d>
WorldModelNode::mapFrom(const std::string& frame, const builtin_interfaces::msg::Time& stamp) const
{
    try
    {
        return tf2::transformToEigen(
            tf_buffer_->lookupTransform(map_frame_, frame, rclcpp::Time(stamp)));
    }
    catch (const tf2::TransformException&)
    {
        return std::nullopt;
    }
}

std::optional<Pose2D> WorldModelNode::robotPose() const
{
    try
    {
        const auto transform =
            tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        const auto& q = transform.transform.rotation;
        return Pose2D{
            transform.transform.translation.x,
            transform.transform.translation.y,
            std::atan2(2.0 * ((q.w * q.z) + (q.x * q.y)), 1.0 - (2.0 * ((q.y * q.y) + (q.z * q.z))))
        };
    }
    catch (const tf2::TransformException&)
    {
        return std::nullopt;
    }
}

std::optional<DepthImage> WorldModelNode::depthView(const sensor_msgs::msg::Image& image)
{
    const std::size_t pixels = static_cast<std::size_t>(image.width) * image.height;
    if (image.encoding == "32FC1")
    {
        if (image.step % sizeof(float) != 0 ||
            image.data.size() < static_cast<std::size_t>(image.step) * image.height)
        {
            return std::nullopt;
        }
        return DepthImage{ reinterpret_cast<const float*>(image.data.data()),
                           static_cast<int>(image.width),
                           static_cast<int>(image.height),
                           image.step / sizeof(float) };
    }
    if (image.encoding == "16UC1")
    {
        // The real D435i: millimetres, 0 for no return.
        depth_scratch_.resize(pixels);
        for (std::uint32_t v = 0; v < image.height; ++v)
        {
            for (std::uint32_t u = 0; u < image.width; ++u)
            {
                std::uint16_t millimetres = 0;
                std::memcpy(
                    &millimetres,
                    image.data.data() + (static_cast<std::size_t>(v) * image.step) +
                        (static_cast<std::size_t>(u) * 2U),
                    sizeof(millimetres));
                depth_scratch_[(static_cast<std::size_t>(v) * image.width) + u] =
                    millimetres == 0 ? std::nanf("") : 1e-3F * static_cast<float>(millimetres);
            }
        }
        return DepthImage{ depth_scratch_.data(),
                           static_cast<int>(image.width),
                           static_cast<int>(image.height),
                           image.width };
    }
    RCLCPP_WARN_ONCE(get_logger(), "depth encoding '%s' is not handled", image.encoding.c_str());
    return std::nullopt;
}

void WorldModelNode::updateCameraModel(
    const std::string& frame, const builtin_interfaces::msg::Time& stamp)
{
    if (camera_known_ || !intrinsics_)
    {
        return;
    }
    try
    {
        const Eigen::Isometry3d base_from_camera = tf2::transformToEigen(
            tf_buffer_->lookupTransform(base_frame_, frame, rclcpp::Time(stamp)));
        const Eigen::Vector3d axis = base_from_camera.linear().col(2);
        CameraModel           model;
        model.height         = base_from_camera.translation().z();
        model.pitch          = std::asin(std::clamp(-axis.z(), -1.0, 1.0));
        model.horizontal_fov = 2.0 * std::atan(0.5 * intrinsics_->width / intrinsics_->fx);
        model.vertical_fov   = 2.0 * std::atan(0.5 * intrinsics_->height / intrinsics_->fy);
        planner_.setCamera(model);
        camera_known_ = true;
        RCLCPP_INFO(
            get_logger(),
            "camera: %.2f m up, pitched %.1f deg down, %.1f x %.1f deg",
            model.height,
            model.pitch * 180.0 / M_PI,
            model.horizontal_fov * 180.0 / M_PI,
            model.vertical_fov * 180.0 / M_PI);
    }
    catch (const tf2::TransformException& error)
    {
        // Tried again with the next frame.
        RCLCPP_DEBUG(get_logger(), "camera mount not on TF yet: %s", error.what());
    }
}

void WorldModelNode::integratePendingDepth()
{
    if (cells_.empty())
    {
        return;
    }
    const double horizon = now().seconds() - tf_wait_s_;
    if (pending_cloud_ != nullptr && seconds(pending_cloud_->header.stamp) <= horizon)
    {
        integrateCloud(*pending_cloud_);
        pending_cloud_.reset();
    }
    if (!intrinsics_)
    {
        return;
    }
    while (!pending_depth_.empty() && seconds(pending_depth_.front()->header.stamp) <= horizon)
    {
        const auto depth = pending_depth_.front();
        pending_depth_.pop_front();
        const double stamp = seconds(depth->header.stamp);
        updateCameraModel(depth->header.frame_id, depth->header.stamp);
        if (!wasStill(stamp))
        {
            continue;
        }
        const auto pose = mapFrom(depth->header.frame_id, depth->header.stamp);
        const auto view = depthView(*depth);
        if (!pose || !view)
        {
            continue;
        }
        if (coverage_.integrate(*view, *intrinsics_, *pose) > 0)
        {
            dirty_ = true;
        }
        ++frames_used_;
    }
}

void WorldModelNode::onMasks(const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks)
{
    if (cells_.empty() || !intrinsics_ || masks->instances.empty())
    {
        return;
    }
    const double stamp = seconds(masks->header.stamp);
    if (!wasStill(stamp))
    {
        return;  // Masks cut from a frame taken mid-turn would land the objects out of place.
    }
    const auto depth = depth_history_.at(stamp);
    if (depth == nullptr)
    {
        RCLCPP_DEBUG(get_logger(), "no depth frame for masks at %.3f", stamp);
        return;
    }
    const auto pose = mapFrom(depth->header.frame_id, depth->header.stamp);
    const auto view = depthView(*depth);
    if (!pose || !view || masks->image_width != depth->width || masks->image_height != depth->height)
    {
        return;
    }

    std::vector<MaskInput> inputs;
    inputs.reserve(masks->instances.size());
    for (const auto& instance : masks->instances)
    {
        if (instance.data.size() !=
            static_cast<std::size_t>(instance.roi.width) * instance.roi.height)
        {
            continue;
        }
        MaskInput input;
        input.label     = instance.label;
        input.score     = instance.score;
        input.x         = static_cast<int>(instance.roi.x_offset);
        input.y         = static_cast<int>(instance.roi.y_offset);
        input.width     = static_cast<int>(instance.roi.width);
        input.height    = static_cast<int>(instance.roi.height);
        input.mask      = instance.data.data();
        input.embedding = instance.embedding;
        inputs.push_back(std::move(input));
    }
    const FrameInput               frame{ stamp, *view, *intrinsics_, *pose };
    const std::vector<MaskOutcome> outcomes = objects_.integrate(inputs, frame);
    dirty_                                  = true;

    const auto color = color_history_.at(stamp);
    for (std::size_t slot = 0; slot < outcomes.size(); ++slot)
    {
        if (outcomes[slot].best_view && color != nullptr)
        {
            if (const MappedObject* object = objects_.find(outcomes[slot].object))
            {
                storeCrop(*object, *color, inputs[slot]);
            }
        }
    }
    coverage_.setSurfaces(objects_.surfaces());
}

void WorldModelNode::storeCrop(
    const MappedObject& object, const sensor_msgs::msg::Image& color, const MaskInput& mask)
{
    // A sliver seen at the image edge is not worth describing: shown little, a small VLM
    // invents something.
    if (std::min(mask.width, mask.height) < describe_min_crop_)
    {
        return;
    }
    const int pad = std::max(8, std::max(mask.width, mask.height) / 8);
    const cv::Rect area(mask.x - pad, mask.y - pad, mask.width + (2 * pad), mask.height + (2 * pad));
    cv::Mat crop = bgrArea(color, area);
    if (crop.empty())
    {
        return;
    }
    // The mask's own pixels on grey, the margin keeping the outline whole: the robot's arms hang
    // into the head camera's view, and a describer shown them names them instead.
    const cv::Rect kept =
        area & cv::Rect(0, 0, static_cast<int>(color.width), static_cast<int>(color.height));
    for (int row = 0; row < crop.rows; ++row)
    {
        auto*     pixel = crop.ptr<cv::Vec3b>(row);
        const int v     = kept.y + row - mask.y;
        for (int col = 0; col < crop.cols; ++col)
        {
            const int u = kept.x + col - mask.x;
            if (u < 0 || v < 0 || u >= mask.width || v >= mask.height ||
                mask.mask[(static_cast<std::size_t>(v) * mask.width) + u] == 0)
            {
                pixel[col] = cv::Vec3b(128, 128, 128);
            }
        }
    }
    std::vector<std::uint8_t> jpeg;
    cv::imencode(".jpg", crop, jpeg, { cv::IMWRITE_JPEG_QUALITY, 85 });
    crops_[object.id] = std::move(jpeg);
    described_.erase(object.id);  // A better view is worth another description.
}

void WorldModelNode::storeRoomView()
{
    const auto       robot = robotPose();
    const RoomState* room  = robot ? roomByLabel(roomLabelAt(robot->x, robot->y)) : nullptr;
    const auto       color = color_history_.atOrBefore(now().seconds());
    if (room == nullptr || color == nullptr)
    {
        return;
    }
    cv::Mat view = bgrArea(
        *color,
        cv::Rect(0, 0, static_cast<int>(color->width), static_cast<int>(color->height)));
    if (view.empty())
    {
        return;
    }
    // Half size: the server shrinks every image to 512 px anyway.
    cv::resize(view, view, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
    std::vector<std::uint8_t> jpeg;
    cv::imencode(".jpg", view, jpeg, { cv::IMWRITE_JPEG_QUALITY, 85 });
    auto& views = room_views_[room->id];
    views.push_back(std::move(jpeg));
    while (static_cast<int>(views.size()) > describe_room_views_)
    {
        views.pop_front();
    }
}

void WorldModelNode::onDescription(const g1_msgs::msg::Description::ConstSharedPtr& description)
{
    const std::string& subject = description->subject_id;
    if (description->task == g1_msgs::msg::DescribeRequest::TASK_ROOM)
    {
        for (RoomState& room : rooms_)
        {
            if (room.id == subject && !description->room_type.empty() &&
                room.type_source != "operator")
            {
                room.type            = description->room_type;
                room.type_confidence = description->confidence;
                room.type_source     = "describer";
                dirty_               = true;
            }
        }
        return;
    }
    if (subject.size() < 2 || subject[0] != 'O')
    {
        return;
    }
    int number = 0;
    if (std::from_chars(subject.data() + 1, subject.data() + subject.size(), number).ec !=
        std::errc{})
    {
        return;
    }
    MappedObject* object = objects_.find(number);
    if (object == nullptr)
    {
        return;
    }
    object->name    = description->name;
    object->caption = description->caption;
    described_.insert(object->id);
    dirty_ = true;
    if (!description->label_ok)
    {
        RCLCPP_INFO(
            get_logger(),
            "%s: the describer sees '%s', not '%s'",
            subject.c_str(),
            description->name.c_str(),
            object->label().c_str());
    }
}

// --- rooms ----------------------------------------------------------------------------------

void WorldModelNode::resegment()
{
    if (cells_.empty())
    {
        return;
    }
    last_segmented_            = now();
    structure_cells_segmented_ = structure_cells_ + cleared_cells_;
    map_changed_               = false;
    Segmentation segmentation  = segmentRooms(structureCells(), geometry_, segmentation_params_);

    const int              previous_count = static_cast<int>(rooms_.size());
    const std::vector<int> match          = matchRegions(
        room_labels_,
        previous_count,
        segmentation.labels,
        static_cast<int>(segmentation.regions.size()),
        0.2);

    std::vector<RoomState> rooms;
    rooms.reserve(segmentation.regions.size());
    for (std::size_t slot = 0; slot < segmentation.regions.size(); ++slot)
    {
        RoomState room;
        const int before = match[slot];
        if (before > 0 && before <= previous_count)
        {
            room = rooms_[static_cast<std::size_t>(before - 1)];
        }
        else
        {
            room.id   = "R" + std::to_string(next_room_);
            room.name = roomAlias(next_room_);
            ++next_room_;
        }
        room.region = segmentation.regions[slot];
        rooms.push_back(std::move(room));
    }
    rooms_       = std::move(rooms);
    room_labels_ = std::move(segmentation.labels);
    dirty_       = true;
    RCLCPP_INFO(get_logger(), "segmented %zu rooms", rooms_.size());
}

void WorldModelNode::applyRestore()
{
    if (!pending_restore_)
    {
        return;
    }
    WorldSnapshot snapshot = std::move(*pending_restore_);
    pending_restore_.reset();
    if (!worldFits(snapshot, cells_, geometry_))
    {
        RCLCPP_WARN(get_logger(), "the saved world was built on a different map; starting empty");
        return;
    }
    objects_.restore(std::move(snapshot.objects));
    coverage_.restoreLayers(
        snapshot.quality,
        snapshot.surface_quality,
        snapshot.flags,
        snapshot.directions);
    coverage_.setSurfaces(objects_.surfaces());
    resegment();
    // Names and types follow the room that now contains their point.
    for (const RoomRecord& record : snapshot.rooms)
    {
        const int label = roomLabelAt(record.x, record.y);
        if (label <= 0 || label > static_cast<int>(rooms_.size()))
        {
            continue;
        }
        RoomState& room      = rooms_[static_cast<std::size_t>(label - 1)];
        room.id              = record.id;
        room.name            = record.name;
        room.type            = record.type;
        room.type_confidence = record.type_confidence;
        room.type_source     = record.type_source;
    }
    next_room_ = std::max(next_room_, snapshot.next_room);
    // Crops from the last run; an object that already has a name needs no second description.
    for (const MappedObject& object : objects_.objects())
    {
        const std::filesystem::path path =
            std::filesystem::path(world_dir_) / "crops" / (objectId(object.id) + ".jpg");
        std::ifstream in(path, std::ios::binary);
        if (in)
        {
            crops_[object.id].assign(
                std::istreambuf_iterator<char>(in),
                std::istreambuf_iterator<char>());
        }
        if (!object.name.empty())
        {
            described_.insert(object.id);
        }
    }
    RCLCPP_INFO(
        get_logger(),
        "resumed %zu objects and the camera coverage",
        objects_.objects().size());
}

int WorldModelNode::roomLabelAt(double x, double y) const
{
    if (room_labels_.empty())
    {
        return 0;
    }
    const CellIndex cell = geometry_.toCell(x, y);
    // Furniture and walls sit on occupied cells, outside every room: take the nearest room.
    const int reach = geometry_.cellsFor(1.5);
    for (int radius = 0; radius <= reach; ++radius)
    {
        for (int dy = -radius; dy <= radius; ++dy)
        {
            for (int dx = -radius; dx <= radius; ++dx)
            {
                if (std::max(std::abs(dx), std::abs(dy)) != radius ||
                    !geometry_.contains(cell.x + dx, cell.y + dy))
                {
                    continue;
                }
                const int label = room_labels_.at<int>(cell.y + dy, cell.x + dx);
                if (label > 0)
                {
                    return label;
                }
            }
        }
    }
    return 0;
}

const WorldModelNode::RoomState* WorldModelNode::roomByLabel(int label) const
{
    return label > 0 && label <= static_cast<int>(rooms_.size()) ?
               &rooms_[static_cast<std::size_t>(label - 1)] :
               nullptr;
}

const WorldModelNode::RoomState* WorldModelNode::roomByText(const std::string& text) const
{
    const std::string wanted = lowered(text);
    for (const RoomState& room : rooms_)
    {
        if (lowered(room.id) == wanted || lowered(room.name) == wanted ||
            (!room.type.empty() && lowered(room.type) == wanted))
        {
            return &room;
        }
    }
    return nullptr;
}

void WorldModelNode::typeRooms()
{
    if (room_types_.types.empty())
    {
        return;
    }
    std::vector<std::vector<std::string>> labels(rooms_.size());
    for (const MappedObject& object : objects_.objects())
    {
        if (object.state != ObjectState::kActive)
        {
            continue;
        }
        const int label = roomLabelAt(object.box_centre.x(), object.box_centre.y());
        if (label > 0 && label <= static_cast<int>(rooms_.size()))
        {
            labels[static_cast<std::size_t>(label - 1)].push_back(object.label());
        }
    }
    for (std::size_t slot = 0; slot < rooms_.size(); ++slot)
    {
        RoomState& room = rooms_[slot];
        if (room.type_source == "operator" || room.type_source == "describer")
        {
            continue;
        }
        const RoomTyping typing =
            classifyRoom(room_types_, labels[slot], room.region.length, room.region.width);
        if (typing.type != room.type)
        {
            room.type            = typing.type;
            room.type_confidence = typing.probability;
            room.type_source     = typing.type.empty() ? "" : "objects";
            dirty_               = true;
        }
    }
}

// --- services -------------------------------------------------------------------------------

void WorldModelNode::onNextViewpoint(
    const g1_msgs::srv::NextViewpoint::Request::SharedPtr&  request,
    const g1_msgs::srv::NextViewpoint::Response::SharedPtr& response)
{
    using Response   = g1_msgs::srv::NextViewpoint::Response;
    const auto robot = robotPose();
    if (cells_.empty() || !robot)
    {
        response->status  = Response::STATUS_UNAVAILABLE;
        response->message = cells_.empty() ? "no map yet" : "no robot pose on TF yet";
        return;
    }
    if (map_changed_)
    {
        resegment();
    }
    const bool   frontier = request->mode == g1_msgs::srv::NextViewpoint::Request::MODE_FRONTIER;
    const auto   started  = std::chrono::steady_clock::now();
    const Plan   plan     = frontier ? planner_.nextFrontier(cells_, geometry_, *robot) :
                                       planner_.nextCoverage(coverage_, room_labels_, *robot);
    const double took_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

    response->message = plan.reason;
    switch (plan.status)
    {
        case PlanStatus::kDone:
            response->status = Response::STATUS_DONE;
            RCLCPP_INFO(
                get_logger(),
                "%s done: %s",
                frontier ? "frontier" : "coverage",
                plan.reason.c_str());
            dirty_ = true;
            return;
        case PlanStatus::kUnavailable:
            response->status = Response::STATUS_UNAVAILABLE;
            return;
        case PlanStatus::kViewpoint:
            break;
    }
    const Viewpoint& viewpoint     = plan.viewpoint;
    response->status               = Response::STATUS_VIEWPOINT;
    response->viewpoint_id         = viewpoint.id;
    response->pose.header.frame_id = map_frame_;
    response->pose.header.stamp    = now();
    response->pose.pose            = poseOf(
        viewpoint.x,
        viewpoint.y,
        0.0,
        viewpoint.headings.empty() ? 0.0 : viewpoint.headings.front());
    for (const double heading : viewpoint.headings)
    {
        response->headings.push_back(static_cast<float>(heading));
    }
    if (const RoomState* room = roomByLabel(roomLabelAt(viewpoint.x, viewpoint.y)))
    {
        response->room_id = room->id;
    }
    response->expected_gain = static_cast<float>(viewpoint.gain);
    RCLCPP_INFO(
        get_logger(),
        "viewpoint %u at (%.2f, %.2f), %zu headings, gain %.0f, cost %.1f s, planned in %.0f ms",
        viewpoint.id,
        viewpoint.x,
        viewpoint.y,
        viewpoint.headings.size(),
        viewpoint.gain,
        viewpoint.cost,
        took_ms);
}

void WorldModelNode::onReportViewpoint(
    const g1_msgs::srv::ReportViewpoint::Request::SharedPtr& request,
    const g1_msgs::srv::ReportViewpoint::Response::SharedPtr& /*response*/)
{
    planner_.report(coverage_, request->viewpoint_id, request->reached);
    // The last heading's frame, after its dwell: a settled, unblurred look at the room.
    if (describe_ && request->reached)
    {
        storeRoomView();
    }
    dirty_ = true;
}

std::vector<std::pair<const MappedObject*, double>> WorldModelNode::matchObjects(
    const std::string& query, const std::string& room,
    const std::vector<float>& query_embedding) const
{
    const RoomState* in_room = room.empty() ? nullptr : roomByText(room);
    std::vector<std::pair<const MappedObject*, double>> matches;
    if (!room.empty() && in_room == nullptr)
    {
        return matches;  // A room nobody knows: "not found", never "found somewhere else".
    }
    for (const MappedObject& object : objects_.objects())
    {
        if (object.state == ObjectState::kRemoved)
        {
            continue;
        }
        if (in_room != nullptr &&
            roomByLabel(roomLabelAt(object.box_centre.x(), object.box_centre.y())) != in_room)
        {
            continue;
        }
        double score = 0.0;
        if (lowered(query) == lowered(objectId(object.id)))
        {
            score = 1.0;
        }
        else if (!query.empty())
        {
            if (lowered(object.label()) == lowered(query) || lowered(object.name) == lowered(query))
            {
                score = 1.0;
            }
            else if (
                containsWord(object.label(), query) || containsWord(query, object.label()) ||
                containsWord(object.name, query))
            {
                score = 0.8;
            }
            else if (containsWord(object.caption, query))
            {
                score = 0.6;
            }
            else
            {
                // A runner-up label: worth something in proportion to its share of the votes.
                float total = 0.0F;
                for (const auto& [label, weight] : object.votes)
                {
                    total += weight;
                }
                const auto vote = object.votes.find(lowered(query));
                if (vote != object.votes.end() && total > 0.0F)
                {
                    score = 0.5 * static_cast<double>(vote->second / total);
                }
            }
        }
        if (!query_embedding.empty() && query_embedding.size() == object.embedding.size())
        {
            double cosine = 0.0;
            for (std::size_t i = 0; i < query_embedding.size(); ++i)
            {
                cosine += static_cast<double>(query_embedding[i]) * object.embedding[i];
            }
            score = std::max(score, std::clamp((cosine - 0.05) / 0.25, 0.0, 1.0));
        }
        if (object.state == ObjectState::kStale)
        {
            score *= 0.5;
        }
        if (score > 0.0)
        {
            matches.emplace_back(&object, score);
        }
    }
    std::sort(matches.begin(), matches.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
    });
    return matches;
}

void WorldModelNode::onFindObjects(
    const g1_msgs::srv::FindObjects::Request::SharedPtr&  request,
    const g1_msgs::srv::FindObjects::Response::SharedPtr& response)
{
    const auto matches = matchObjects(request->query, request->room, request->query_embedding);
    const auto limit   = request->max_results == 0 ? std::size_t{ 10 } :
                                                     static_cast<std::size_t>(request->max_results);
    for (std::size_t i = 0; i < matches.size() && i < limit; ++i)
    {
        response->objects.push_back(toMessage(*matches[i].first));
        response->scores.push_back(static_cast<float>(matches[i].second));
    }
    response->found = !matches.empty() && matches.front().second >= 0.5;

    // What share of the searched rooms the camera has seen: a "no" is only as good as this.
    const std::vector<CoverageTally> tallies =
        coverage_.tally(room_labels_, static_cast<int>(rooms_.size()));
    const RoomState* in_room = request->room.empty() ? nullptr : roomByText(request->room);
    double           seen    = 0.0;
    double           total   = 0.0;
    for (std::size_t slot = 0; slot < rooms_.size(); ++slot)
    {
        if (in_room != nullptr && in_room != &rooms_[slot])
        {
            continue;
        }
        const CoverageTally& tally = tallies[slot + 1];
        seen += tally.floor_seen + tally.face_seen;
        total += tally.floor + tally.face;
    }
    response->searched_coverage = total > 0.0 ? static_cast<float>(seen / total) : 0.0F;
    if (!request->room.empty() && in_room == nullptr)
    {
        response->message = "no room called '" + request->room + "'";
    }
    else if (!response->found)
    {
        response->message = "not in the " +
                            std::to_string(static_cast<int>(100.0 * response->searched_coverage)) +
                            "% of the searched area the camera has seen";
    }
}

void WorldModelNode::onGetApproachPose(
    const g1_msgs::srv::GetApproachPose::Request::SharedPtr&  request,
    const g1_msgs::srv::GetApproachPose::Response::SharedPtr& response)
{
    const auto robot = robotPose();
    if (cells_.empty() || !robot)
    {
        response->message = cells_.empty() ? "no map yet" : "no robot pose on TF yet";
        return;
    }
    planner_.prepare(cells_, geometry_, *robot);

    Footprint   footprint;
    std::string target_id;
    if (const RoomState* room = roomByText(request->target))
    {
        // A room: its most open spot, facing into it.
        const cv::Mat& clearance = planner_.clearance();
        double         best      = -1.0;
        CellIndex      spot;
        for (int y = 0; y < geometry_.height; ++y)
        {
            for (int x = 0; x < geometry_.width; ++x)
            {
                if (room_labels_.at<int>(y, x) == room->region.label &&
                    clearance.at<float>(y, x) > best &&
                    std::isfinite(
                        planner_.travel()[static_cast<std::size_t>(geometry_.index(x, y))]))
                {
                    best = clearance.at<float>(y, x);
                    spot = { x, y };
                }
            }
        }
        if (best < approach_params_.robot_radius)
        {
            response->message = room->name + " has no reachable open spot";
            return;
        }
        const double x                 = geometry_.centreX(spot.x);
        const double y                 = geometry_.centreY(spot.y);
        response->success              = true;
        response->target_id            = room->id;
        response->pose.header.frame_id = map_frame_;
        response->pose.header.stamp    = now();
        response->pose.pose            = poseOf(x, y, 0.0, std::atan2(y - robot->y, x - robot->x));
        response->message              = "the middle of " + room->name;
        return;
    }

    const auto          matches = matchObjects(request->target, request->room, {});
    const MappedObject* object  = nullptr;
    double              nearest = std::numeric_limits<double>::infinity();
    for (const auto& [candidate, score] : matches)
    {
        if (score < matches.front().second - 1e-6 || candidate->state != ObjectState::kActive)
        {
            continue;
        }
        // Among equally good matches, the one nearest the robot.
        const double distance =
            std::hypot(candidate->box_centre.x() - robot->x, candidate->box_centre.y() - robot->y);
        if (distance < nearest)
        {
            nearest = distance;
            object  = candidate;
        }
    }
    if (object == nullptr)
    {
        if (!request->room.empty() && roomByText(request->room) == nullptr)
        {
            response->message = "no room called '" + request->room + "'";
            return;
        }
        response->message = "nothing matching '" + request->target + "'" +
                            (request->room.empty() ? std::string{} : " in " + request->room) +
                            " is on the map";
        return;
    }
    footprint       = { object->box_centre, object->box_size, object->box_yaw };
    target_id       = objectId(object->id);
    const auto pose = approachPose(
        planner_.clearance(),
        planner_.travel(),
        geometry_,
        footprint,
        request->standoff,
        approach_params_);
    if (!pose)
    {
        response->message = "no reachable spot near " + target_id;
        return;
    }
    response->success              = true;
    response->target_id            = target_id;
    response->pose.header.frame_id = map_frame_;
    response->pose.header.stamp    = now();
    response->pose.pose            = poseOf(pose->x, pose->y, 0.0, pose->yaw);
    response->message              = "facing " + object->label() + " " + target_id;
}

void WorldModelNode::onSave(
    const std_srvs::srv::Trigger::Request::SharedPtr& /*request*/,
    const std_srvs::srv::Trigger::Response::SharedPtr& response)
{
    if (world_dir_.empty())
    {
        response->message = "world_dir is not set";
        return;
    }
    const std::string failure = saveNow();
    response->success         = failure.empty();
    response->message         = failure.empty() ? "saved to " + world_dir_ : failure;
}

std::string WorldModelNode::saveNow()
{
    if (world_dir_.empty() || cells_.empty())
    {
        return "nothing to save";
    }
    WorldSnapshot snapshot;
    snapshot.geometry = geometry_;
    snapshot.cells.assign(cells_.datastart, cells_.dataend);
    snapshot.next_room = next_room_;
    for (const RoomState& room : rooms_)
    {
        // A point inside the room: its peak clearance cell would do; its centroid usually does.
        double x = room.region.centroid_x;
        double y = room.region.centroid_y;
        if (roomLabelAt(x, y) != room.region.label)
        {
            for (int index = 0; index < static_cast<int>(geometry_.cellCount()); ++index)
            {
                if (room_labels_.ptr<int>(0)[index] == room.region.label)
                {
                    x = geometry_.centreX(index % geometry_.width);
                    y = geometry_.centreY(index / geometry_.width);
                    break;
                }
            }
        }
        snapshot.rooms.push_back(
            { room.id, room.name, room.type, room.type_confidence, room.type_source, x, y });
    }
    snapshot.objects         = objects_.objects();
    const auto layers        = coverage_.layers();
    snapshot.quality         = *layers[0];
    snapshot.surface_quality = *layers[1];
    snapshot.flags           = *layers[2];
    snapshot.directions      = *layers[3];
    std::string failure      = saveWorld(world_dir_, snapshot);
    if (failure.empty())
    {
        const std::filesystem::path crops = std::filesystem::path(world_dir_) / "crops";
        std::error_code             error;
        std::filesystem::create_directories(crops, error);
        for (const auto& [id, jpeg] : crops_)
        {
            std::ofstream out(crops / (objectId(id) + ".jpg"), std::ios::binary | std::ios::trunc);
            out.write(
                reinterpret_cast<const char*>(jpeg.data()),
                static_cast<std::streamsize>(jpeg.size()));
        }
        dirty_ = false;
    }
    return failure;
}

// --- publishing -----------------------------------------------------------------------------

g1_msgs::msg::WorldObject WorldModelNode::toMessage(const MappedObject& object) const
{
    g1_msgs::msg::WorldObject out;
    out.id         = objectId(object.id);
    out.label      = object.label();
    out.name       = object.name;
    out.caption    = object.caption;
    out.confidence = object.confidence();
    if (const RoomState* room =
            roomByLabel(roomLabelAt(object.box_centre.x(), object.box_centre.y())))
    {
        out.room_id = room->id;
    }
    out.support_id = object.support > 0 ? objectId(object.support) : std::string{};
    out.pose       = poseOf(
        object.box_centre.x(),
        object.box_centre.y(),
        0.5 * (object.z_min + object.z_max),
        object.box_yaw);
    out.size.x       = object.box_size.x();
    out.size.y       = object.box_size.y();
    out.size.z       = object.height();
    out.observations = static_cast<std::uint32_t>(object.observations);
    out.first_seen   = rclcpp::Time(static_cast<std::int64_t>(object.first_seen * 1e9));
    out.last_seen    = rclcpp::Time(static_cast<std::int64_t>(object.last_seen * 1e9));
    out.state        = static_cast<std::uint8_t>(object.state);
    return out;
}

void WorldModelNode::publishState()
{
    if (cells_.empty())
    {
        return;
    }
    // Walls keep arriving from the LiDAR while the robot explores; rooms follow them.
    const int  seen       = structure_cells_ + cleared_cells_;
    const bool walls_grew = structure_cells_ >= structure_min_cells_ &&
                            seen > structure_cells_segmented_ + (structure_cells_segmented_ / 20);

    if ((map_changed_ || walls_grew) && (now() - last_segmented_).seconds() >= resegment_period_s_)
    {
        resegment();
    }
    typeRooms();

    const auto                       stamp = now();
    const std::vector<CoverageTally> tallies =
        coverage_.tally(room_labels_, static_cast<int>(rooms_.size()));
    std::vector<int> object_counts(rooms_.size() + 1, 0);
    for (const MappedObject& object : objects_.objects())
    {
        if (object.state != ObjectState::kRemoved)
        {
            ++object_counts[static_cast<std::size_t>(std::clamp(
                roomLabelAt(object.box_centre.x(), object.box_centre.y()),
                0,
                static_cast<int>(rooms_.size())))];
        }
    }

    g1_msgs::msg::RoomArray rooms;
    rooms.header.stamp    = stamp;
    rooms.header.frame_id = map_frame_;
    for (std::size_t slot = 0; slot < rooms_.size(); ++slot)
    {
        const RoomState&     state = rooms_[slot];
        const CoverageTally& tally = tallies[slot + 1];
        g1_msgs::msg::Room   room;
        room.id              = state.id;
        room.name            = state.name;
        room.type            = state.type;
        room.type_confidence = static_cast<float>(state.type_confidence);
        room.type_source     = state.type_source;
        for (const cv::Point2d& corner : state.region.outline)
        {
            geometry_msgs::msg::Point32 point;
            point.x = static_cast<float>(corner.x);
            point.y = static_cast<float>(corner.y);
            room.outline.points.push_back(point);
        }
        room.centroid.x = state.region.centroid_x;
        room.centroid.y = state.region.centroid_y;
        room.area       = static_cast<float>(state.region.area);
        for (const RegionContact& contact : state.region.contacts)
        {
            g1_msgs::msg::Doorway doorway;
            if (const RoomState* other = roomByLabel(contact.to_label))
            {
                doorway.to_room = other->id;
            }
            doorway.centre.x = contact.x;
            doorway.centre.y = contact.y;
            doorway.width    = static_cast<float>(contact.width);
            room.doorways.push_back(doorway);
        }
        const auto share = [](int seen, int total) {
            return total > 0 ? static_cast<float>(seen) / static_cast<float>(total) : 0.0F;
        };
        room.floor_coverage     = share(tally.floor_seen, tally.floor);
        room.face_coverage      = share(tally.face_seen, tally.face);
        room.surface_coverage   = share(tally.surface_seen, tally.surface);
        room.unobservable_cells = static_cast<std::uint32_t>(tally.unobservable);
        room.object_count       = static_cast<std::uint32_t>(object_counts[slot + 1]);
        rooms.rooms.push_back(std::move(room));
    }
    rooms_pub_->publish(rooms);

    g1_msgs::msg::WorldObjectArray objects;
    objects.header = rooms.header;
    for (const MappedObject& object : objects_.objects())
    {
        if (object.state != ObjectState::kRemoved)
        {
            objects.objects.push_back(toMessage(object));
        }
    }
    objects_pub_->publish(objects);

    nav_msgs::msg::OccupancyGrid coverage;
    coverage.header                    = rooms.header;
    coverage.info.resolution           = static_cast<float>(geometry_.resolution);
    coverage.info.width                = static_cast<std::uint32_t>(geometry_.width);
    coverage.info.height               = static_cast<std::uint32_t>(geometry_.height);
    coverage.info.origin.position.x    = geometry_.origin_x;
    coverage.info.origin.position.y    = geometry_.origin_y;
    coverage.info.origin.orientation.w = 1.0;
    coverage.data                      = coverage_.qualityGrid();
    coverage_pub_->publish(coverage);

    publishMarkers(tallies);
}

void WorldModelNode::publishMarkers(const std::vector<CoverageTally>& tallies)
{
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker      clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const auto stamp = now();
    int        id    = 0;
    const auto base  = [&](const std::string& ns, int type) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id    = map_frame_;
        marker.header.stamp       = stamp;
        marker.ns                 = ns;
        marker.id                 = id++;
        marker.type               = type;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        return marker;
    };

    for (std::size_t slot = 0; slot < rooms_.size(); ++slot)
    {
        const RoomState& room    = rooms_[slot];
        auto             outline = base("rooms", visualization_msgs::msg::Marker::LINE_STRIP);
        outline.scale.x          = 0.04;
        outline.color.r          = 0.2F;
        outline.color.g          = 0.6F;
        outline.color.b          = 1.0F;
        outline.color.a          = 0.9F;
        for (const cv::Point2d& corner : room.region.outline)
        {
            geometry_msgs::msg::Point point;
            point.x = corner.x;
            point.y = corner.y;
            point.z = 0.02;
            outline.points.push_back(point);
        }
        if (!outline.points.empty())
        {
            outline.points.push_back(outline.points.front());
            markers.markers.push_back(outline);
        }

        const CoverageTally& tally = tallies[slot + 1];
        const int            seen  = tally.floor_seen + tally.face_seen;
        const int            total = std::max(1, tally.floor + tally.face);
        auto text = base("room_names", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        text.pose.position.x = room.region.centroid_x;
        text.pose.position.y = room.region.centroid_y;
        text.pose.position.z = 1.8;
        text.scale.z         = 0.35;
        text.color.r = text.color.g = text.color.b = 1.0F;
        text.color.a                               = 1.0F;
        text.text = room.name + (room.type.empty() ? "" : ": " + room.type) + "\n" +
                    std::to_string((100 * seen) / total) + "% seen";
        markers.markers.push_back(text);

        for (const RegionContact& contact : room.region.contacts)
        {
            auto door            = base("doorways", visualization_msgs::msg::Marker::CYLINDER);
            door.pose.position.x = contact.x;
            door.pose.position.y = contact.y;
            door.pose.position.z = 0.05;
            door.scale.x = door.scale.y = contact.width;
            door.scale.z                = 0.05;
            door.color.g                = 1.0F;
            door.color.a                = 0.5F;
            markers.markers.push_back(door);
        }
    }

    for (const MappedObject& object : objects_.objects())
    {
        if (object.state == ObjectState::kRemoved)
        {
            continue;
        }
        auto box = base("objects", visualization_msgs::msg::Marker::CUBE);
        box.pose = poseOf(
            object.box_centre.x(),
            object.box_centre.y(),
            0.5 * (object.z_min + object.z_max),
            object.box_yaw);
        box.scale.x = object.box_size.x();
        box.scale.y = object.box_size.y();
        box.scale.z = std::max(0.02, object.height());
        box.color.r = object.state == ObjectState::kStale ? 0.6F : 1.0F;
        box.color.g = 0.55F;
        box.color.b = 0.1F;
        box.color.a = 0.35F;
        markers.markers.push_back(box);

        auto label = base("object_names", visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        label.pose.position.x = object.box_centre.x();
        label.pose.position.y = object.box_centre.y();
        label.pose.position.z = object.z_max + 0.15;
        label.scale.z         = 0.15;
        label.color.r = label.color.g = label.color.b = 1.0F;
        label.color.a                                 = 1.0F;
        label.text =
            objectId(object.id) + " " + (object.name.empty() ? object.label() : object.name);
        markers.markers.push_back(label);
    }
    markers_pub_->publish(markers);
}

void WorldModelNode::requestDescriptions()
{
    if (!describe_)
    {
        return;
    }
    const double now_s       = now().seconds();
    const auto   asked_since = [&](const std::string& id) {
        const auto asked = requested_at_.find(id);
        return asked != requested_at_.end() && now_s - asked->second < describe_retry_s_;
    };

    // Rooms first: few, and a room's type steers every search in it. Only once the camera has
    // seen most of one, and only where its objects leave the type in doubt.
    std::vector<CoverageTally> tallies;
    for (std::size_t slot = 0; slot < rooms_.size(); ++slot)
    {
        const RoomState& room  = rooms_[slot];
        const auto       views = room_views_.find(room.id);
        if (views == room_views_.end() || room.type_source == "operator" ||
            room.type_source == "describer" ||
            (!room.type.empty() && room.type_confidence >= describe_rooms_below_) ||
            asked_since(room.id))
        {
            continue;
        }
        if (tallies.empty())
        {
            tallies = coverage_.tally(room_labels_, static_cast<int>(rooms_.size()));
        }
        const CoverageTally& tally = tallies[slot + 1];
        if (tally.floor == 0 ||
            static_cast<double>(tally.floor_seen) < describe_room_coverage_ * tally.floor)
        {
            continue;
        }
        g1_msgs::msg::DescribeRequest request;
        request.subject_id = room.id;
        request.task       = g1_msgs::msg::DescribeRequest::TASK_ROOM;
        for (const std::vector<std::uint8_t>& jpeg : views->second)
        {
            sensor_msgs::msg::CompressedImage image;
            image.format = "jpeg";
            image.data   = jpeg;
            request.images.push_back(std::move(image));
        }
        std::map<std::string, int> counts;
        for (const MappedObject& object : objects_.objects())
        {
            if (object.state == ObjectState::kActive &&
                roomLabelAt(object.box_centre.x(), object.box_centre.y()) ==
                    static_cast<int>(slot) + 1)
            {
                ++counts[object.label()];
            }
        }
        for (const auto& [label, count] : counts)
        {
            request.labels.push_back(label);
            request.votes.push_back(static_cast<float>(count));
        }
        describe_pub_->publish(request);
        requested_at_[room.id] = now_s;
        return;  // One at a time: the describer is rate limited anyway.
    }

    for (const MappedObject& object : objects_.objects())
    {
        const std::string id = objectId(object.id);
        if (object.state != ObjectState::kActive || object.observations < describe_after_ ||
            described_.contains(object.id) || !crops_.contains(object.id))
        {
            continue;
        }
        if (asked_since(id))
        {
            continue;
        }
        g1_msgs::msg::DescribeRequest request;
        request.subject_id = id;
        request.task       = g1_msgs::msg::DescribeRequest::TASK_OBJECT;
        sensor_msgs::msg::CompressedImage image;
        image.format = "jpeg";
        image.data   = crops_.at(object.id);
        request.images.push_back(std::move(image));
        for (const auto& [label, weight] : object.votes)
        {
            request.labels.push_back(label);
            request.votes.push_back(weight);
        }
        describe_pub_->publish(request);
        requested_at_[id] = now_s;
        return;  // One at a time: the describer is rate limited anyway.
    }
}

}  // namespace g1_world_model
