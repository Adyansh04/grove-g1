#ifndef G1_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
#define G1_WORLD_MODEL__WORLD_MODEL_NODE_HPP_

/**
 * @file world_model_node.hpp
 * @brief The world model on ROS: rooms, camera coverage and objects over the SLAM map.
 *
 * Inputs are the occupancy grid, aligned depth and colour, the detector's instance masks and
 * odometry. Frames count only when the base had been still for a while before they were taken:
 * the relay stamps images on arrival, so a frame taken while turning lands degrees off. Rooms are
 * segmented on the walls alone: LiDAR returns in a band above tables, sofas and beds mark what is
 * structure, and furniture that never reaches it stops cutting rooms into pieces. Outputs
 * are latched room and object arrays, a coverage grid, RViz markers, and the services the
 * exploration tree and the mission trees call. Everything runs on one callback group; the
 * heaviest call, a coverage plan, takes a fraction of a second.
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <g1_msgs/msg/describe_request.hpp>
#include <g1_msgs/msg/description.hpp>
#include <g1_msgs/msg/instance_mask_array.hpp>
#include <g1_msgs/msg/room_array.hpp>
#include <g1_msgs/msg/world_object_array.hpp>
#include <g1_msgs/srv/find_objects.hpp>
#include <g1_msgs/srv/get_approach_pose.hpp>
#include <g1_msgs/srv/next_viewpoint.hpp>
#include <g1_msgs/srv/report_viewpoint.hpp>
#include <map>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "g1_perception/depth_history.hpp"
#include "g1_world_model/approach_pose.hpp"
#include "g1_world_model/coverage_map.hpp"
#include "g1_world_model/object_map.hpp"
#include "g1_world_model/room_segmentation.hpp"
#include "g1_world_model/room_typing.hpp"
#include "g1_world_model/viewpoint_planner.hpp"
#include "g1_world_model/world_store.hpp"

namespace g1_world_model
{

class WorldModelNode : public rclcpp::Node
{
public:
    explicit WorldModelNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /// Saves the world one last time.
    ~WorldModelNode() override;

    WorldModelNode(const WorldModelNode&)            = delete;
    WorldModelNode& operator=(const WorldModelNode&) = delete;
    WorldModelNode(WorldModelNode&&)                 = delete;
    WorldModelNode& operator=(WorldModelNode&&)      = delete;

private:
    struct RoomState
    {
        std::string id;
        std::string name;
        std::string type;
        double      type_confidence = 0.0;
        std::string type_source;
        Region      region;
    };

    void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& map);
    void onDepth(sensor_msgs::msg::Image::ConstSharedPtr depth);
    void onColor(sensor_msgs::msg::Image::ConstSharedPtr color);
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
    void onMasks(const g1_msgs::msg::InstanceMaskArray::ConstSharedPtr& masks);
    void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry);
    void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
    void integrateCloud(const sensor_msgs::msg::PointCloud2& cloud);
    void onDescription(const g1_msgs::msg::Description::ConstSharedPtr& description);

    void onNextViewpoint(
        const g1_msgs::srv::NextViewpoint::Request::SharedPtr&  request,
        const g1_msgs::srv::NextViewpoint::Response::SharedPtr& response);
    void onReportViewpoint(
        const g1_msgs::srv::ReportViewpoint::Request::SharedPtr&  request,
        const g1_msgs::srv::ReportViewpoint::Response::SharedPtr& response);
    void onFindObjects(
        const g1_msgs::srv::FindObjects::Request::SharedPtr&  request,
        const g1_msgs::srv::FindObjects::Response::SharedPtr& response);
    void onGetApproachPose(
        const g1_msgs::srv::GetApproachPose::Request::SharedPtr&  request,
        const g1_msgs::srv::GetApproachPose::Response::SharedPtr& response);
    void onSave(
        const std_srvs::srv::Trigger::Request::SharedPtr&  request,
        const std_srvs::srv::Trigger::Response::SharedPtr& response);

    /// Integrates queued depth frames old enough for their transform to exist.
    void integratePendingDepth();
    void resegment();
    /// The map with furniture cleared: occupied cells no wall-height return backs up.
    [[nodiscard]] cv::Mat structureCells() const;
    void                  applyRestore();
    void                  typeRooms();
    void                  publishState();
    void                  publishMarkers(const std::vector<CoverageTally>& tallies);
    void                  requestDescriptions();
    std::string           saveNow();

    [[nodiscard]] bool wasStill(double stamp) const;
    [[nodiscard]] std::optional<Eigen::Isometry3d>
    mapFrom(const std::string& frame, const builtin_interfaces::msg::Time& stamp) const;
    [[nodiscard]] std::optional<Pose2D> robotPose() const;
    [[nodiscard]] int                   roomLabelAt(double x, double y) const;
    [[nodiscard]] const RoomState*      roomByLabel(int label) const;
    [[nodiscard]] const RoomState*      roomByText(const std::string& text) const;
    [[nodiscard]] std::vector<std::pair<const MappedObject*, double>> matchObjects(
        const std::string& query, const std::string& room,
        const std::vector<float>& query_embedding) const;
    [[nodiscard]] g1_msgs::msg::WorldObject toMessage(const MappedObject& object) const;
    [[nodiscard]] std::optional<DepthImage> depthView(const sensor_msgs::msg::Image& image);
    void updateCameraModel(const std::string& frame, const builtin_interfaces::msg::Time& stamp);
    void storeCrop(
        const MappedObject& object, const sensor_msgs::msg::Image& color, const MaskInput& mask);
    /// Keeps the newest colour frame as a view of the room the robot stands in.
    void storeRoomView();

    // Parameters.
    std::string map_frame_;
    std::string base_frame_;
    std::string world_dir_;
    bool        require_stillness_      = true;
    double      still_linear_           = 0.05;
    double      still_angular_          = 0.05;
    double      settle_s_               = 1.0;
    double      tf_wait_s_              = 0.2;
    double      resegment_period_s_     = 5.0;
    double      autosave_period_s_      = 60.0;
    bool        describe_               = false;
    int         describe_after_         = 3;
    double      describe_retry_s_       = 60.0;
    int         describe_min_crop_      = 32;
    int         describe_room_views_    = 3;
    double      describe_rooms_below_   = 0.8;
    double      describe_room_coverage_ = 0.8;
    bool        structure_enabled_      = true;
    double      structure_min_z_        = 1.4;
    double      structure_max_z_        = 2.2;
    int         structure_min_hits_     = 3;
    double      structure_dilate_       = 0.30;
    int         structure_min_cells_    = 200;
    int         structure_min_clear_    = 5;

    // The model.
    CoverageMap            coverage_;
    ObjectMap              objects_;
    ViewpointPlanner       planner_;
    RoomSegmentationParams segmentation_params_;
    ApproachParams         approach_params_;
    RoomTypeTable          room_types_;

    cv::Mat                      cells_;
    GridGeometry                 geometry_;
    bool                         map_changed_ = false;
    rclcpp::Time                 last_segmented_{ 0, 0, RCL_ROS_TIME };
    cv::Mat                      room_labels_;
    std::vector<RoomState>       rooms_;
    int                          next_room_ = 1;
    std::optional<WorldSnapshot> pending_restore_;
    std::vector<std::uint16_t>   structure_hits_;
    std::vector<std::uint16_t>   band_clear_;  // Rays that crossed the band above a cell.
    int                          cleared_cells_             = 0;
    int                          structure_cells_           = 0;
    int                          structure_cells_segmented_ = 0;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;

    std::optional<Intrinsics>                                    intrinsics_;
    bool                                                         camera_known_ = false;
    g1_perception::DepthHistory                                  depth_history_;
    g1_perception::DepthHistory                                  color_history_;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr>          pending_depth_;
    std::vector<float>                                           depth_scratch_;
    std::deque<std::pair<double, bool>>                          motion_;
    std::map<int, std::vector<std::uint8_t>>                     crops_;
    std::set<int>                                                described_;
    std::map<std::string, double>                                requested_at_;
    std::map<std::string, std::deque<std::vector<std::uint8_t>>> room_views_;  // JPEGs by room id.
    bool                                                         dirty_       = false;
    std::size_t                                                  frames_seen_ = 0;
    std::size_t                                                  frames_used_ = 0;

    // ROS plumbing.
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr    map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr         color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr    info_sub_;
    rclcpp::Subscription<g1_msgs::msg::InstanceMaskArray>::SharedPtr masks_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_sub_;
    rclcpp::Subscription<g1_msgs::msg::Description>::SharedPtr       description_sub_;

    rclcpp::Publisher<g1_msgs::msg::RoomArray>::SharedPtr              rooms_pub_;
    rclcpp::Publisher<g1_msgs::msg::WorldObjectArray>::SharedPtr       objects_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr         coverage_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<g1_msgs::msg::DescribeRequest>::SharedPtr        describe_pub_;

    rclcpp::Service<g1_msgs::srv::NextViewpoint>::SharedPtr   next_viewpoint_srv_;
    rclcpp::Service<g1_msgs::srv::ReportViewpoint>::SharedPtr report_viewpoint_srv_;
    rclcpp::Service<g1_msgs::srv::FindObjects>::SharedPtr     find_objects_srv_;
    rclcpp::Service<g1_msgs::srv::GetApproachPose>::SharedPtr approach_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr        save_srv_;

    rclcpp::TimerBase::SharedPtr integrate_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr describe_timer_;
    rclcpp::TimerBase::SharedPtr autosave_timer_;
};

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__WORLD_MODEL_NODE_HPP_
