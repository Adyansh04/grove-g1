#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cv_bridge/cv_bridge.hpp>
#include <exception>
#include <filesystem>
#include <format>
#include <memory>
#include <mutex>
#include <optional>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sam3_inference_msgs/srv/segment_objects.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sam3_inference/request_checks.hpp"
#include "sam3_inference/sam3_engine.hpp"

namespace sam3_inference
{
namespace
{

// The preload runs once, right after the executor starts spinning.
constexpr std::chrono::milliseconds kPreloadDelay{ 100 };

rcl_interfaces::msg::ParameterDescriptor readOnly()
{
    rcl_interfaces::msg::ParameterDescriptor d;
    // Read once at start: a later set would report success and change nothing.
    d.read_only = true;
    return d;
}

}  // namespace

/**
 * @class Sam3SegmentationNode
 * @brief Serves SAM 3.1 open-vocabulary instance segmentation as the `segment_objects` service.
 * @details A request segments the image it carries, or, when `image_topic` is set, the newest image
 * of that topic, for each of its prompts. The engines load at start (`preload`) or on the first
 * request, and are freed after `idle_unload_s` without one (0: never) where the GPU is shared. A
 * failed load is answered with its reason until `load_retry_s` has passed. All parameters are
 * read-only.
 */
class Sam3SegmentationNode : public rclcpp::Node
{
public:
    using SegmentObjects = sam3_inference_msgs::srv::SegmentObjects;

    explicit Sam3SegmentationNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("sam3_segmentation", options)
    {
        config_.model_dir =
            declare_parameter<std::string>("model_dir", "/root/data/models/sam3", readOnly());
        config_.engine_dir = declare_parameter<std::string>(
            "engine_dir",
            "/root/data/models/sam3/engines",
            readOnly());
        config_.suffix = declare_parameter<std::string>("precision_suffix", ".bf16", readOnly());
        config_.workspace_bytes =
            static_cast<std::size_t>(
                declare_parameter<std::int64_t>("engine_workspace_mb", 2048, readOnly()))
            << 20;
        config_.max_cached_prompts = static_cast<std::size_t>(
            declare_parameter<std::int64_t>("max_cached_prompts", 64, readOnly()));
        std::string vocabulary = declare_parameter<std::string>("vocabulary", "", readOnly());
        if (vocabulary.empty())
        {
            vocabulary = (std::filesystem::path(
                              ament_index_cpp::get_package_share_directory("sam3_inference")) /
                          "resources" / "bpe_simple_vocab_16e6.txt.gz")
                             .string();
        }
        config_.vocabulary_path = vocabulary;
        min_score_ = static_cast<float>(declare_parameter<double>("min_score", 0.5, readOnly()));
        max_image_age_s_          = declare_parameter<double>("max_image_age_s", 1.0, readOnly());
        idle_unload_s_            = declare_parameter<double>("idle_unload_s", 0.0, readOnly());
        load_retry_s_             = declare_parameter<double>("load_retry_s", 60.0, readOnly());
        const double idle_check_s = declare_parameter<double>("idle_check_s", 5.0, readOnly());
        if (config_.workspace_bytes == 0 || max_image_age_s_ <= 0.0 || idle_unload_s_ < 0.0 ||
            load_retry_s_ < 0.0 || idle_check_s <= 0.0)
        {
            throw std::invalid_argument(
                "sam3_segmentation: engine_workspace_mb and max_image_age_s must be positive, "
                "idle_unload_s and load_retry_s not negative, idle_check_s positive");
        }

        service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        const std::string image_topic =
            declare_parameter<std::string>("image_topic", "", readOnly());
        if (!image_topic.empty())
        {
            image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions sub;
            sub.callback_group = image_group_;
            image_sub_         = create_subscription<sensor_msgs::msg::Image>(
                image_topic,
                rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                    const std::scoped_lock lock(image_mutex_);
                    latest_ = std::move(msg);
                },
                sub);
        }
        service_ = create_service<SegmentObjects>(
            declare_parameter<std::string>("service", "segment_objects", readOnly()),
            [this](
                const std::shared_ptr<SegmentObjects::Request>&  req,
                const std::shared_ptr<SegmentObjects::Response>& res) { onSegment(req, *res); },
            rclcpp::ServicesQoS(),
            service_group_);
        if (idle_unload_s_ > 0.0)
        {
            idle_timer_ = create_wall_timer(
                std::chrono::duration<double>(idle_check_s),
                [this]() {
                    const std::scoped_lock lock(engine_mutex_);
                    if (engine_ && std::chrono::steady_clock::now() - last_used_ >
                                       std::chrono::duration<double>(idle_unload_s_))
                    {
                        engine_.reset();
                        RCLCPP_INFO(
                            get_logger(),
                            "SAM 3.1 idle for %.0f s: engines freed",
                            idle_unload_s_);
                    }
                },
                service_group_);
        }
        if (declare_parameter<bool>("preload", true, readOnly()))
        {
            preload_timer_ = create_wall_timer(
                kPreloadDelay,
                [this]() {
                    preload_timer_->cancel();
                    const std::scoped_lock lock(engine_mutex_);
                    std::string            why;
                    if (!engine_ && !loadEngine(why))
                    {
                        RCLCPP_ERROR(get_logger(), "SAM 3.1 engines did not load: %s", why.c_str());
                    }
                    last_used_ = std::chrono::steady_clock::now();
                },
                service_group_);
        }
        RCLCPP_INFO(
            get_logger(),
            "SAM 3.1 segmentation ready; engines from %s",
            config_.engine_dir.c_str());
    }

private:
    // Called with engine_mutex_ held. A failure is kept for load_retry_s, so requests that waited
    // for a long failed build do not start it again one by one.
    [[nodiscard]] bool loadEngine(std::string& why)
    {
        const auto now = std::chrono::steady_clock::now();
        if (load_error_ && now - load_failed_at_ < std::chrono::duration<double>(load_retry_s_))
        {
            why = *load_error_;
            return false;
        }
        try
        {
            engine_ = std::make_unique<Sam3Engine>(config_);
        }
        catch (const std::exception& e)
        {
            load_error_ = e.what();
            // When it ended: a long failed build must not count as long ago for those it kept waiting.
            load_failed_at_ = std::chrono::steady_clock::now();
            why             = e.what();
            return false;
        }
        load_error_.reset();
        RCLCPP_INFO(
            get_logger(),
            "SAM 3.1 engines loaded in %.1f s",
            std::chrono::duration<double>(std::chrono::steady_clock::now() - now).count());
        return true;
    }

    void
    onSegment(const std::shared_ptr<SegmentObjects::Request>& req, SegmentObjects::Response& res)
    {
        res.success = false;
        try
        {
            serve(req, res);
        }
        catch (const CudaFailure& e)
        {
            res.success = false;
            res.message = std::format("SAM 3.1 failed: {}", e.what());
            RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
            // The error stays for this process's life: launch's respawn starts a clean one, a
            // second from now, once this request has its answer.
            if (!e.outOfMemory())
            {
                exit_timer_ =
                    create_wall_timer(std::chrono::seconds(1), [] { std::_Exit(EXIT_FAILURE); });
            }
        }
        catch (const std::exception& e)
        {
            res.success = false;
            res.message = std::format("SAM 3.1 failed: {}", e.what());
            RCLCPP_ERROR(get_logger(), "%s", res.message.c_str());
        }
    }

    void serve(const std::shared_ptr<SegmentObjects::Request>& req, SegmentObjects::Response& res)
    {
        if (const auto why = checkPrompts(req->prompts))
        {
            res.message = *why;
            return;
        }
        // The request's own image, shared with the request instead of copied.
        sensor_msgs::msg::Image::ConstSharedPtr image;
        if (!req->image.data.empty())
        {
            image = sensor_msgs::msg::Image::ConstSharedPtr(req, &req->image);
        }
        else if (!image_sub_)
        {
            res.message = "no image in the request, and no image_topic is configured";
            return;
        }
        else
        {
            {
                const std::scoped_lock lock(image_mutex_);
                image = latest_;
            }
            if (!image)
            {
                res.message = std::format(
                    "no image in the request and none received on {}",
                    image_sub_->get_topic_name());
                return;
            }
            const auto age_s =
                imageAgeS(image->header.stamp.sec, image->header.stamp.nanosec, now().seconds());
            if (!age_s || *age_s > max_image_age_s_)
            {
                res.message = std::format(
                    "the newest image is older than {:.1f} s or has a negative stamp",
                    max_image_age_s_);
                return;
            }
        }
        cv_bridge::CvImageConstPtr rgb;
        try
        {
            rgb = cv_bridge::toCvShare(image, "rgb8");
        }
        catch (const cv_bridge::Exception& e)
        {
            res.message = std::format("cannot read the image: {}", e.what());
            return;
        }
        // service_group_ already serialises the service and the timers; the lock stays in case
        // the group changes.
        const std::scoped_lock lock(engine_mutex_);
        std::string            why;
        if (!engine_ && !loadEngine(why))
        {
            res.message = std::format("SAM 3.1 engines did not load: {}", why);
            return;
        }
        const float                 min_score = req->min_score > 0.0F ? req->min_score : min_score_;
        const std::vector<Instance> instances =
            engine_->segment(rgb->image, req->prompts, min_score);
        last_used_ = std::chrono::steady_clock::now();
        res.header = image->header;
        res.segments.reserve(instances.size());
        for (const Instance& instance : instances)
        {
            sam3_inference_msgs::msg::Segment& segment = res.segments.emplace_back();
            segment.label                              = instance.label;
            segment.score                              = instance.score;
            segment.roi.x_offset                       = static_cast<std::uint32_t>(instance.box.x);
            segment.roi.y_offset                       = static_cast<std::uint32_t>(instance.box.y);
            segment.roi.width  = static_cast<std::uint32_t>(instance.box.width);
            segment.roi.height = static_cast<std::uint32_t>(instance.box.height);
            cv_bridge::CvImage(image->header, "mono8", instance.mask).toImageMsg(segment.mask);
        }
        const Timing& t = engine_->timing();
        RCLCPP_INFO(
            get_logger(),
            "%zu instances of %zu prompts: preprocess %.1f, image %.1f, text %.1f, decoder "
            "%.1f, post %.1f ms",
            instances.size(),
            req->prompts.size(),
            t.preprocess_ms,
            t.image_encoder_ms,
            t.text_encoder_ms,
            t.decoder_ms,
            t.postprocess_ms);
        res.success = true;
        res.message = std::format("{} instances", instances.size());
    }

    EngineConfig                                             config_;
    float                                                    min_score_{ 0.5F };
    double                                                   max_image_age_s_{ 1.0 };
    double                                                   idle_unload_s_{ 0.0 };
    rclcpp::TimerBase::SharedPtr                             exit_timer_;
    double                                                   load_retry_s_{ 60.0 };
    rclcpp::TimerBase::SharedPtr                             preload_timer_;
    rclcpp::CallbackGroup::SharedPtr                         image_group_;
    rclcpp::CallbackGroup::SharedPtr                         service_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<SegmentObjects>::SharedPtr               service_;
    rclcpp::TimerBase::SharedPtr                             idle_timer_;
    // Guards latest_, written by the image callback and read by the service.
    std::mutex                              image_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr latest_;
    // Guards engine_, last_used_, load_error_ and load_failed_at_, used by the service and the
    // timers (all in service_group_, which already runs one at a time).
    std::mutex                            engine_mutex_;
    std::unique_ptr<Sam3Engine>           engine_;
    std::chrono::steady_clock::time_point last_used_{ std::chrono::steady_clock::now() };
    std::optional<std::string>            load_error_;
    std::chrono::steady_clock::time_point load_failed_at_;
};

}  // namespace sam3_inference

RCLCPP_COMPONENTS_REGISTER_NODE(sam3_inference::Sam3SegmentationNode)
