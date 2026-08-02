#include "sensor_publisher.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace grove_g1
{
namespace
{

struct Config
{
    bool        enabled          = false;
    double      rate_hz          = 10.0;
    std::string node_name        = "g1_sim_sensors";
    // Delay after mjData appears before rclcpp::init(). The Unitree SDK calls
    // dds_create_domain EXPLICITLY and that fails if the domain already exists, so ROS must
    // not get there first. Its bridge thread polls for mjData every 500 ms and initialises
    // immediately after, so waiting past that hands the SDK the domain.
    double      sdk_settle_s     = 2.0;
};

struct State
{
    std::thread       thread;
    std::atomic<bool> running{false};
    bool              owns_rclcpp = false;
};

State& state()
{
    static State s;
    return s;
}

// Reads GROVE_G1_SENSOR_CONFIG. Absent or unreadable means sensors stay off, which is the
// default the locomotion suites run under.
Config loadConfig()
{
    Config cfg;
    const char* path = std::getenv("GROVE_G1_SENSOR_CONFIG");
    if (path == nullptr || *path == '\0') {
        return cfg;
    }
    try {
        const YAML::Node root = YAML::LoadFile(path);
        cfg.enabled = root["enabled"] ? root["enabled"].as<bool>() : true;
        if (root["rate_hz"]) {
            cfg.rate_hz = root["rate_hz"].as<double>();
        }
        if (root["node_name"]) {
            cfg.node_name = root["node_name"].as<std::string>();
        }
        if (root["sdk_settle_s"]) {
            cfg.sdk_settle_s = root["sdk_settle_s"].as<double>();
        }
    } catch (const std::exception& e) {
        // Loud, and still off: a malformed config must not look like a working sensor.
        std::fprintf(
            stderr, "[grove_g1] sensor config '%s' failed to load (%s); sensors DISABLED\n", path,
            e.what());
        cfg.enabled = false;
    }
    if (cfg.rate_hz <= 0.0) {
        std::fprintf(stderr, "[grove_g1] rate_hz must be positive; sensors DISABLED\n");
        cfg.enabled = false;
    }
    return cfg;
}

void sensorLoop(const Config cfg, int argc, char** argv, mjModel** model, mjData** data)
{
    // The model is loaded by the physics thread after this one starts, same as the SDK
    // bridge's own wait.
    while (state().running.load(std::memory_order_relaxed) && *data == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!state().running.load(std::memory_order_relaxed)) {
        return;
    }

    // Let the SDK claim the DDS domain before ROS touches it. See Config::sdk_settle_s.
    std::this_thread::sleep_for(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cfg.sdk_settle_s)));
    if (!state().running.load(std::memory_order_relaxed)) {
        return;
    }

    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
        state().owns_rclcpp = true;
    }
    auto node = std::make_shared<rclcpp::Node>(cfg.node_name);

    RCLCPP_INFO(
        node->get_logger(), "Sensor thread up at %.1f Hz on a model with %d geoms.", cfg.rate_hz,
        (*model)->ngeom);

    const auto period = std::chrono::duration<double>(1.0 / cfg.rate_hz);
    auto       next   = std::chrono::steady_clock::now();
    while (state().running.load(std::memory_order_relaxed) && rclcpp::ok()) {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        // Sweep and publish land here.
        std::this_thread::sleep_until(next);
    }
}

}  // namespace

void StartSensorPublisher(
    mjModel** model, mjData** data, std::recursive_mutex* sim_mtx, int argc, char** argv)
{
    (void)sim_mtx;

    const Config cfg = loadConfig();
    if (!cfg.enabled) {
        return;
    }
    if (state().running.load()) {
        return;
    }

    state().running.store(true);
    state().thread = std::thread(sensorLoop, cfg, argc, argv, model, data);
}

void StopSensorPublisher()
{
    if (!state().running.exchange(false)) {
        return;
    }
    if (state().thread.joinable()) {
        state().thread.join();
    }
    if (state().owns_rclcpp && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

}  // namespace grove_g1
