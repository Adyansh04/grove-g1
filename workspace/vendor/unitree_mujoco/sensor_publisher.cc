#include "sensor_publisher.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "sensor_frame.h"

namespace grove_g1
{
namespace
{

struct Config
{
    bool        enabled     = false;
    double      rate_hz     = 10.0;
    std::string socket_path = "/tmp/g1_sensors.sock";

    // Mid360 envelope. Resolution is a real budget, not a formality: 360x32 costs ~32 ms per
    // sweep against the G1 scene, so it is configurable and the timing gate decides what ships.
    // Which MuJoCo geom group the sweep sees. The scene puts walls, floor and obstacles in
    // group 3; the robot's own geoms are not in it. Without this every ray returns the
    // torso shell ~6 cm from the mount and the world is invisible.
    int    scene_geom_group = 3;
    int    azimuth_steps   = 360;
    int    elevation_steps = 32;
    double azimuth_min     = -M_PI;
    double azimuth_max     = M_PI;
    double elevation_min   = -0.12217305;  // -7 deg
    double elevation_max   = 0.90757121;   // +52 deg
    double range_min       = 0.1;
    double range_max       = 40.0;

    // mid360_joint in Unitree's vendored URDF, relative to torso_link. NOT the torso-folded
    // values g1_sim uses: that fold exists only because the sandbox body has no torso.
    double mount_xyz[3] = {0.0002835, 0.00003, 0.428434};
    double mount_rpy[3] = {M_PI, 0.05112069379091391, 0.0};
};

struct State
{
    std::thread       thread;
    std::atomic<bool> running{false};
};

State& state()
{
    // Deliberately leaked. unitree_mujoco's physics thread ends with exit(0), which runs
    // static destructors while our sampler thread is still running; destroying a joinable
    // std::thread calls std::terminate ("terminate called without an active exception").
    // The vendor's own bridge thread sidesteps this because main() ends in pthread_exit,
    // which skips destructors entirely. A never-destroyed singleton is the small fix.
    static State* s = new State();
    return *s;
}

// One line per ~5 s at 10 Hz. A missing relay is a normal condition and must neither spam
// nor be silent.
void logThrottled(const char* what, int& counter)
{
    if (counter++ % 50 == 0) {
        std::fprintf(stderr, "[grove_g1] %s\n", what);
    }
}

Config loadConfig()
{
    Config      cfg;
    const char* path = std::getenv("GROVE_G1_SENSOR_CONFIG");
    if (path == nullptr || *path == '\0') {
        return cfg;
    }
    try {
        const YAML::Node root = YAML::LoadFile(path);
        cfg.enabled           = root["enabled"] ? root["enabled"].as<bool>() : true;
        if (root["rate_hz"]) {
            cfg.rate_hz = root["rate_hz"].as<double>();
        }
        if (root["socket_path"]) {
            cfg.socket_path = root["socket_path"].as<std::string>();
        }
        if (root["azimuth_steps"]) {
            cfg.azimuth_steps = root["azimuth_steps"].as<int>();
        }
        if (root["elevation_steps"]) {
            cfg.elevation_steps = root["elevation_steps"].as<int>();
        }
        if (root["scene_geom_group"]) {
            cfg.scene_geom_group = root["scene_geom_group"].as<int>();
        }
        if (root["range_max"]) {
            cfg.range_max = root["range_max"].as<double>();
        }
    } catch (const std::exception& e) {
        // Loud, and still off: a malformed config must not look like a working sensor.
        std::fprintf(
            stderr, "[grove_g1] sensor config '%s' failed to load (%s); sensors DISABLED\n", path,
            e.what());
        cfg.enabled = false;
    }
    if (cfg.rate_hz <= 0.0 || cfg.azimuth_steps <= 0 || cfg.elevation_steps <= 0) {
        std::fprintf(stderr, "[grove_g1] sensor config has non-positive values; DISABLED\n");
        cfg.enabled = false;
    }
    return cfg;
}

// Extrinsic XYZ (URDF rpy) to a row-major 3x3.
void rpyToMatrix(const double rpy[3], double R[9])
{
    const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    R[0] = cy * cp;
    R[1] = cy * sp * sr - sy * cr;
    R[2] = cy * sp * cr + sy * sr;
    R[3] = sy * cp;
    R[4] = sy * sp * sr + cy * cr;
    R[5] = sy * sp * cr - cy * sr;
    R[6] = -sp;
    R[7] = cp * sr;
    R[8] = cp * cr;
}

void matrixToQuat(const double R[9], double q[4])
{
    const double t = R[0] + R[4] + R[8];
    if (t > 0.0) {
        const double s = std::sqrt(t + 1.0) * 2.0;
        q[0]           = 0.25 * s;
        q[1]           = (R[7] - R[5]) / s;
        q[2]           = (R[2] - R[6]) / s;
        q[3]           = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        const double s = std::sqrt(1.0 + R[0] - R[4] - R[8]) * 2.0;
        q[0]           = (R[7] - R[5]) / s;
        q[1]           = 0.25 * s;
        q[2]           = (R[1] + R[3]) / s;
        q[3]           = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        const double s = std::sqrt(1.0 + R[4] - R[0] - R[8]) * 2.0;
        q[0]           = (R[2] - R[6]) / s;
        q[1]           = (R[1] + R[3]) / s;
        q[2]           = 0.25 * s;
        q[3]           = (R[5] + R[7]) / s;
    } else {
        const double s = std::sqrt(1.0 + R[8] - R[0] - R[4]) * 2.0;
        q[0]           = (R[3] - R[1]) / s;
        q[1]           = (R[2] + R[6]) / s;
        q[2]           = (R[5] + R[7]) / s;
        q[3]           = 0.25 * s;
    }
}

// Never blocks, never raises SIGPIPE, never throws. A dead or slow relay costs one dropped
// frame, never a stalled simulator.
class RelaySocket
{
public:
    explicit RelaySocket(std::string path) : path_(std::move(path)) {}

    ~RelaySocket() { closeFd(); }

    void send(const SensorFrameHeader& header, const void* payload, std::size_t payload_bytes)
    {
        if (fd_ < 0 && !tryConnect()) {
            logThrottled("relay not connected; dropping frame", connect_log_);
            return;
        }
        if (!sendAll(&header, sizeof(header))) {
            return;
        }
        sendAll(payload, payload_bytes);
    }

private:
    bool tryConnect()
    {
        const int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
        if (fd < 0) {
            return false;
        }
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);

        if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 &&
            errno != EINPROGRESS) {
            ::close(fd);
            return false;
        }
        fd_ = fd;
        std::fprintf(stderr, "[grove_g1] connected to relay at %s\n", path_.c_str());
        return true;
    }

    // All-or-nothing by policy. A partial write desynchronises the length-prefixed stream,
    // and resynchronising is not worth the code, so the connection is dropped and remade.
    bool sendAll(const void* buf, std::size_t bytes)
    {
        const char* p    = static_cast<const char*>(buf);
        std::size_t sent = 0;
        while (sent < bytes) {
            // MSG_NOSIGNAL is load-bearing: without it a vanished relay raises SIGPIPE and
            // kills the simulator.
            const ssize_t n = ::send(fd_, p + sent, bytes - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n > 0) {
                sent += static_cast<std::size_t>(n);
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (sent == 0) {
                    logThrottled("relay slow; dropping frame", slow_log_);
                    return false;
                }
                closeFd();
                logThrottled("partial write; reconnecting", slow_log_);
                return false;
            }
            closeFd();
            logThrottled("relay disconnected; will retry", slow_log_);
            return false;
        }
        return true;
    }

    void closeFd()
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    std::string path_;
    int         fd_          = -1;
    int         connect_log_ = 0;
    int         slow_log_    = 0;
};

void sensorLoop(const Config cfg, mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    // The model is loaded by the physics thread after this one starts, same as the SDK
    // bridge's own wait.
    while (state().running.load(std::memory_order_relaxed) && *data == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!state().running.load(std::memory_order_relaxed)) {
        return;
    }

    mjModel*  m        = *model;
    const int torso_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
    if (torso_id < 0) {
        std::fprintf(stderr, "[grove_g1] no torso_link in the model; sensors DISABLED\n");
        return;
    }

    double R_mount[9];
    rpyToMatrix(cfg.mount_rpy, R_mount);

    mjtByte geomgroup[mjNGROUP] = {0};
    if (cfg.scene_geom_group >= 0 && cfg.scene_geom_group < mjNGROUP) {
        geomgroup[cfg.scene_geom_group] = 1;
    }

    const int n_rays = cfg.azimuth_steps * cfg.elevation_steps;
    std::fprintf(
        stderr, "[grove_g1] sensor thread up: %dx%d rays at %.1f Hz -> %s\n", cfg.azimuth_steps,
        cfg.elevation_steps, cfg.rate_hz, cfg.socket_path.c_str());

    // Ray directions in the sensor frame never change; build them once.
    std::vector<double> dirs(static_cast<std::size_t>(n_rays) * 3);
    for (int a = 0; a < cfg.azimuth_steps; ++a) {
        const double az =
            cfg.azimuth_min +
            (cfg.azimuth_max - cfg.azimuth_min) * a / static_cast<double>(cfg.azimuth_steps);
        for (int e = 0; e < cfg.elevation_steps; ++e) {
            const double el =
                cfg.elevation_steps == 1
                    ? cfg.elevation_min
                    : cfg.elevation_min + (cfg.elevation_max - cfg.elevation_min) * e /
                                              static_cast<double>(cfg.elevation_steps - 1);
            const std::size_t i = (static_cast<std::size_t>(e) * cfg.azimuth_steps + a) * 3;
            dirs[i + 0]         = std::cos(el) * std::cos(az);
            dirs[i + 1]         = std::cos(el) * std::sin(az);
            dirs[i + 2]         = std::sin(el);
        }
    }

    mjData*     snapshot = mj_makeData(m);
    RelaySocket relay(cfg.socket_path);

    std::vector<float> points(static_cast<std::size_t>(n_rays) * 3);
    std::vector<int>   geomid(static_cast<std::size_t>(n_rays));

    const auto period = std::chrono::duration<double>(1.0 / cfg.rate_hz);
    auto       next   = std::chrono::steady_clock::now();

    while (state().running.load(std::memory_order_relaxed)) {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

        // Snapshot under the lock, raycast outside it. Holding the lock across a ~32 ms
        // sweep would stall physics exactly as badly as running inline.
        double sim_time = 0.0;
        double torso_pos[3];
        double torso_mat[9];
        {
            std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
            mj_copyData(snapshot, m, *data);
            sim_time = snapshot->time;
            std::memcpy(torso_pos, snapshot->xpos + 3 * torso_id, sizeof(torso_pos));
            std::memcpy(torso_mat, snapshot->xmat + 9 * torso_id, sizeof(torso_mat));
        }

        // A snapshot taken before MuJoCo has run kinematics has an all-zero xmat, and
        // mj_ray aborts the whole process on a zero-length direction ("vector length is too
        // small"). Skip the cycle rather than hand it one.
        const double row0 = torso_mat[0] * torso_mat[0] + torso_mat[1] * torso_mat[1] +
                            torso_mat[2] * torso_mat[2];
        if (row0 < 0.5) {
            std::this_thread::sleep_until(next);
            continue;
        }

        // Sensor pose = torso pose composed with the fixed mount, taken live rather than
        // hardcoded: torso_link's height depends on the waist chain and the current stance,
        // so any baked-in constant is wrong the moment the robot walks.
        double origin[3];
        for (int r = 0; r < 3; ++r) {
            origin[r] = torso_pos[r] + torso_mat[3 * r + 0] * cfg.mount_xyz[0] +
                        torso_mat[3 * r + 1] * cfg.mount_xyz[1] +
                        torso_mat[3 * r + 2] * cfg.mount_xyz[2];
        }
        double R_sensor[9];
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                double acc = 0.0;
                for (int k = 0; k < 3; ++k) {
                    acc += torso_mat[3 * r + k] * R_mount[3 * k + c];
                }
                R_sensor[3 * r + c] = acc;
            }
        }

        for (int i = 0; i < n_rays; ++i) {
            const double* d = &dirs[static_cast<std::size_t>(i) * 3];
            double        world_dir[3];
            for (int r = 0; r < 3; ++r) {
                world_dir[r] = R_sensor[3 * r + 0] * d[0] + R_sensor[3 * r + 1] * d[1] +
                               R_sensor[3 * r + 2] * d[2];
            }
            // flg_static=1 so the world-body walls and floor are hit at all. geomgroup
            // restricts returns to the scene, keeping the robot from occluding itself.
            double dist = mj_ray(m, snapshot, origin, world_dir, geomgroup, 1, -1, &geomid[i]);
            if (dist < cfg.range_min || dist > cfg.range_max) {
                dist = 0.0;  // no return
            }
            const std::size_t o = static_cast<std::size_t>(i) * 3;
            points[o + 0]       = static_cast<float>(d[0] * dist);
            points[o + 1]       = static_cast<float>(d[1] * dist);
            points[o + 2]       = static_cast<float>(d[2] * dist);
        }

        SensorFrameHeader header{};
        header.magic         = kSensorFrameMagic;
        header.version       = kSensorFrameVersion;
        header.kind          = static_cast<uint32_t>(SensorFrameKind::PointCloud);
        header.payload_bytes = static_cast<uint32_t>(points.size() * sizeof(float));
        header.sim_time_s    = sim_time;
        std::memcpy(header.sensor_pos, origin, sizeof(origin));
        matrixToQuat(R_sensor, header.sensor_quat);
        header.point_count = static_cast<uint32_t>(n_rays);

        relay.send(header, points.data(), points.size() * sizeof(float));

        std::this_thread::sleep_until(next);
    }

    mj_deleteData(snapshot);
}

}  // namespace

void StartSensorPublisher(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    const Config cfg = loadConfig();
    if (!cfg.enabled || state().running.load()) {
        return;
    }
    state().running.store(true);
    state().thread = std::thread(sensorLoop, cfg, model, data, sim_mtx);
}

void StopSensorPublisher()
{
    if (!state().running.exchange(false)) {
        return;
    }
    if (state().thread.joinable()) {
        state().thread.join();
    }
}

}  // namespace grove_g1
