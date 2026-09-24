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
#include <algorithm>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <GLFW/glfw3.h>

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

    // Geom group the sweep sees. Scene geometry is group 2: unmasked, rays hit the robot's own
    // torso shell, and group 3 would be hit by rays but never drawn by the viewer.
    int    scene_geom_group = 2;

    bool camera_enabled = false;
    int  camera_width   = 848;
    int  camera_height  = 480;

    // Bodies whose ground-truth pose is published for the sim-only object source. An explicit
    // list, never "every free body": the pelvis has a free joint too. Empty turns it off.
    std::vector<std::string> object_bodies;
    // Mid360 envelope. Resolution is a real cost: 360x32 takes ~32 ms per sweep of the G1 scene.
    int    azimuth_steps   = 360;
    int    elevation_steps = 32;
    double azimuth_min     = -M_PI;
    double azimuth_max     = M_PI;
    double elevation_min   = -0.12217305;  // -7 deg
    double elevation_max   = 0.90757121;   // +52 deg
    double range_min       = 0.1;
    double range_max       = 40.0;

    // The IMU inside the Mid360, at the real unit's rate.
    double imu_rate_hz = 200.0;
};

// Mount poses relative to torso_link, copied from mid360_joint and d435_joint in Unitree's
// vendored URDF. Constants, not Config: the URDF owns them, and these move when it does.
constexpr double kMountXyz[3] = {0.0002835, 0.00003, 0.428434};
constexpr double kMountRpy[3] = {M_PI, 0.05112069379091391, 0.0};
constexpr double kCamXyz[3]   = {0.0576235, 0.01753, 0.42987};
constexpr double kCamRpy[3]   = {0.0, 0.8307767239493009, 0.0};

class RelaySocket;

struct State
{
    std::thread                  thread;
    std::thread                  imu_thread;
    std::thread                  base_thread;
    std::unique_ptr<RelaySocket> relay;
    std::atomic<bool>            running{false};
};

State& state()
{
    // Leaked on purpose: the physics thread ends in exit(0), and a static destructor
    // destroying these still-joinable threads would call std::terminate.
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
        if (root["camera_enabled"]) {
            cfg.camera_enabled = root["camera_enabled"].as<bool>();
        }
        if (root["range_max"]) {
            cfg.range_max = root["range_max"].as<double>();
        }
        if (root["imu_rate_hz"]) {
            cfg.imu_rate_hz = root["imu_rate_hz"].as<double>();
        }
        if (root["object_bodies"]) {
            cfg.object_bodies = root["object_bodies"].as<std::vector<std::string>>();
        }
    } catch (const std::exception& e) {
        // Loud, and still off: a malformed config must not look like a working sensor.
        std::fprintf(
            stderr, "[grove_g1] sensor config '%s' failed to load (%s); sensors DISABLED\n", path,
            e.what());
        cfg.enabled = false;
    }
    if (cfg.rate_hz <= 0.0 || cfg.imu_rate_hz <= 0.0 || cfg.azimuth_steps <= 0 ||
        cfg.elevation_steps <= 0) {
        std::fprintf(stderr, "[grove_g1] sensor config has non-positive values; DISABLED\n");
        cfg.enabled = false;
    }
    return cfg;
}

// Axis-aligned extents of a body's own geometry, as full widths in the body frame. Each geom's
// box is taken about the body origin, so an off-centre geom still fits inside.
void bodyExtents(const mjModel* m, int body, double out[3])
{
    out[0] = out[1] = out[2] = 0.0;
    for (int i = 0; i < m->body_geomnum[body]; ++i)
    {
        const int     geom = m->body_geomadr[body] + i;
        const mjtNum* size = m->geom_size + 3 * geom;
        // geom_size means something different per type; an unhandled type contributes nothing
        // rather than a wrong number.
        double half[3] = { 0.0, 0.0, 0.0 };
        switch (m->geom_type[geom])
        {
            case mjGEOM_BOX:
                half[0] = size[0];
                half[1] = size[1];
                half[2] = size[2];
                break;
            case mjGEOM_SPHERE:
                half[0] = half[1] = half[2] = size[0];
                break;
            case mjGEOM_CYLINDER:
            case mjGEOM_CAPSULE:
                half[0] = half[1] = size[0];
                half[2] = size[1];
                break;
            default:
                continue;
        }
        const mjtNum* pos = m->geom_pos + 3 * geom;
        for (int a = 0; a < 3; ++a)
        {
            out[a] = std::max(out[a], 2.0 * (std::abs(pos[a]) + half[a]));
        }
    }
}

// Resolves the tracked bodies against the current model. A missing body is normal (not every
// scene has the props), so it is logged once and skipped.
void resolveObjectBodies(
    const mjModel* m, const std::vector<std::string>& names, std::vector<int>& ids,
    std::vector<ObjectPoseRecord>& records)
{
    ids.clear();
    records.clear();
    for (const std::string& name : names) {
        if (name.size() >= sizeof(ObjectPoseRecord::name)) {
            std::fprintf(
                stderr, "[grove_g1] object body '%s' exceeds %zu chars; NOT tracked\n",
                name.c_str(), sizeof(ObjectPoseRecord::name) - 1);
            continue;
        }
        const int id = mj_name2id(m, mjOBJ_BODY, name.c_str());
        if (id < 0) {
            std::fprintf(
                stderr, "[grove_g1] no body '%s' in this scene; not tracked\n", name.c_str());
            continue;
        }
        ids.push_back(id);
        ObjectPoseRecord record{};
        std::strncpy(record.name, name.c_str(), sizeof(record.name) - 1);
        // Geometry is fixed for the run, so it is measured here rather than every cycle.
        bodyExtents(m, id, record.size);
        records.push_back(record);
    }
    if (!ids.empty()) {
        std::fprintf(stderr, "[grove_g1] tracking %zu object bodies\n", ids.size());
    }
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

// Non-blocking, no SIGPIPE, no exceptions: a dead relay costs frames, never the simulator. One
// connection for all sender threads keeps frames in order; small streams use trySend().
class RelaySocket
{
public:
    explicit RelaySocket(std::string path) : path_(std::move(path)) {}

    ~RelaySocket() { closeFd(); }

    void send(const SensorFrameHeader& header, const void* payload, std::size_t payload_bytes)
    {
        const std::lock_guard<std::mutex> guard(send_mtx_);
        sendLocked(header, payload, payload_bytes);
    }

    /// Drops the frame rather than waiting for another thread's send to finish.
    void trySend(const SensorFrameHeader& header, const void* payload, std::size_t payload_bytes)
    {
        const std::unique_lock<std::mutex> guard(send_mtx_, std::try_to_lock);
        if (!guard.owns_lock()) {
            return;
        }
        sendLocked(header, payload, payload_bytes);
    }

private:
    void sendLocked(const SensorFrameHeader& header, const void* payload,
                    std::size_t payload_bytes)
    {
        if (fd_ < 0 && !tryConnect()) {
            logThrottled("relay not connected; dropping frame", connect_log_);
            return;
        }
        if (!sendAll(&header, sizeof(header), /*frame_started=*/false)) {
            return;
        }
        // The header is out, so the body must follow or the relay reads the next frame's bytes
        // as this one's payload.
        sendAll(payload, payload_bytes, /*frame_started=*/true);
    }

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
        // A depth+colour frame is ~2.9 MB. SO_SNDBUF is silently clamped to 2*net.core.wmem_max,
        // so SO_SNDBUFFORCE goes first. It needs CAP_NET_ADMIN; without it, sendAll's retry copes.
        const int snd = 4 * 1024 * 1024;
        if (::setsockopt(fd, SOL_SOCKET, SO_SNDBUFFORCE, &snd, sizeof(snd)) != 0) {
            ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof(snd));
        }

        fd_ = fd;
        std::fprintf(stderr, "[grove_g1] connected to relay at %s\n", path_.c_str());
        return true;
    }

    // All-or-nothing by policy. A partial write desynchronises the length-prefixed stream,
    // and resynchronising is not worth the code, so the connection is dropped and remade.
    bool sendAll(const void* buf, std::size_t bytes, bool frame_started)
    {
        const char* p        = static_cast<const char*>(buf);
        std::size_t sent     = 0;
        // Per send call, so a stalled relay can hold a cycle for several of these. Every send
        // happens off the sim lock, so only the sensor rate drops, never physics.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);
        while (sent < bytes) {
            // MSG_NOSIGNAL is load-bearing: without it a vanished relay raises SIGPIPE and
            // kills the simulator.
            const ssize_t n = ::send(fd_, p + sent, bytes - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
            if (n > 0) {
                sent += static_cast<std::size_t>(n);
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                if (sent == 0 && !frame_started) {
                    // None of this frame is on the wire yet, so dropping it is clean.
                    logThrottled("relay slow; dropping frame", slow_log_);
                    return false;
                }
                // EAGAIN mid-frame is normal for a frame bigger than the socket buffer: wait for
                // the relay to drain, and past the deadline reset rather than desynchronise.
                if (std::chrono::steady_clock::now() < deadline) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
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
    std::mutex  send_mtx_;
    int         fd_          = -1;
    int         connect_log_ = 0;
    int         slow_log_    = 0;
};

void sensorLoop(const Config cfg, mjModel** model, mjData** data, std::recursive_mutex* sim_mtx,
                RelaySocket* relay_socket)
{
    // The model is loaded by the physics thread after this one starts, same as the SDK
    // bridge's own wait.
    while (state().running.load(std::memory_order_relaxed) && *data == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!state().running.load(std::memory_order_relaxed)) {
        return;
    }

    mjModel* m        = *model;
    int      torso_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
    if (torso_id < 0) {
        std::fprintf(stderr, "[grove_g1] no torso_link in the model; sensors DISABLED\n");
        return;
    }

    double R_mount[9];
    rpyToMatrix(kMountRpy, R_mount);
    double R_cam_mount[9];
    rpyToMatrix(kCamRpy, R_cam_mount);
    int cam_id = mj_name2id(m, mjOBJ_CAMERA, "d435i");

    std::vector<int>              object_ids;
    std::vector<ObjectPoseRecord> object_records;
    resolveObjectBodies(m, cfg.object_bodies, object_ids, object_records);

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

    mjData*      snapshot = mj_makeData(m);
    RelaySocket& relay    = *relay_socket;

    // Offscreen render state in this thread's own GL context. glfwCreateWindow off the main
    // thread works here despite the GLFW docs.
    mjvScene    cam_scn;
    mjrContext  cam_con;
    mjvOption   cam_opt;
    mjvCamera   cam_cam;
    GLFWwindow* cam_win = nullptr;
    std::vector<unsigned char> cam_rgb;
    std::vector<unsigned char> cam_payload;
    std::vector<float>         cam_depth;
    if (cfg.camera_enabled) {
        if (glfwInit() && (glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE), true) &&
            (cam_win = glfwCreateWindow(cfg.camera_width, cfg.camera_height, "g1cam", nullptr,
                                        nullptr)) != nullptr) {
            glfwMakeContextCurrent(cam_win);
            mjv_defaultScene(&cam_scn);
            mjv_makeScene(m, &cam_scn, 2000);
            mjr_defaultContext(&cam_con);
            mjr_makeContext(m, &cam_con, mjFONTSCALE_100);
            mjr_setBuffer(mjFB_OFFSCREEN, &cam_con);
            mjv_defaultOption(&cam_opt);
            mjv_defaultCamera(&cam_cam);
            // FIXED on the d435i camera, not FREE: a free camera ignores cam_xpos/cam_xmat, so
            // the pose written each frame would be discarded.
            cam_cam.type       = mjCAMERA_FIXED;
            cam_cam.fixedcamid = mj_name2id(m, mjOBJ_CAMERA, "d435i");
            cam_rgb.resize(static_cast<std::size_t>(cfg.camera_width) * cfg.camera_height * 3);
            cam_depth.resize(static_cast<std::size_t>(cfg.camera_width) * cfg.camera_height);
            std::fprintf(stderr, "[grove_g1] camera: offscreen %dx%d ready\n",
                         cam_con.offWidth, cam_con.offHeight);
        } else {
            std::fprintf(stderr, "[grove_g1] camera: GL setup FAILED; disabled\n");
            cam_win = nullptr;
        }
    }

    std::vector<float> points(static_cast<std::size_t>(n_rays) * 3);
    std::vector<int>   geomid(static_cast<std::size_t>(n_rays));

    const auto period = std::chrono::duration<double>(1.0 / cfg.rate_hz);
    auto       next   = std::chrono::steady_clock::now();

    bool reload_pending = false;
    while (state().running.load(std::memory_order_relaxed)) {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        // Re-anchor after an overrun, or sleep_until returns at once forever and the loop
        // free-runs on sim_mtx.
        next = std::max(next, std::chrono::steady_clock::now());

        // Snapshot under the lock, raycast outside it: a ~32 ms sweep under the lock would
        // stall physics.
        double     sim_time = 0.0;
        double     torso_pos[3];
        double     torso_mat[9];
        const auto lock_start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
            // Not mj_copyData, which aborts while the render thread holds the stack. A reload frees
            // the old model, so a latched pointer would size these copies from freed memory.
            if (*model != m) {
                reload_pending = true;
            } else {
                const mjData* live = *data;
                std::memcpy(
                    snapshot->geom_xpos, live->geom_xpos, sizeof(mjtNum) * 3 * m->ngeom);
                std::memcpy(
                    snapshot->geom_xmat, live->geom_xmat, sizeof(mjtNum) * 9 * m->ngeom);
                sim_time = live->time;
                std::memcpy(torso_pos, live->xpos + 3 * torso_id, sizeof(torso_pos));
                std::memcpy(torso_mat, live->xmat + 9 * torso_id, sizeof(torso_mat));
                for (std::size_t i = 0; i < object_ids.size(); ++i) {
                    std::memcpy(
                        object_records[i].pos, live->xpos + 3 * object_ids[i],
                        sizeof(object_records[i].pos));
                    std::memcpy(
                        object_records[i].quat, live->xquat + 4 * object_ids[i],
                        sizeof(object_records[i].quat));
                }
            }
        }

        // Rebuilt outside the lock: mj_makeData allocates, and physics must not wait on it.
        if (reload_pending) {
            reload_pending = false;
            mj_deleteData(snapshot);
            m        = *model;
            snapshot = mj_makeData(m);
            torso_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
            // Body ids are indices into the old model's name table; a reload renumbers them.
            resolveObjectBodies(m, cfg.object_bodies, object_ids, object_records);
            std::fprintf(stderr, "[grove_g1] model reloaded; sensor snapshot rebuilt\n");
            // The render state and cam_id belong to the old model. Rebuilding them is not worth
            // it for a debugging-only Reload, so the camera stops and the LiDAR carries on.
            if (cam_win != nullptr) {
                cam_win = nullptr;
                cam_id  = -1;
                std::fprintf(
                    stderr,
                    "[grove_g1] camera DISABLED after model reload; restart the sim for it\n");
            }
            if (torso_id < 0) {
                std::fprintf(
                    stderr, "[grove_g1] reloaded model has no torso_link; sensors DISABLED\n");
                mj_deleteData(snapshot);
                return;
            }
            std::this_thread::sleep_until(next);
            continue;
        }
        const double lock_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lock_start)
                .count();

        // Before kinematics has run, xmat is all zero, and mj_ray aborts the process on a
        // zero-length direction. Skip the cycle rather than hand it one.
        const double row0 = torso_mat[0] * torso_mat[0] + torso_mat[1] * torso_mat[1] +
                            torso_mat[2] * torso_mat[2];
        // NaN fails every comparison, so `row0 < 0.5` alone would let a diverged pose through.
        if (!std::isfinite(row0) || row0 < 0.5 || !std::isfinite(torso_pos[0]) ||
            !std::isfinite(torso_pos[1]) || !std::isfinite(torso_pos[2])) {
            std::this_thread::sleep_until(next);
            continue;
        }

        // Sent before the sweep, so they are ~32 ms fresher. No sensor pose: the records carry
        // world poses directly.
        if (!object_records.empty()) {
            SensorFrameHeader oh{};
            oh.magic         = kSensorFrameMagic;
            oh.version       = kSensorFrameVersion;
            oh.kind          = static_cast<uint32_t>(SensorFrameKind::ObjectPoses);
            oh.payload_bytes =
                static_cast<uint32_t>(object_records.size() * sizeof(ObjectPoseRecord));
            oh.sim_time_s      = sim_time;
            oh.sensor_quat[0]  = 1.0;
            relay.send(oh, object_records.data(), oh.payload_bytes);
        }

        // Sensor pose = live torso pose composed with the fixed mount; torso_link moves with
        // the waist and the stance.
        double origin[3];
        for (int r = 0; r < 3; ++r) {
            origin[r] = torso_pos[r] + torso_mat[3 * r + 0] * kMountXyz[0] +
                        torso_mat[3 * r + 1] * kMountXyz[1] +
                        torso_mat[3 * r + 2] * kMountXyz[2];
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
            const std::size_t o = static_cast<std::size_t>(i) * 3;
            if (dist < cfg.range_min || dist > cfg.range_max) {
                // NaN, not zero: (0,0,0) is a valid point at the sensor, and filters honouring
                // is_dense=false would keep it and mark the robot's own position in a costmap.
                points[o + 0] = points[o + 1] = points[o + 2] =
                    std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            points[o + 0] = static_cast<float>(d[0] * dist);
            points[o + 1] = static_cast<float>(d[1] * dist);
            points[o + 2] = static_cast<float>(d[2] * dist);
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

        // The snapshot is the only work that contends with physics, so its lock time is what
        // gets reported.
        {
            static int    cycles     = 0;
            static double lock_worst = 0.0;
            lock_worst               = std::max(lock_worst, lock_ms);
            if (++cycles % 300 == 0) {
                std::fprintf(
                    stderr, "[grove_g1] snapshot lock: worst %.3f ms over %d cycles\n", lock_worst,
                    cycles);
                lock_worst = 0.0;
            }
        }

        if (cam_win != nullptr && cam_id >= 0 && !reload_pending) {
            // Transform arrays only, never mj_copyData; see the sweep snapshot above.
            const auto cam_t0 = std::chrono::steady_clock::now();
            double     cam_torso_pos[3];
            double     cam_torso_mat[9];
            {
                std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
                // Re-checked: the lock was released since the sweep, and a reload in between
                // would size these copies from the old model.
                if (*model != m) {
                    reload_pending = true;
                }
                const mjData* live = reload_pending ? nullptr : *data;
                if (live != nullptr) {
                std::memcpy(snapshot->xpos, live->xpos, sizeof(mjtNum) * 3 * m->nbody);
                std::memcpy(snapshot->xquat, live->xquat, sizeof(mjtNum) * 4 * m->nbody);
                std::memcpy(snapshot->xmat, live->xmat, sizeof(mjtNum) * 9 * m->nbody);
                if (m->nsite) {
                    std::memcpy(
                        snapshot->site_xpos, live->site_xpos, sizeof(mjtNum) * 3 * m->nsite);
                    std::memcpy(
                        snapshot->site_xmat, live->site_xmat, sizeof(mjtNum) * 9 * m->nsite);
                }
                if (m->nlight) {
                    std::memcpy(
                        snapshot->light_xpos, live->light_xpos, sizeof(mjtNum) * 3 * m->nlight);
                    std::memcpy(
                        snapshot->light_xdir, live->light_xdir, sizeof(mjtNum) * 3 * m->nlight);
                }
                std::memcpy(cam_torso_pos, live->xpos + 3 * torso_id, sizeof(cam_torso_pos));
                std::memcpy(cam_torso_mat, live->xmat + 9 * torso_id, sizeof(cam_torso_mat));
                }
            }
            const auto cam_t1 = std::chrono::steady_clock::now();

            const double cam_row0 = cam_torso_mat[0] * cam_torso_mat[0] +
                                    cam_torso_mat[1] * cam_torso_mat[1] +
                                    cam_torso_mat[2] * cam_torso_mat[2];
            if (std::isfinite(cam_row0) && cam_row0 >= 0.5) {
                // Our own snapshot, so the camera pose can be written straight into it.
                double cam_pos[3];
                for (int r = 0; r < 3; ++r) {
                    cam_pos[r] = cam_torso_pos[r] +
                                 cam_torso_mat[3 * r + 0] * kCamXyz[0] +
                                 cam_torso_mat[3 * r + 1] * kCamXyz[1] +
                                 cam_torso_mat[3 * r + 2] * kCamXyz[2];
                }
                double R_body[9];
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        double acc = 0.0;
                        for (int k = 0; k < 3; ++k) {
                            acc += cam_torso_mat[3 * r + k] * R_cam_mount[3 * k + c];
                        }
                        R_body[3 * r + c] = acc;
                    }
                }
                // MuJoCo cameras look down -z with +y up; the URDF mount frame is x forward,
                // y left, z up. The columns are permuted accordingly.
                double R_cam[9];
                for (int r = 0; r < 3; ++r) {
                    R_cam[3 * r + 0] = -R_body[3 * r + 1];  // cam x  <- -body y
                    R_cam[3 * r + 1] = R_body[3 * r + 2];   // cam y  <-  body z
                    R_cam[3 * r + 2] = -R_body[3 * r + 0];  // cam z  <- -body x
                }
                std::memcpy(snapshot->cam_xpos + 3 * cam_id, cam_pos, sizeof(cam_pos));
                std::memcpy(snapshot->cam_xmat + 9 * cam_id, R_cam, sizeof(R_cam));

                mjv_updateScene(m, snapshot, &cam_opt, nullptr, &cam_cam, mjCAT_ALL, &cam_scn);
                mjrRect vp{0, 0, cfg.camera_width, cfg.camera_height};
                mjr_render(vp, &cam_scn, &cam_con);
                mjr_readPixels(cam_rgb.data(), cam_depth.data(),
                               vp, &cam_con);

                // Linearise the [0,1] OpenGL depth with the frustum the render actually used;
                // vis.map * stat.extent gives a different near plane and wrong depth everywhere.
                const double znear = cam_scn.camera[0].frustum_near;
                const double zfar  = cam_scn.camera[0].frustum_far;
                for (std::size_t i = 0; i < cam_depth.size(); ++i) {
                    const double z = cam_depth[i];
                    cam_depth[i]   = (z >= 1.0)
                                       ? std::numeric_limits<float>::quiet_NaN()
                                       : static_cast<float>(
                                             znear * zfar / (zfar - z * (zfar - znear)));
                }

                // Rows come out of GL bottom-up; ROS images are top-down.
                const int w = cfg.camera_width, h = cfg.camera_height;
                for (int y = 0; y < h / 2; ++y) {
                    std::swap_ranges(cam_depth.begin() + static_cast<std::size_t>(y) * w,
                                     cam_depth.begin() + static_cast<std::size_t>(y + 1) * w,
                                     cam_depth.begin() + static_cast<std::size_t>(h - 1 - y) * w);
                }
                {
                    const std::size_t stride = static_cast<std::size_t>(w) * 3;
                    for (int y = 0; y < h / 2; ++y) {
                        std::swap_ranges(cam_rgb.begin() + static_cast<std::size_t>(y) * stride,
                                         cam_rgb.begin() + static_cast<std::size_t>(y + 1) * stride,
                                         cam_rgb.begin() +
                                             static_cast<std::size_t>(h - 1 - y) * stride);
                    }
                }

                SensorFrameHeader dh{};
                dh.magic         = kSensorFrameMagic;
                dh.version       = kSensorFrameVersion;
                dh.kind          = static_cast<uint32_t>(SensorFrameKind::Depth);
                const std::size_t depth_bytes = cam_depth.size() * sizeof(float);
                const std::size_t rgb_bytes    = cam_rgb.size();
                dh.payload_bytes = static_cast<uint32_t>(depth_bytes + rgb_bytes);
                dh.rgb_bytes     = static_cast<uint32_t>(rgb_bytes);
                dh.sim_time_s    = sim_time;
                std::memcpy(dh.sensor_pos, cam_pos, sizeof(cam_pos));
                matrixToQuat(R_body, dh.sensor_quat);
                dh.width    = static_cast<uint32_t>(w);
                dh.height   = static_cast<uint32_t>(h);
                dh.fovy_deg = static_cast<float>(m->cam_fovy[cam_id]);
                // Colour rides in the same frame: it came from the same render, and pairing it up
                // downstream could only lose that.
                cam_payload.resize(depth_bytes + rgb_bytes);
                std::memcpy(cam_payload.data(), cam_depth.data(), depth_bytes);
                if (rgb_bytes != 0) {
                    std::memcpy(cam_payload.data() + depth_bytes, cam_rgb.data(), rgb_bytes);
                }
                relay.send(dh, cam_payload.data(), cam_payload.size());
            }
            const auto cam_t2 = std::chrono::steady_clock::now();

            static int    cam_n = 0;
            static double cam_lock_ms = 0, cam_total_ms = 0;
            cam_lock_ms += std::chrono::duration<double, std::milli>(cam_t1 - cam_t0).count();
            cam_total_ms += std::chrono::duration<double, std::milli>(cam_t2 - cam_t0).count();
            if (++cam_n % 100 == 0) {
                std::fprintf(
                    stderr, "[grove_g1] camera: %.2f ms/frame, %.3f ms under lock (%d frames)\n",
                    cam_total_ms / cam_n, cam_lock_ms / cam_n, cam_n);
            }
        }

        relay.send(header, points.data(), points.size() * sizeof(float));

        std::this_thread::sleep_until(next);
    }

    mj_deleteData(snapshot);

    // The GL window, scene and context are leaked on purpose: glfwDestroyWindow must run on
    // the main thread, and a stopped sampler never restarts.
}

// The Mid360's own IMU, which FAST-LIO needs beside the laser rather than below the waist joints.
// Its own thread, since a sweep spends ~32 ms raycasting.
void imuLoop(const Config cfg, mjModel** model, mjData** data, std::recursive_mutex* sim_mtx,
             RelaySocket* relay_socket)
{
    // The physics thread loads both after this one starts.
    mjModel* m = nullptr;
    while (state().running.load(std::memory_order_relaxed) &&
           ((m = *model) == nullptr || *data == nullptr)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!state().running.load(std::memory_order_relaxed)) {
        return;
    }

    // By name, never by index: appending sensors to the MJCF shifts every index after them.
    const int quat_id = mj_name2id(m, mjOBJ_SENSOR, "mid360_imu_quat");
    const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "mid360_imu_gyro");
    const int acc_id  = mj_name2id(m, mjOBJ_SENSOR, "mid360_imu_acc");
    const int site_id = mj_name2id(m, mjOBJ_SITE, "mid360_imu");
    if (quat_id < 0 || gyro_id < 0 || acc_id < 0 || site_id < 0) {
        std::fprintf(stderr,
                     "[grove_g1] no mid360_imu sensors in this model; FAST-LIO will get no IMU. "
                     "Is the MJCF patch applied?\n");
        return;
    }
    const int quat_adr = m->sensor_adr[quat_id];
    const int gyro_adr = m->sensor_adr[gyro_id];
    const int acc_adr  = m->sensor_adr[acc_id];

    std::fprintf(stderr, "[grove_g1] mid360 IMU up at %.1f Hz\n", cfg.imu_rate_hz);

    const auto period = std::chrono::duration<double>(1.0 / cfg.imu_rate_hz);
    auto       next   = std::chrono::steady_clock::now();
    while (state().running.load(std::memory_order_relaxed)) {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        next = std::max(next, std::chrono::steady_clock::now());

        SensorFrameHeader header{};
        ImuSampleRecord   sample{};
        bool              have = false;
        {
            std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
            const mjData*                         d = *data;
            // A viewer reload swaps both pointers, and the addresses above were resolved
            // against the old model. Same check, and the same reason, as the sweep loop's.
            if (*model != m || d == nullptr) {
                break;
            }
            header.sim_time_s = d->time;
            for (int i = 0; i < 3; ++i) {
                header.sensor_pos[i] = d->site_xpos[site_id * 3 + i];
                sample.gyro[i]       = d->sensordata[gyro_adr + i];
                sample.acc[i]        = d->sensordata[acc_adr + i];
            }
            for (int i = 0; i < 4; ++i) {
                header.sensor_quat[i] = d->sensordata[quat_adr + i];
            }
            have = true;
        }

        if (have) {
            header.magic         = kSensorFrameMagic;
            header.version       = kSensorFrameVersion;
            header.kind          = static_cast<uint32_t>(SensorFrameKind::Imu);
            header.payload_bytes = static_cast<uint32_t>(sizeof(sample));
            // trySend: waiting out another thread's 2.9 MB frame would leave a far bigger gap
            // in this stream than one dropped sample.
            relay_socket->trySend(header, &sample, sizeof(sample));
        }

        std::this_thread::sleep_until(next);
    }
}

// Fast enough that the 50 Hz odometry publisher always has a fresh sample, and the frame is
// 48 bytes, so the cost is noise next to one LiDAR sweep.
constexpr double kBaseStateRateHz = 200.0;

// Ground truth for the sim-only odometry source: stock MJCF sensors on the pelvis IMU site, so
// this is exact MuJoCo state rather than anything modelled.
void baseStateLoop(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx,
                   RelaySocket* relay_socket)
{
    mjModel* m = nullptr;
    while (state().running.load(std::memory_order_relaxed) &&
           ((m = *model) == nullptr || *data == nullptr)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!state().running.load(std::memory_order_relaxed)) {
        return;
    }

    const int quat_id = mj_name2id(m, mjOBJ_SENSOR, "imu_quat");
    const int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
    const int pos_id  = mj_name2id(m, mjOBJ_SENSOR, "frame_pos");
    const int vel_id  = mj_name2id(m, mjOBJ_SENSOR, "frame_vel");
    const int site_id = mj_name2id(m, mjOBJ_SITE, "imu");
    if (quat_id < 0 || gyro_id < 0 || pos_id < 0 || vel_id < 0 || site_id < 0) {
        std::fprintf(stderr,
                     "[grove_g1] no pelvis imu sensors in this model; nothing will publish "
                     "ground-truth odometry\n");
        return;
    }
    const int quat_adr = m->sensor_adr[quat_id];
    const int gyro_adr = m->sensor_adr[gyro_id];
    const int pos_adr  = m->sensor_adr[pos_id];
    const int vel_adr  = m->sensor_adr[vel_id];

    std::fprintf(stderr, "[grove_g1] ground-truth base state up at %.1f Hz\n", kBaseStateRateHz);

    const auto period = std::chrono::duration<double>(1.0 / kBaseStateRateHz);
    auto       next   = std::chrono::steady_clock::now();
    while (state().running.load(std::memory_order_relaxed)) {
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        next = std::max(next, std::chrono::steady_clock::now());

        SensorFrameHeader header{};
        BaseStateRecord   sample{};
        bool              have = false;
        {
            std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
            const mjData*                         d = *data;
            // A viewer reload swaps both pointers and the addresses above belong to the old
            // model. Same check, same reason, as the sweep and IMU loops'.
            if (*model != m || d == nullptr) {
                break;
            }
            header.sim_time_s = d->time;
            for (int i = 0; i < 3; ++i) {
                header.sensor_pos[i] = d->sensordata[pos_adr + i];
                sample.ang_vel[i]    = d->sensordata[gyro_adr + i];
            }
            for (int i = 0; i < 4; ++i) {
                header.sensor_quat[i] = d->sensordata[quat_adr + i];
            }
            // framelinvel reports in the world frame; the record carries body-frame rates, so
            // rotate here and the relay can publish a conventional Odometry twist untouched.
            const double* site_mat = d->site_xmat + (site_id * 9);
            for (int r = 0; r < 3; ++r) {
                double acc = 0.0;
                for (int k = 0; k < 3; ++k) {
                    acc += site_mat[(3 * k) + r] * d->sensordata[vel_adr + k];
                }
                sample.lin_vel[r] = acc;
            }
            have = true;
        }

        if (have) {
            header.magic         = kSensorFrameMagic;
            header.version       = kSensorFrameVersion;
            header.kind          = static_cast<uint32_t>(SensorFrameKind::BaseState);
            header.payload_bytes = static_cast<uint32_t>(sizeof(sample));
            // trySend for the same reason the IMU uses it: this socket is shared, and waiting
            // out someone else's multi-megabyte frame costs more than one dropped sample.
            relay_socket->trySend(header, &sample, sizeof(sample));
        }

        std::this_thread::sleep_until(next);
    }
}

}  // namespace

void StartSensorPublisher(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    const Config cfg = loadConfig();
    if (!cfg.enabled || state().running.load()) {
        return;
    }
    state().running.store(true);
    state().relay      = std::make_unique<RelaySocket>(cfg.socket_path);
    state().thread     = std::thread(sensorLoop, cfg, model, data, sim_mtx, state().relay.get());
    state().imu_thread = std::thread(imuLoop, cfg, model, data, sim_mtx, state().relay.get());
    state().base_thread =
        std::thread(baseStateLoop, model, data, sim_mtx, state().relay.get());
}

void StopSensorPublisher()
{
    State& s = state();
    if (!s.running.exchange(false)) {
        return;
    }
    // Join, not just signal: the render and raycast read the model outside the sim lock, so it
    // may be freed only once the threads have stopped. Costs up to a period plus one sweep.
    if (s.thread.joinable()) {
        s.thread.join();
    }
    if (s.imu_thread.joinable()) {
        s.imu_thread.join();
    }
    if (s.base_thread.joinable()) {
        s.base_thread.join();
    }
    // Closed once every thread has stopped, so the relay sees EOF and logs a disconnect.
    s.relay.reset();
    std::fprintf(stderr,
                 "[grove_g1] SENSORS DISABLED: the model was replaced and the sampler was "
                 "stopped to avoid reading a freed model. Relaunch the sim to restore "
                 "them; nothing will publish on /livox/lidar or /camera/* until then.\n");
}


}  // namespace grove_g1
