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

    // Mid360 envelope. Resolution is a real budget, not a formality: 360x32 costs ~32 ms per
    // sweep against the G1 scene, so it is configurable and the timing gate decides what ships.
    // Which MuJoCo geom group the sweep sees. Scene geometry is group 2; the robot uses
    // groups 0 (collision) and 1 (visual). Without the mask every ray returns the torso
    // shell ~6 cm from the mount and the world is never reached.
    //
    // Group 2 and not 3, because MuJoCo's viewer renders only groups 0-2: scene geometry in
    // group 3 is physically present, hit by rays, and completely invisible on screen.
    int    scene_geom_group = 2;

    bool camera_enabled = false;
    bool camera_color   = true;
    int  camera_width   = 848;
    int  camera_height  = 480;
    // d435_joint in the vendored URDF, relative to torso_link. Same provenance as the
    // LiDAR mount, and equally not to be confused with g1_sim's torso-folded values.
    double cam_xyz[3] = {0.0576235, 0.01753, 0.42987};
    double cam_rpy[3] = {0.0, 0.8307767239493009, 0.0};
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
        if (root["camera_color"]) {
            cfg.camera_color = root["camera_color"].as<bool>();
        }
        if (root["camera_enabled"]) {
            cfg.camera_enabled = root["camera_enabled"].as<bool>();
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
        if (!sendAll(&header, sizeof(header), /*frame_started=*/false)) {
            return;
        }
        // Mid-frame now: dropping the body would leave a header with no payload, and the
        // relay would read the next frame's bytes as this one's points, validate them, and
        // publish a self-consistent cloud of garbage.
        sendAll(payload, payload_bytes, /*frame_started=*/true);
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
        // Depth frames are ~1.6 MB; the default buffer would force a retry on every one.
        const int snd = 4 * 1024 * 1024;
        ::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &snd, sizeof(snd));

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
        // 20 ms covered a 1.6 MB depth frame but not the 2.9 MB depth+colour one: the
        // receiver drains roughly a socket buffer per wakeup, so the budget has to cover
        // enough of its wakeups to move the whole payload. Still well inside the 100 ms
        // frame period, and this waits off the sim lock, so overrunning it costs nothing
        // on the physics side.
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
                // A depth frame is ~1.6 MB and will not fit the socket buffer in one go, so
                // EAGAIN mid-frame is normal rather than a fault. Wait briefly for the relay
                // to drain. This is off the sim lock, so it delays only this thread, never
                // physics; past the deadline the frame is abandoned and the connection reset
                // rather than left desynchronised.
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

    mjModel* m        = *model;
    int      torso_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
    if (torso_id < 0) {
        std::fprintf(stderr, "[grove_g1] no torso_link in the model; sensors DISABLED\n");
        return;
    }

    double R_mount[9];
    rpyToMatrix(cfg.mount_rpy, R_mount);
    double R_cam_mount[9];
    rpyToMatrix(cfg.cam_rpy, R_cam_mount);
    const int cam_id = mj_name2id(m, mjOBJ_CAMERA, "d435i");

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

    // S3 spike scaffolding. Its own GL context on this thread: measured safe alongside the
    // viewer's, and glfwCreateWindow off the main thread works here despite the docs.
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
            // FIXED and bound to our camera element, not FREE. A free camera ignores
            // cam_xpos/cam_xmat entirely and orbits its own default pose, so every pose
            // written below would be silently discarded and the depth would be identical
            // whatever the robot did.
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
        // If a sweep overruns its period, sleep_until returns immediately forever and the
        // loop free-runs, taking sim_mtx as fast as it can. Resolution is configurable, so
        // that is reachable rather than theoretical.
        next = std::max(next, std::chrono::steady_clock::now());

        // Snapshot under the lock, raycast outside it. Holding the lock across a ~32 ms
        // sweep would stall physics exactly as badly as running inline.
        double     sim_time = 0.0;
        double     torso_pos[3];
        double     torso_mat[9];
        const auto lock_start = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
            // NOT mj_copyData. That copies the arena as well, and MuJoCo rejects it
            // outright while another thread has the stack in use ("attempting to copy
            // mjData while stack is in use") -- which killed the simulator, because
            // sim.mtx does not cover the render thread's use of mjData.
            //
            // mj_ray only reads geom poses, and everything this sweep can hit is a
            // primitive (the scene's boxes and plane), so no mesh BVH is involved. Two
            // arrays is both correct and far cheaper than a full mjData.
            // The viewer's Reload button and drag-and-drop both replace the model and
            // free the old one (main.cc reassigns m/d). A latched pointer would dangle,
            // and the memcpy below would size itself from a freed model's ngeom.
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
            }
        }

        // Rebuilt outside the lock: mj_makeData allocates, and physics must not wait on it.
        if (reload_pending) {
            reload_pending = false;
            mj_deleteData(snapshot);
            m        = *model;
            snapshot = mj_makeData(m);
            torso_id = mj_name2id(m, mjOBJ_BODY, "torso_link");
            std::fprintf(stderr, "[grove_g1] model reloaded; sensor snapshot rebuilt\n");
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

        // A snapshot taken before MuJoCo has run kinematics has an all-zero xmat, and
        // mj_ray aborts the whole process on a zero-length direction ("vector length is too
        // small"). Skip the cycle rather than hand it one.
        const double row0 = torso_mat[0] * torso_mat[0] + torso_mat[1] * torso_mat[1] +
                            torso_mat[2] * torso_mat[2];
        // !isfinite first: NaN fails every comparison, so `row0 < 0.5` alone lets a diverged
        // pose through and mj_ray answers a zero-length direction with mju_error, which
        // aborts the simulator rather than returning.
        if (!std::isfinite(row0) || row0 < 0.5 || !std::isfinite(torso_pos[0]) ||
            !std::isfinite(torso_pos[1]) || !std::isfinite(torso_pos[2])) {
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
            const std::size_t o = static_cast<std::size_t>(i) * 3;
            if (dist < cfg.range_min || dist > cfg.range_max) {
                // NaN, not (0,0,0). Zero is a valid point AT the sensor, and every filter
                // that honours is_dense=false keeps it: thousands of phantom returns
                // stacked on the robot, straight into a costmap.
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

        // The snapshot is the only thing that contends with physics, so its cost is the
        // number that matters; the ~32 ms sweep below it runs off-lock. Reported rarely
        // rather than never: if the copy ever grows, this is where it shows up.
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

        if (cam_win != nullptr && cam_id >= 0) {
            // Transform arrays only, never mj_copyData: the copy is what MuJoCo refuses
            // when the live stack is in use, and refusing aborts the process (S3, test A).
            const auto cam_t0 = std::chrono::steady_clock::now();
            double     cam_torso_pos[3];
            double     cam_torso_mat[9];
            {
                std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
                const mjData* live = *data;
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
            const auto cam_t1 = std::chrono::steady_clock::now();

            const double cam_row0 = cam_torso_mat[0] * cam_torso_mat[0] +
                                    cam_torso_mat[1] * cam_torso_mat[1] +
                                    cam_torso_mat[2] * cam_torso_mat[2];
            if (std::isfinite(cam_row0) && cam_row0 >= 0.5) {
                // Our own snapshot, so writing the camera pose straight into it is safe and
                // avoids needing a mocap body or a camera on the vendored robot.
                double cam_pos[3];
                for (int r = 0; r < 3; ++r) {
                    cam_pos[r] = cam_torso_pos[r] +
                                 cam_torso_mat[3 * r + 0] * cfg.cam_xyz[0] +
                                 cam_torso_mat[3 * r + 1] * cfg.cam_xyz[1] +
                                 cam_torso_mat[3 * r + 2] * cfg.cam_xyz[2];
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
                // MuJoCo cameras look down their own -z with +y up; the URDF mount frame is
                // x forward, y left, z up. Columns are permuted rather than multiplying by a
                // constant rotation, which is the same mapping written more directly.
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
                mjr_readPixels(cfg.camera_color ? cam_rgb.data() : nullptr, cam_depth.data(),
                               vp, &cam_con);

                // mjr_readPixels hands back the raw OpenGL depth buffer: non-linear, in
                // [0,1]. Publishing it as metres would look plausible at every distance and
                // be wrong at all of them, so it is linearised here against the model's own
                // near/far planes.
                // The frustum the render actually used, not vis.map. mjv_updateScene
                // derives frustum_near/far per camera and mjr_render projects with those;
                // reconstructing them from vis.map * stat.extent gives a different, wrong
                // near plane and therefore a depth that is plausible at every range and
                // correct at none.
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
                if (cfg.camera_color) {
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
                const std::size_t rgb_bytes    = cfg.camera_color ? cam_rgb.size() : 0;
                dh.payload_bytes = static_cast<uint32_t>(depth_bytes + rgb_bytes);
                dh.rgb_bytes     = static_cast<uint32_t>(rgb_bytes);
                dh.sim_time_s    = sim_time;
                std::memcpy(dh.sensor_pos, cam_pos, sizeof(cam_pos));
                matrixToQuat(R_body, dh.sensor_quat);
                dh.width    = static_cast<uint32_t>(w);
                dh.height   = static_cast<uint32_t>(h);
                dh.fovy_deg = static_cast<float>(m->cam_fovy[cam_id]);
                // Colour rides in the same frame rather than a second one: it came from the
                // same render, so pairing it up downstream could only lose that guarantee.
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


}  // namespace grove_g1
