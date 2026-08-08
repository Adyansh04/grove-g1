#include "grasp_weld.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace grove_g1
{
namespace
{

// Welds this manager owns are named with this prefix in the scene. Anything else in
// <equality> is left alone.
constexpr const char* kWeldPrefix = "grasp_";

constexpr int kTickRateHz = 100;

struct Config
{
    bool enabled = false;

    // How close the object's origin must be to the palm's for a closing hand to take it.
    // Sized for the palm-to-grasp-point offset plus half a small object, not measured against
    // anything real -- see the header on why none of these transfer to hardware.
    double capture_radius_m = 0.12;

    // Fraction of available travel, averaged over the hand's seven joints, that counts as
    // closed. The SRDF's `closed` posture measures 0.70 by this metric and `open` measures 0,
    // so 0.5 sits between them with room either side.
    double close_fraction = 0.5;

    // Released below this, not at close_fraction: without the gap, a hand holding at exactly
    // the threshold would drop and re-grab the object every few ticks.
    double release_fraction = 0.3;
};

// One managed weld: a palm, an object, and the constraint that can join them.
struct ManagedWeld
{
    int         eq_id     = -1;
    int         palm_id   = -1;
    int         object_id = -1;
    std::string name;
};

// A hand, and whichever weld it currently holds.
struct Hand
{
    int              palm_id = -1;
    std::vector<int> finger_qpos_adr;
    std::vector<int> finger_jnt_id;
    int              holding_eq = -1;
};

struct State
{
    std::thread       thread;
    std::atomic<bool> running{ false };
};

State& state()
{
    // Deliberately leaked, for the same reason sensor_publisher and dex3_handler leak theirs:
    // the physics thread ends in exit(0), which runs static destructors while this thread may
    // still be alive, and destroying a joinable std::thread calls std::terminate.
    static State* s = new State();
    return *s;
}

Config loadConfig()
{
    Config      cfg;
    const char* path = std::getenv("GROVE_G1_SENSOR_CONFIG");
    if (path == nullptr || *path == '\0')
    {
        return cfg;
    }
    try
    {
        const YAML::Node root = YAML::LoadFile(path);
        const YAML::Node node = root["grasp_weld"];
        if (!node)
        {
            return cfg;
        }
        cfg.enabled = node["enabled"] ? node["enabled"].as<bool>() : true;
        if (node["capture_radius_m"])
        {
            cfg.capture_radius_m = node["capture_radius_m"].as<double>();
        }
        if (node["close_fraction"])
        {
            cfg.close_fraction = node["close_fraction"].as<double>();
        }
        if (node["release_fraction"])
        {
            cfg.release_fraction = node["release_fraction"].as<double>();
        }
    }
    catch (const std::exception& e)
    {
        // Loud, and still off: a malformed config must not look like a working grasp.
        std::fprintf(
            stderr, "[grove_g1] grasp_weld config '%s' failed to load (%s); DISABLED\n", path,
            e.what());
        cfg.enabled = false;
    }
    if (cfg.capture_radius_m <= 0.0 || cfg.release_fraction > cfg.close_fraction)
    {
        std::fprintf(
            stderr,
            "[grove_g1] grasp_weld needs capture_radius_m > 0 and release_fraction <= "
            "close_fraction; DISABLED\n");
        cfg.enabled = false;
    }
    return cfg;
}

// True if `ancestor` is `body` or one of its parents. Used instead of matching joint names, so
// the fingers are found by where they are in the model rather than by what they are called.
bool isInSubtree(const mjModel* model, int body, int ancestor)
{
    while (body > 0)
    {
        if (body == ancestor)
        {
            return true;
        }
        body = model->body_parentid[body];
    }
    return body == ancestor;
}

// Mean fraction of available travel across the hand's joints. Zero at the URDF zero (which is
// what `open` is) and about 0.70 at the SRDF's `closed` posture.
//
// Signed against whichever range bound the joint is moving toward, so it reads the same for
// both hands: the Dex3's two sides mirror, and the left closes negative where the right
// closes positive.
double closureOf(const mjModel* model, const mjData* data, const Hand& hand)
{
    if (hand.finger_qpos_adr.empty())
    {
        return 0.0;
    }
    double total = 0.0;
    for (std::size_t i = 0; i < hand.finger_qpos_adr.size(); ++i)
    {
        const double q     = data->qpos[hand.finger_qpos_adr[i]];
        const int    jnt   = hand.finger_jnt_id[i];
        const double bound = q < 0.0 ? model->jnt_range[2 * jnt] : model->jnt_range[2 * jnt + 1];
        if (std::abs(bound) > 1e-9)
        {
            total += std::abs(q / bound);
        }
    }
    return total / static_cast<double>(hand.finger_qpos_adr.size());
}

double distanceBetween(const mjData* data, int body_a, int body_b)
{
    double d[3];
    mju_sub3(d, data->xpos + 3 * body_a, data->xpos + 3 * body_b);
    return mju_norm3(d);
}

// Freezes the object where it currently sits relative to the palm.
//
// The relative pose MUST be written: MuJoCo's compiler pre-fills eq_data's relpose from the
// model's initial configuration, so an untouched weld would snap the object to wherever it
// happened to start relative to the hand. Layout verified against MuJoCo 3.3.6 rather than
// assumed -- anchor(3), relpose position(3), relpose quaternion(4), torquescale(1).
void engage(mjModel* model, mjData* data, const ManagedWeld& weld)
{
    double palm_quat_inv[4];
    double delta[3];
    double rel_pos[3];
    double rel_quat[4];
    mju_negQuat(palm_quat_inv, data->xquat + 4 * weld.palm_id);
    mju_sub3(delta, data->xpos + 3 * weld.object_id, data->xpos + 3 * weld.palm_id);
    mju_rotVecQuat(rel_pos, delta, palm_quat_inv);
    mju_mulQuat(rel_quat, palm_quat_inv, data->xquat + 4 * weld.object_id);

    mjtNum* eq_data = model->eq_data + weld.eq_id * mjNEQDATA;
    mju_zero3(eq_data);
    mju_copy3(eq_data + 3, rel_pos);
    mju_copy4(eq_data + 6, rel_quat);
    eq_data[10] = 1.0;

    data->eq_active[weld.eq_id] = 1;
    std::fprintf(stderr, "[grove_g1] grasp: %s engaged\n", weld.name.c_str());
}

void release(mjData* data, const ManagedWeld& weld)
{
    data->eq_active[weld.eq_id] = 0;
    std::fprintf(stderr, "[grove_g1] grasp: %s released\n", weld.name.c_str());
}

// Reads the managed welds out of the model, and the hands out of the welds. Returns false if
// this model declares none, which is the normal case for the flat and perception worlds.
bool resolve(const mjModel* model, std::vector<ManagedWeld>& welds, std::vector<Hand>& hands)
{
    welds.clear();
    hands.clear();
    const std::size_t prefix_len = std::char_traits<char>::length(kWeldPrefix);

    for (int eq = 0; eq < model->neq; ++eq)
    {
        const char* name = mj_id2name(model, mjOBJ_EQUALITY, eq);
        if (name == nullptr || std::string(name).compare(0, prefix_len, kWeldPrefix) != 0)
        {
            continue;
        }
        if (model->eq_type[eq] != mjEQ_WELD || model->eq_objtype[eq] != mjOBJ_BODY)
        {
            std::fprintf(
                stderr, "[grove_g1] equality '%s' is not a body weld; not managed\n", name);
            continue;
        }
        ManagedWeld weld;
        weld.eq_id     = eq;
        weld.palm_id   = model->eq_obj1id[eq];
        weld.object_id = model->eq_obj2id[eq];
        weld.name      = name;
        welds.push_back(weld);

        // Inactive at rest, whatever the scene said: a weld that starts engaged would hold an
        // object the hand is nowhere near.
        if (model->eq_active0[eq] != 0)
        {
            std::fprintf(
                stderr, "[grove_g1] weld '%s' declares active=true; forcing it off\n", name);
        }
    }
    if (welds.empty())
    {
        return false;
    }

    for (const ManagedWeld& weld : welds)
    {
        const bool known = std::any_of(
            hands.begin(), hands.end(),
            [&weld](const Hand& hand) { return hand.palm_id == weld.palm_id; });
        if (known)
        {
            continue;
        }
        Hand hand;
        hand.palm_id = weld.palm_id;
        for (int jnt = 0; jnt < model->njnt; ++jnt)
        {
            // Hinges only, and only inside the palm's own subtree: that is exactly the seven
            // finger joints, found without depending on what they are named.
            if (model->jnt_type[jnt] != mjJNT_HINGE ||
                !isInSubtree(model, model->jnt_bodyid[jnt], hand.palm_id))
            {
                continue;
            }
            hand.finger_qpos_adr.push_back(model->jnt_qposadr[jnt]);
            hand.finger_jnt_id.push_back(jnt);
        }
        if (hand.finger_qpos_adr.empty())
        {
            std::fprintf(
                stderr,
                "[grove_g1] weld body1 '%s' has no finger joints under it; grasp DISABLED\n",
                mj_id2name(model, mjOBJ_BODY, hand.palm_id));
            return false;
        }
        hands.push_back(hand);
    }

    std::fprintf(
        stderr, "[grove_g1] grasp weld: %zu welds across %zu hands\n", welds.size(), hands.size());
    return true;
}

void run(const Config cfg, mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    auto& s = state();
    while (s.running.load(std::memory_order_relaxed) && (*model == nullptr || *data == nullptr))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!s.running.load(std::memory_order_relaxed))
    {
        return;
    }

    mjModel*                 m = *model;
    std::vector<ManagedWeld> welds;
    std::vector<Hand>        hands;
    if (!resolve(m, welds, hands))
    {
        std::fprintf(stderr, "[grove_g1] no grasp welds in this scene; grasp weld is OFF\n");
        return;
    }

    const auto period = std::chrono::nanoseconds(std::chrono::seconds(1)) / kTickRateHz;
    auto       next   = std::chrono::steady_clock::now();

    while (s.running.load(std::memory_order_relaxed))
    {
        next += period;
        std::this_thread::sleep_until(next);

        std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
        // A reload frees the model these ids index. One-way, like the sensor sampler: the
        // grasp stops rather than resolving against a model it never measured.
        if (*model != m || *data == nullptr)
        {
            std::fprintf(stderr, "[grove_g1] model replaced; grasp weld is OFF\n");
            return;
        }
        mjData* d = *data;

        for (Hand& hand : hands)
        {
            const double closure = closureOf(m, d, hand);

            if (hand.holding_eq >= 0)
            {
                if (closure <= cfg.release_fraction)
                {
                    release(d, welds[hand.holding_eq]);
                    hand.holding_eq = -1;
                }
                continue;
            }
            if (closure < cfg.close_fraction)
            {
                continue;
            }

            // Nearest candidate wins, so a hand closing between two objects takes one rather
            // than whichever the scene happens to list first.
            int    best     = -1;
            double best_gap = cfg.capture_radius_m;
            for (std::size_t i = 0; i < welds.size(); ++i)
            {
                if (welds[i].palm_id != hand.palm_id || d->eq_active[welds[i].eq_id] != 0)
                {
                    continue;
                }
                const double gap = distanceBetween(d, welds[i].palm_id, welds[i].object_id);
                if (gap < best_gap)
                {
                    best     = static_cast<int>(i);
                    best_gap = gap;
                }
            }
            if (best >= 0)
            {
                engage(m, d, welds[best]);
                hand.holding_eq = best;
            }
        }
    }

    // A stopped manager must not leave an object glued to a hand: eq_active persists, and the
    // next thing to read this model would see a grasp nobody is maintaining.
    std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
    if (*model == m && *data != nullptr)
    {
        for (const ManagedWeld& weld : welds)
        {
            (*data)->eq_active[weld.eq_id] = 0;
        }
    }
}

}  // namespace

void StartGraspWeld(mjModel** model, mjData** data, std::recursive_mutex* sim_mtx)
{
    auto& s = state();
    if (s.running.load(std::memory_order_relaxed))
    {
        return;
    }
    const Config cfg = loadConfig();
    if (!cfg.enabled)
    {
        return;
    }
    s.running.store(true, std::memory_order_relaxed);
    s.thread = std::thread(run, cfg, model, data, sim_mtx);
}

void StopGraspWeld()
{
    auto& s = state();
    if (!s.thread.joinable())
    {
        s.running.store(false, std::memory_order_relaxed);
        return;
    }
    s.running.store(false, std::memory_order_relaxed);
    s.thread.join();
}

}  // namespace grove_g1
