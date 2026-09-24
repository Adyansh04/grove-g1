#include "grasp_weld.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace grove_g1
{
namespace
{

// Scene equalities with this prefix are managed; any other equality is left alone.
constexpr const char* kWeldPrefix = "grasp_";

constexpr int kTickRateHz = 100;

struct Config
{
    bool enabled = false;

    // Palm-to-object origin distance within which the hand can take the object. A released
    // object must get this far away before it can be taken again.
    double capture_radius_m = 0.12;

    // How long a held object may go without thumb-and-finger contact before release: long
    // enough to ride out contact flicker during a carry.
    double release_after_s = 0.3;
};

// One managed weld: a palm, an object, and the constraint that can join them.
struct ManagedWeld
{
    int         eq_id     = -1;
    int         palm_id   = -1;
    int         object_id = -1;
    std::string name;
    // Cleared on release and set again once the palm is outside the capture radius, so a hand
    // opening over the object it just put down does not take it straight back.
    bool armed = true;
};

// A hand, and whichever weld it currently holds.
struct Hand
{
    int palm_id     = -1;
    int thumb       = -1;  ///< the palm child the thumb hangs from
    int holding_eq  = -1;
    int apart_ticks = 0;
};

struct State
{
    std::thread       thread;
    std::atomic<bool> running{ false };
};

State& state()
{
    // Leaked on purpose: the physics thread ends in exit(0), and a static destructor
    // destroying this still-joinable std::thread would call std::terminate.
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
        if (node["release_after_s"])
        {
            cfg.release_after_s = node["release_after_s"].as<double>();
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
    if (cfg.capture_radius_m <= 0.0 || cfg.release_after_s < 0.0)
    {
        std::fprintf(
            stderr,
            "[grove_g1] grasp_weld needs capture_radius_m > 0 and release_after_s >= 0; "
            "DISABLED\n");
        cfg.enabled = false;
    }
    return cfg;
}

// The digit `body` belongs to, as the palm's direct child it hangs from; -1 for the palm itself
// or anything outside this hand.
int digitOf(const mjModel* model, int body, int palm_id)
{
    while (body > 0)
    {
        const int parent = model->body_parentid[body];
        if (parent == palm_id)
        {
            return body;
        }
        body = parent;
    }
    return -1;
}

// Whether the thumb and at least one other digit of `hand` touch `object_id` this step. The
// thumb is required so side-by-side fingers with nothing opposite cannot take the object.
bool opposedOn(const mjModel* model, const mjData* data, const Hand& hand, int object_id)
{
    bool thumb  = false;
    bool finger = false;
    for (int i = 0; i < data->ncon; ++i)
    {
        const mjContact& contact = data->contact[i];
        if (contact.exclude != 0)
        {
            continue;
        }
        for (int side = 0; side < 2; ++side)
        {
            const int mine  = contact.geom[side];
            const int other = contact.geom[1 - side];
            if (mine < 0 || other < 0 || model->geom_bodyid[other] != object_id)
            {
                continue;
            }
            const int digit = digitOf(model, model->geom_bodyid[mine], hand.palm_id);
            thumb           = thumb || (digit >= 0 && digit == hand.thumb);
            finger          = finger || (digit >= 0 && digit != hand.thumb);
        }
    }
    return thumb && finger;
}

double distanceBetween(const mjData* data, int body_a, int body_b)
{
    double d[3];
    mju_sub3(d, data->xpos + 3 * body_a, data->xpos + 3 * body_b);
    return mju_norm3(d);
}

// Freezes the object where it sits relative to the palm. relpose must be written, or the weld snaps
// the object to its spawn offset; eq_data (MuJoCo 3.3.6) is anchor, pos, quat, torquescale.
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
// this model declares none, as most scenes do.
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

        // A weld that starts engaged holds an object the hand is nowhere near; run() clears it.
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
        // The palm's direct children are its digits. Which one is the thumb is only in its
        // name: nothing in the tree's shape tells it from the fingers.
        int digits = 0;
        for (int body = 1; body < model->nbody; ++body)
        {
            if (model->body_parentid[body] != hand.palm_id)
            {
                continue;
            }
            ++digits;
            const char* name = mj_id2name(model, mjOBJ_BODY, body);
            if (name != nullptr && std::strstr(name, "thumb") != nullptr)
            {
                hand.thumb = body;
            }
        }
        if (digits < 2 || hand.thumb < 0)
        {
            std::fprintf(
                stderr,
                "[grove_g1] weld body1 '%s' has no thumb and finger under it; grasp DISABLED\n",
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

    {
        std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
        if (*model == m && *data != nullptr)
        {
            for (const ManagedWeld& weld : welds)
            {
                (*data)->eq_active[weld.eq_id] = 0;
            }
        }
    }

    const auto period = std::chrono::nanoseconds(std::chrono::seconds(1)) / kTickRateHz;
    const int  release_ticks = static_cast<int>(std::ceil(cfg.release_after_s * kTickRateHz));
    auto       next   = std::chrono::steady_clock::now();

    while (s.running.load(std::memory_order_relaxed))
    {
        next += period;
        std::this_thread::sleep_until(next);

        std::lock_guard<std::recursive_mutex> lock(*sim_mtx);
        // A reload frees the model these ids index. One-way, like the sensor sampler.
        if (*model != m || *data == nullptr)
        {
            std::fprintf(stderr, "[grove_g1] model replaced; grasp weld is OFF\n");
            return;
        }
        mjData* d = *data;

        for (ManagedWeld& weld : welds)
        {
            if (!weld.armed &&
                distanceBetween(d, weld.palm_id, weld.object_id) > cfg.capture_radius_m)
            {
                weld.armed = true;
            }
        }

        for (Hand& hand : hands)
        {
            if (hand.holding_eq >= 0)
            {
                ManagedWeld& held = welds[hand.holding_eq];
                hand.apart_ticks =
                    opposedOn(m, d, hand, held.object_id) ? 0 : hand.apart_ticks + 1;
                if (hand.apart_ticks > release_ticks)
                {
                    release(d, held);
                    held.armed       = false;
                    hand.holding_eq  = -1;
                    hand.apart_ticks = 0;
                }
                continue;
            }

            // Nearest opposed candidate wins, so a hand closing between two objects takes one
            // rather than whichever the scene lists first.
            int    best     = -1;
            double best_gap = cfg.capture_radius_m;
            for (std::size_t i = 0; i < welds.size(); ++i)
            {
                const ManagedWeld& weld = welds[i];
                if (weld.palm_id != hand.palm_id || !weld.armed || d->eq_active[weld.eq_id] != 0)
                {
                    continue;
                }
                const double gap = distanceBetween(d, weld.palm_id, weld.object_id);
                if (gap < best_gap && opposedOn(m, d, hand, weld.object_id))
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

    // eq_active persists, so a stopped manager must not leave an object glued to a hand.
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
