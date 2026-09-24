#include "dex3_handler.h"

#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/idl/hg/HandState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

namespace grove_g1
{
namespace
{

constexpr std::size_t kNumJoints = 7;

// Dex3 wire order, identical for both hands. Resolved by name, never by index: a reordered MJCF
// would otherwise close the wrong fingers.
constexpr std::array<const char*, kNumJoints> kJointSuffixes = {
    "thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1",
};

// Peak torque per joint, N m, from the URDF's effort limits, so a simulated finger is no stronger
// than the real one.
constexpr std::array<double, kNumJoints> kEffortLimit = {
    2.45, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4,
};

constexpr int    kControlRateHz  = 1000;  // matches the SDK bridge's own tick
constexpr int    kStatePublishHz = 100;   // real rate is undocumented; matches the cmd rate
constexpr double kCommandTimeoutS = 1.0;  // Unitree's own timeout bit means ~1 s

// Lock means "motor held", not "motor off": undriven fingers hold station as on hardware, which is
// also what MoveIt reads as the start state. The hand's own nominal gains.
constexpr double kHoldKp = 1.5;
constexpr double kHoldKd = 0.2;

// status lives in bits 4-6 of the packed mode byte; 1 = FOC (driven), 0 = Lock.
constexpr std::uint8_t kStatusLock = 0x00;
constexpr std::uint8_t kStatusFoc  = 0x01;

std::uint8_t statusOf(std::uint8_t mode) { return (mode >> 4) & 0x07; }

std::uint8_t packMode(std::size_t index, std::uint8_t status)
{
    return static_cast<std::uint8_t>((index & 0x0F) | ((status & 0x07) << 4));
}

using HandCmd   = unitree_hg::msg::dds_::HandCmd_;
using HandState = unitree_hg::msg::dds_::HandState_;

struct Hand
{
    std::string                          side;
    std::array<int, kNumJoints>          qpos_adr{};
    std::array<int, kNumJoints>          dof_adr{};
    std::array<double, kNumJoints>       lower{};
    std::array<double, kNumJoints>       upper{};

    // Where an undriven finger is held. Tracks the measurement while driven, so releasing
    // freezes the finger where it stopped rather than snapping it back.
    std::array<double, kNumJoints> hold{};
    std::array<bool, kNumJoints>   driven{};

    std::mutex                                 mutex;
    std::array<unitree_hg::msg::dds_::MotorCmd_, kNumJoints> command{};
    bool                                       have_command{ false };
    std::chrono::steady_clock::time_point      last_command{};

    unitree::robot::ChannelSubscriberPtr<HandCmd>  sub;
    unitree::robot::ChannelPublisherPtr<HandState> pub;
    HandState                                      state;
};

struct State
{
    std::thread       thread;
    std::atomic<bool> running{ false };
    std::array<Hand, 2> hands;
};

State& state()
{
    // Leaked on purpose: the physics thread ends in exit(0), and a static destructor
    // destroying this still-joinable std::thread would call std::terminate.
    static State* s = new State();
    return *s;
}

// Resolves the seven joints of one hand. False means this model has no such hand.
bool resolveHand(const mjModel* model, Hand& hand)
{
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        const std::string name = hand.side + "_hand_" + kJointSuffixes[i] + "_joint";
        const int         id   = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        if (id < 0)
        {
            return false;
        }
        hand.qpos_adr[i] = model->jnt_qposadr[id];
        hand.dof_adr[i]  = model->jnt_dofadr[id];
        hand.lower[i]    = model->jnt_range[2 * id];
        hand.upper[i]    = model->jnt_range[2 * id + 1];
    }
    return true;
}

void openChannels(Hand& hand)
{
    hand.sub.reset(new unitree::robot::ChannelSubscriber<HandCmd>("rt/dex3/" + hand.side + "/cmd"));
    hand.sub->InitChannel(
        [&hand](const void* message) {
            const auto* msg = static_cast<const HandCmd*>(message);
            if (msg->motor_cmd().size() < kNumJoints)
            {
                return;
            }
            std::lock_guard<std::mutex> lock(hand.mutex);
            std::copy_n(msg->motor_cmd().begin(), kNumJoints, hand.command.begin());
            hand.have_command = true;
            hand.last_command = std::chrono::steady_clock::now();
        },
        1);

    hand.pub.reset(
        new unitree::robot::ChannelPublisher<HandState>("rt/dex3/" + hand.side + "/state"));
    hand.pub->InitChannel();
    hand.state.motor_state().resize(kNumJoints);
    // press_sensor_state stays empty: the tactile sensors are not simulated, and zeros would
    // claim nothing is touching the fingers.
}

// Runs the PD the real finger motor runs, into the generalized-force channel rather than an
// actuator, since the fingers deliberately have none.
void driveHand(const mjModel* model, mjData* data, Hand& hand)
{
    std::array<unitree_hg::msg::dds_::MotorCmd_, kNumJoints> command{};
    bool                                                     driven = false;
    {
        std::lock_guard<std::mutex> lock(hand.mutex);
        const auto elapsed = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - hand.last_command)
                                 .count();
        driven  = hand.have_command && elapsed < kCommandTimeoutS;
        command = hand.command;
    }

    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        const double q  = data->qpos[hand.qpos_adr[i]];
        const double dq = data->qvel[hand.dof_adr[i]];

        hand.driven[i] = driven && statusOf(command[i].mode()) == kStatusFoc;

        double tau;
        if (hand.driven[i])
        {
            const double target =
                std::clamp(static_cast<double>(command[i].q()), hand.lower[i], hand.upper[i]);
            tau = static_cast<double>(command[i].tau()) +
                  static_cast<double>(command[i].kp()) * (target - q) +
                  static_cast<double>(command[i].kd()) *
                      (static_cast<double>(command[i].dq()) - dq);
            hand.hold[i] = q;
        }
        else
        {
            // Lock, timed out, or never commanded.
            tau = kHoldKp * (hand.hold[i] - q) - kHoldKd * dq;
        }
        data->qfrc_applied[hand.dof_adr[i]] =
            std::clamp(tau, -kEffortLimit[i], kEffortLimit[i]);
    }
}

void publishState(const mjData* data, Hand& hand)
{
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        auto& motor = hand.state.motor_state()[i];
        motor.q()   = static_cast<float>(data->qpos[hand.qpos_adr[i]]);
        motor.dq()  = static_cast<float>(data->qvel[hand.dof_adr[i]]);
        // The applied torque is exactly what the simulated motor produced.
        motor.tau_est() = static_cast<float>(data->qfrc_applied[hand.dof_adr[i]]);
        motor.mode()    = packMode(i, hand.driven[i] ? kStatusFoc : kStatusLock);
    }
    hand.pub->Write(hand.state);
}

void run(mjModel** model, mjData** data)
{
    auto& s = state();

    while (s.running && (*model == nullptr || *data == nullptr))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!s.running)
    {
        return;
    }

    for (auto& hand : s.hands)
    {
        if (!resolveHand(*model, hand))
        {
            std::fprintf(
                stderr,
                "[grove_g1] no %s Dex3 joints in this model; hand control is OFF\n",
                hand.side.c_str());
            s.running = false;
            return;
        }
    }
    for (auto& hand : s.hands)
    {
        openChannels(hand);
    }
    std::fprintf(stderr, "[grove_g1] Dex3 hands answering rt/dex3/{left,right}/{cmd,state}\n");

    const auto control_period =
        std::chrono::nanoseconds(std::chrono::seconds(1)) / kControlRateHz;
    const int publish_every = kControlRateHz / kStatePublishHz;
    int       tick          = 0;
    auto      next          = std::chrono::steady_clock::now();

    while (s.running)
    {
        // No sim.mtx, as in the SDK bridge: these are independent doubles, a torn read costs
        // one stale tick, and a 1 kHz lock would contend with physics.
        mjModel* m = *model;
        mjData*  d = *data;
        if (m != nullptr && d != nullptr)
        {
            for (auto& hand : s.hands)
            {
                driveHand(m, d, hand);
            }
            if (tick % publish_every == 0)
            {
                for (auto& hand : s.hands)
                {
                    publishState(d, hand);
                }
            }
        }
        ++tick;
        next += control_period;
        std::this_thread::sleep_until(next);
    }

    // qfrc_applied persists between steps, so a torque left here would act forever.
    if (*data != nullptr)
    {
        for (const auto& hand : s.hands)
        {
            for (const int dof : hand.dof_adr)
            {
                (*data)->qfrc_applied[dof] = 0.0;
            }
        }
    }
    for (auto& hand : s.hands)
    {
        if (hand.sub)
        {
            hand.sub->CloseChannel();
        }
        if (hand.pub)
        {
            hand.pub->CloseChannel();
        }
    }
}

}  // namespace

void StartDex3Handler(mjModel** model, mjData** data)
{
    auto& s = state();
    if (s.running)
    {
        return;
    }
    s.hands[0].side = "left";
    s.hands[1].side = "right";
    s.running       = true;
    s.thread        = std::thread([model, data] { run(model, data); });
}

void StopDex3Handler()
{
    auto& s = state();
    if (!s.thread.joinable())
    {
        s.running = false;
        return;
    }
    s.running = false;
    s.thread.join();
}

}  // namespace grove_g1
