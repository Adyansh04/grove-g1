#ifndef G1_INSPIRE_HAND__TEST__FAKE_HAND_HPP_
#define G1_INSPIRE_HAND__TEST__FAKE_HAND_HPP_

/**
 * @file fake_hand.hpp
 * @brief A hand on the far end of a pseudo-terminal, answering either framing, for tests.
 */

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <span>
#include <stop_token>
#include <string>
#include <thread>

#include "g1_inspire_hand/inspire_protocol.hpp"

namespace g1_inspire_hand::test
{

/**
 * @brief Answers reads and writes of a byte-addressed register file. A target written to ANGLE_SET
 *        is reached at once, except for values of -1, which leave the actuator where it is.
 */
class FakeHand
{
public:
    explicit FakeHand(std::uint8_t id = 1);
    FakeHand(const FakeHand&)            = delete;
    FakeHand& operator=(const FakeHand&) = delete;
    FakeHand(FakeHand&&)                 = delete;
    FakeHand& operator=(FakeHand&&)      = delete;
    ~FakeHand();

    /// The device a master opens.
    [[nodiscard]] const std::string& port() const { return port_; }

    /// Stops answering, as an unpowered hand does.
    void setSilent(bool silent) { silent_ = silent; }

    [[nodiscard]] std::array<std::int16_t, kNumActuators> words(std::uint16_t address) const;
    void setWords(std::uint16_t address, const std::array<std::int16_t, kNumActuators>& values);
    [[nodiscard]] std::uint8_t byteAt(std::uint16_t address) const;
    void                       setByte(std::uint16_t address, std::uint8_t value);

    /// Requests answered, and writes to ANGLE_SET among them.
    [[nodiscard]] int requests() const { return requests_; }
    [[nodiscard]] int angleWrites() const { return angle_writes_; }

private:
    void serve(const std::stop_token& stop);
    /// Answers the request at the start of @p frame; returns the bytes it used, 0 for "need more".
    std::size_t answerInspire(std::span<const std::uint8_t> frame);
    std::size_t answerModbus(std::span<const std::uint8_t> frame);
    /// Stores bytes in the hand's order and runs what a write there does. Holds mutex_.
    void store(std::uint16_t address, std::span<const std::uint8_t> bytes);
    void reply(std::span<const std::uint8_t> bytes) const;

    std::uint8_t id_;
    int          master_fd_{ -1 };
    std::string  port_;

    mutable std::mutex             mutex_;
    std::array<std::uint8_t, 2048> memory_{};  // guarded by mutex_
    std::atomic<bool>              silent_{ false };
    std::atomic<int>               requests_{ 0 };
    std::atomic<int>               angle_writes_{ 0 };
    std::jthread                   thread_;
};

}  // namespace g1_inspire_hand::test

#endif  // G1_INSPIRE_HAND__TEST__FAKE_HAND_HPP_
