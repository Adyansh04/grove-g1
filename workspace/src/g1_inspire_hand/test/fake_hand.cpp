/**
 * @file fake_hand.cpp
 * @brief The slave side of both framings, written apart from the master's so each checks the other.
 */

#include "fake_hand.hpp"

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <cstdlib>
#include <stdexcept>
#include <vector>

namespace g1_inspire_hand::test
{

FakeHand::FakeHand(std::uint8_t id)
  : id_(id)
  , master_fd_(::posix_openpt(O_RDWR | O_NOCTTY))
{
    if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0)
    {
        throw std::runtime_error("no pseudo-terminal for the fake hand");
    }
    std::array<char, 128> name{};
    if (::ptsname_r(master_fd_, name.data(), name.size()) != 0)
    {
        throw std::runtime_error("no name for the fake hand's pseudo-terminal");
    }
    port_ = name.data();
    // Measured angles start half open.
    std::array<std::int16_t, kNumActuators> half{};
    half.fill(500);
    setWords(reg::kAngleAct, half);
    setWords(reg::kAngleSet, half);
    thread_ = std::jthread([this](const std::stop_token& stop) { serve(stop); });
}

FakeHand::~FakeHand()
{
    thread_.request_stop();
    thread_.join();
    ::close(master_fd_);
}

std::array<std::int16_t, kNumActuators> FakeHand::words(std::uint16_t address) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return unpackWords(std::span<const std::uint8_t, kWordBlockBytes>{ memory_.data() + address,
                                                                       kWordBlockBytes });
}

void FakeHand::setWords(std::uint16_t address, const std::array<std::int16_t, kNumActuators>& values)
{
    const auto                        bytes = packWords(values);
    const std::lock_guard<std::mutex> lock(mutex_);
    std::copy(bytes.begin(), bytes.end(), memory_.begin() + address);
}

std::uint8_t FakeHand::byteAt(std::uint16_t address) const
{
    const std::lock_guard<std::mutex> lock(mutex_);
    return memory_[address];
}

void FakeHand::setByte(std::uint16_t address, std::uint8_t value)
{
    const std::lock_guard<std::mutex> lock(mutex_);
    memory_[address] = value;
}

void FakeHand::reply(std::span<const std::uint8_t> bytes) const
{
    // A short write would show up as a reply the master refuses, which the tests would catch.
    if (::write(master_fd_, bytes.data(), bytes.size()) < 0)
    {
        return;
    }
}

void FakeHand::store(std::uint16_t address, std::span<const std::uint8_t> bytes)
{
    std::copy(bytes.begin(), bytes.end(), memory_.begin() + address);
    if (address == reg::kAngleSet)
    {
        ++angle_writes_;
        for (std::size_t i = 0; i < kNumActuators; ++i)
        {
            const std::size_t at = reg::kAngleSet + (2 * i);
            if (memory_[at] != 0xFF || memory_[at + 1] != 0xFF)
            {
                memory_[reg::kAngleAct + (2 * i)]     = memory_[at];
                memory_[reg::kAngleAct + (2 * i) + 1] = memory_[at + 1];
            }
        }
    }
    if (address == reg::kClearError && memory_[reg::kClearError] == 1)
    {
        std::fill_n(memory_.begin() + reg::kError, kNumActuators, std::uint8_t{ 0 });
        memory_[reg::kClearError] = 0;
    }
}

std::size_t FakeHand::answerInspire(std::span<const std::uint8_t> frame)
{
    if (frame.size() < 4 || frame.size() < 5U + frame[3])
    {
        return 0;
    }
    const std::size_t total = 5U + frame[3];
    if (frame[total - 1] != inspireChecksum(frame.first(total - 1)) || frame[2] != id_)
    {
        return total;
    }
    ++requests_;
    if (silent_)
    {
        return total;
    }
    const std::uint8_t        command = frame[4];
    const auto                address = static_cast<std::uint16_t>(frame[5] | (frame[6] << 8));
    std::vector<std::uint8_t> out{ 0x90, 0xEB, id_ };
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (command == 0x11)
        {
            const std::uint8_t length = frame[7];
            out.insert(
                out.end(),
                { static_cast<std::uint8_t>(3 + length), command, frame[5], frame[6] });
            out.insert(out.end(), memory_.begin() + address, memory_.begin() + address + length);
        }
        else if (command == 0x12)
        {
            store(address, frame.subspan(7, frame[3] - 3U));
            out.insert(out.end(), { 4, command, frame[5], frame[6], 1 });
        }
        else
        {
            return total;
        }
    }
    out.push_back(inspireChecksum(out));
    reply(out);
    return total;
}

std::size_t FakeHand::answerModbus(std::span<const std::uint8_t> frame)
{
    if (frame.size() < 7)
    {
        return 0;
    }
    const std::uint8_t function = frame[1];
    if (function != 0x03 && function != 0x10)
    {
        return 1;  // not a request this hand knows: resynchronise on the next byte
    }
    const std::size_t total = function == 0x03 ? 8 : 9U + frame[6];
    if (frame.size() < total)
    {
        return 0;
    }
    const std::uint16_t crc = modbusCrc(frame.first(total - 2));
    if (frame[total - 2] != (crc & 0xFFU) || frame[total - 1] != (crc >> 8U) || frame[0] != id_)
    {
        return total;
    }
    ++requests_;
    if (silent_)
    {
        return total;
    }
    const auto                address = static_cast<std::uint16_t>((frame[2] << 8) | frame[3]);
    const std::size_t         length  = 2U * static_cast<std::size_t>((frame[4] << 8) | frame[5]);
    std::vector<std::uint8_t> out{ id_, function };
    {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (function == 0x03)
        {
            out.push_back(static_cast<std::uint8_t>(length));
            for (std::size_t i = 0; i < length; i += 2)
            {
                out.push_back(memory_[address + i + 1]);
                out.push_back(memory_[address + i]);
            }
        }
        else
        {
            std::vector<std::uint8_t> bytes(length);
            for (std::size_t i = 0; i < length; i += 2)
            {
                bytes[i]     = frame[7 + i + 1];
                bytes[i + 1] = frame[7 + i];
            }
            store(address, bytes);
            out.insert(out.end(), { frame[2], frame[3], frame[4], frame[5] });
        }
    }
    const std::uint16_t out_crc = modbusCrc(out);
    out.push_back(static_cast<std::uint8_t>(out_crc & 0xFFU));
    out.push_back(static_cast<std::uint8_t>(out_crc >> 8U));
    reply(out);
    return total;
}

void FakeHand::serve(const std::stop_token& stop)
{
    std::vector<std::uint8_t>     pending;
    std::array<std::uint8_t, 256> chunk{};
    while (!stop.stop_requested())
    {
        pollfd    ready{ .fd = master_fd_, .events = POLLIN, .revents = 0 };
        const int polled = ::poll(&ready, 1, 20);
        if (polled <= 0 || (ready.revents & POLLIN) == 0)
        {
            continue;
        }
        const ssize_t n = ::read(master_fd_, chunk.data(), chunk.size());
        if (n <= 0)
        {
            continue;
        }
        pending.insert(pending.end(), chunk.begin(), chunk.begin() + n);
        while (!pending.empty())
        {
            const std::size_t used =
                pending.front() == 0xEB ? answerInspire(pending) : answerModbus(pending);
            if (used == 0)
            {
                break;
            }
            pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(used));
        }
    }
}

}  // namespace g1_inspire_hand::test
