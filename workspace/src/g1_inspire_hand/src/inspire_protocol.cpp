/**
 * @file inspire_protocol.cpp
 * @brief Building and checking the Inspire hand's RS485 frames.
 */

#include "g1_inspire_hand/inspire_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace g1_inspire_hand
{
namespace
{

constexpr std::uint8_t kRequestHead0 = 0xEB;
constexpr std::uint8_t kRequestHead1 = 0x90;
constexpr std::uint8_t kReplyHead0   = 0x90;
constexpr std::uint8_t kReplyHead1   = 0xEB;
constexpr std::uint8_t kInspireRead  = 0x11;
constexpr std::uint8_t kInspireWrite = 0x12;

constexpr std::uint8_t kModbusRead      = 0x03;
constexpr std::uint8_t kModbusWrite     = 0x10;
constexpr std::uint8_t kModbusException = 0x80;

// Header, id, length, command, two address bytes: what precedes the data in an EB 90 frame.
constexpr std::size_t kInspireDataOffset = 7;
// Id, function, byte count: what precedes the data in a Modbus read reply.
constexpr std::size_t kModbusReplyDataOffset  = 3;
constexpr std::size_t kModbusWriteReplyBytes  = 8;
constexpr std::size_t kModbusExceptionBytes   = 5;
constexpr std::size_t kInspireWriteReplyBytes = 9;

constexpr double kGramsToNewtons = 9.80665e-3;

std::uint8_t lowByte(std::uint32_t value) { return static_cast<std::uint8_t>(value & 0xFFU); }
std::uint8_t highByte(std::uint32_t value)
{
    return static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

bool lengthFits(Framing framing, std::size_t length)
{
    return length >= 1 && length <= kMaxDataBytes &&
           (framing != Framing::kModbusRtu || length % 2 == 0);
}

/// Appends the trailer: the byte sum for EB 90, the CRC for Modbus.
void seal(Framing framing, Frame& frame)
{
    const std::span<const std::uint8_t> body{ frame.bytes.data(), frame.size };
    if (framing == Framing::kInspire)
    {
        frame.bytes[frame.size++] = inspireChecksum(body);
        return;
    }
    const std::uint16_t crc   = modbusCrc(body);
    frame.bytes[frame.size++] = lowByte(crc);
    frame.bytes[frame.size++] = highByte(crc);
}

bool sealedInspire(std::span<const std::uint8_t> reply)
{
    return reply.size() >= 2 && reply.back() == inspireChecksum(reply.first(reply.size() - 1));
}

bool sealedModbus(std::span<const std::uint8_t> reply)
{
    if (reply.size() < 3)
    {
        return false;
    }
    const std::uint16_t crc = modbusCrc(reply.first(reply.size() - 2));
    return reply[reply.size() - 2] == lowByte(crc) && reply.back() == highByte(crc);
}

}  // namespace

std::optional<Framing> framingFromName(std::string_view name)
{
    if (name == "inspire")
    {
        return Framing::kInspire;
    }
    if (name == "modbus_rtu")
    {
        return Framing::kModbusRtu;
    }
    return std::nullopt;
}

std::uint8_t inspireChecksum(std::span<const std::uint8_t> frame)
{
    std::uint32_t sum = 0;
    for (std::size_t i = 2; i < frame.size(); ++i)
    {
        sum += frame[i];
    }
    return lowByte(sum);
}

std::uint16_t modbusCrc(std::span<const std::uint8_t> bytes)
{
    std::uint16_t crc = 0xFFFF;
    for (const std::uint8_t byte : bytes)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1U) != 0 ? static_cast<std::uint16_t>((crc >> 1U) ^ 0xA001U) :
                                    static_cast<std::uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

std::optional<Frame>
encodeRead(Framing framing, std::uint8_t id, std::uint16_t address, std::size_t length)
{
    if (!lengthFits(framing, length))
    {
        return std::nullopt;
    }
    Frame frame;
    auto  push = [&frame](std::uint32_t byte) { frame.bytes[frame.size++] = lowByte(byte); };
    if (framing == Framing::kInspire)
    {
        // The length byte counts what follows it up to the checksum: command, address, count.
        push(kRequestHead0);
        push(kRequestHead1);
        push(id);
        push(4);
        push(kInspireRead);
        push(lowByte(address));
        push(highByte(address));
        push(static_cast<std::uint32_t>(length));
    }
    else
    {
        const auto registers = static_cast<std::uint32_t>(length / 2);
        push(id);
        push(kModbusRead);
        push(highByte(address));
        push(lowByte(address));
        push(highByte(registers));
        push(lowByte(registers));
    }
    seal(framing, frame);
    return frame;
}

std::optional<Frame> encodeWrite(
    Framing framing, std::uint8_t id, std::uint16_t address, std::span<const std::uint8_t> data)
{
    if (!lengthFits(framing, data.size()))
    {
        return std::nullopt;
    }
    Frame frame;
    auto  push = [&frame](std::uint32_t byte) { frame.bytes[frame.size++] = lowByte(byte); };
    if (framing == Framing::kInspire)
    {
        push(kRequestHead0);
        push(kRequestHead1);
        push(id);
        push(static_cast<std::uint32_t>(3 + data.size()));
        push(kInspireWrite);
        push(lowByte(address));
        push(highByte(address));
        for (const std::uint8_t byte : data)
        {
            push(byte);
        }
    }
    else
    {
        const auto registers = static_cast<std::uint32_t>(data.size() / 2);
        push(id);
        push(kModbusWrite);
        push(highByte(address));
        push(lowByte(address));
        push(highByte(registers));
        push(lowByte(registers));
        push(static_cast<std::uint32_t>(data.size()));
        // A register goes high byte first; the hand keeps each value low byte first.
        for (std::size_t i = 0; i + 1 < data.size(); i += 2)
        {
            push(data[i + 1]);
            push(data[i]);
        }
    }
    seal(framing, frame);
    return frame;
}

std::size_t readReplySize(Framing framing, std::size_t length)
{
    return framing == Framing::kInspire ? kInspireDataOffset + length + 1 :
                                          kModbusReplyDataOffset + length + 2;
}

std::size_t writeReplySize(Framing framing)
{
    return framing == Framing::kInspire ? kInspireWriteReplyBytes : kModbusWriteReplyBytes;
}

ReplyError decodeReadReply(
    Framing framing, std::uint8_t id, std::span<const std::uint8_t> reply,
    std::span<std::uint8_t> out)
{
    if (framing == Framing::kInspire)
    {
        if (reply.size() != readReplySize(framing, out.size()))
        {
            return "a reply of the wrong length";
        }
        if (reply[0] != kReplyHead0 || reply[1] != kReplyHead1)
        {
            return "a reply without the 90 EB header";
        }
        if (!sealedInspire(reply))
        {
            return "a checksum mismatch";
        }
        if (reply[2] != id || reply[4] != kInspireRead)
        {
            return "a reply to another request";
        }
        std::copy_n(reply.begin() + kInspireDataOffset, out.size(), out.begin());
        return {};
    }

    if (reply.size() == kModbusExceptionBytes && sealedModbus(reply) &&
        reply[1] == (kModbusRead | kModbusException))
    {
        return "a Modbus exception";
    }
    if (reply.size() != readReplySize(framing, out.size()))
    {
        return "a reply of the wrong length";
    }
    if (!sealedModbus(reply))
    {
        return "a CRC mismatch";
    }
    if (reply[0] != id || reply[1] != kModbusRead || reply[2] != out.size())
    {
        return "a reply to another request";
    }
    for (std::size_t i = 0; i + 1 < out.size(); i += 2)
    {
        out[i]     = reply[kModbusReplyDataOffset + i + 1];
        out[i + 1] = reply[kModbusReplyDataOffset + i];
    }
    return {};
}

ReplyError decodeWriteReply(
    Framing framing, std::uint8_t id, std::uint16_t address, std::size_t length,
    std::span<const std::uint8_t> reply)
{
    if (framing == Framing::kInspire)
    {
        if (reply.size() != kInspireWriteReplyBytes)
        {
            return "a reply of the wrong length";
        }
        if (reply[0] != kReplyHead0 || reply[1] != kReplyHead1)
        {
            return "a reply without the 90 EB header";
        }
        if (!sealedInspire(reply))
        {
            return "a checksum mismatch";
        }
        if (reply[2] != id || reply[4] != kInspireWrite)
        {
            return "a reply to another request";
        }
        return {};
    }

    if (reply.size() == kModbusExceptionBytes && sealedModbus(reply) &&
        reply[1] == (kModbusWrite | kModbusException))
    {
        return "a Modbus exception";
    }
    if (reply.size() != kModbusWriteReplyBytes)
    {
        return "a reply of the wrong length";
    }
    if (!sealedModbus(reply))
    {
        return "a CRC mismatch";
    }
    const std::size_t registers = length / 2;
    if (reply[0] != id || reply[1] != kModbusWrite || reply[2] != highByte(address) ||
        reply[3] != lowByte(address) || reply[4] != highByte(registers) ||
        reply[5] != lowByte(registers))
    {
        return "a reply to another request";
    }
    return {};
}

std::array<std::uint8_t, kWordBlockBytes>
packWords(const std::array<std::int16_t, kNumActuators>& values)
{
    std::array<std::uint8_t, kWordBlockBytes> bytes{};
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        const auto word  = static_cast<std::uint16_t>(values[i]);
        bytes[2 * i]     = lowByte(word);
        bytes[2 * i + 1] = highByte(word);
    }
    return bytes;
}

std::array<std::int16_t, kNumActuators>
unpackWords(std::span<const std::uint8_t, kWordBlockBytes> bytes)
{
    std::array<std::int16_t, kNumActuators> values{};
    for (std::size_t i = 0; i < kNumActuators; ++i)
    {
        values[i] = static_cast<std::int16_t>(
            static_cast<std::uint16_t>(bytes[2 * i]) |
            static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[2 * i + 1]) << 8U));
    }
    return values;
}

std::int16_t angleFromJoint(double joint_rad, const ActuatorMap& map)
{
    const double span  = map.open_rad - map.closed_rad;
    const double angle = static_cast<double>(kAngleOpen) * (joint_rad - map.closed_rad) / span;
    return static_cast<std::int16_t>(std::lround(
        std::clamp(angle, static_cast<double>(kAngleClosed), static_cast<double>(kAngleOpen))));
}

double jointFromAngle(std::int16_t angle, const ActuatorMap& map)
{
    const double fraction = static_cast<double>(std::clamp(angle, kAngleClosed, kAngleOpen)) /
                            static_cast<double>(kAngleOpen);
    return map.closed_rad + (fraction * (map.open_rad - map.closed_rad));
}

double effortFromForce(std::int16_t force_g, const ActuatorMap& map)
{
    return static_cast<double>(force_g) * kGramsToNewtons * map.lever_arm_m;
}

}  // namespace g1_inspire_hand
