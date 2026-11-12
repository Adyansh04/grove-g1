#ifndef G1_INSPIRE_HAND__INSPIRE_PROTOCOL_HPP_
#define G1_INSPIRE_HAND__INSPIRE_PROTOCOL_HPP_

/**
 * @file inspire_protocol.hpp
 * @brief The Inspire RH56 hand's register map and its two RS485 framings, without any I/O.
 *
 * The hand is a register device: every quantity is a run of bytes at a fixed address, six
 * little-endian 16-bit values for the six actuators or one byte each. Two framings carry the same
 * map over RS485. `kInspire` is the hand's own frame (`EB 90` header, byte-sum checksum), which
 * Unitree's G1 hand service speaks; `kModbusRtu` is Modbus RTU (function 0x03 read, 0x10 write,
 * CRC-16), which newer firmware also answers. In Modbus the register address is the byte address
 * and each register carries two bytes, high byte first on the wire.
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace g1_inspire_hand
{

/// Actuators per hand, in register order: pinky, ring, middle, index, thumb bend, thumb rotation.
inline constexpr std::size_t kNumActuators = 6;

/// Byte addresses of the registers this package uses.
namespace reg
{
inline constexpr std::uint16_t kClearError = 1004;  ///< write 1: clears every actuator's error
inline constexpr std::uint16_t kAngleSet   = 1486;  ///< six int16, 0 closed to 1000 open, -1 none
inline constexpr std::uint16_t kForceSet   = 1498;  ///< six int16, grams: a closing finger stops
inline constexpr std::uint16_t kSpeedSet   = 1522;  ///< six int16, 0 to 1000
inline constexpr std::uint16_t kAngleAct   = 1546;  ///< six int16, measured, 0 to 1000
inline constexpr std::uint16_t kForceAct   = 1582;  ///< six int16, measured, grams, signed
inline constexpr std::uint16_t kCurrent    = 1594;  ///< six int16, mA
inline constexpr std::uint16_t kError      = 1606;  ///< six bytes, ErrorBit flags
inline constexpr std::uint16_t kStatus     = 1612;  ///< six bytes, Status values
inline constexpr std::uint16_t kTemp       = 1618;  ///< six bytes, deg C
}  // namespace reg

/// Bytes in a block of six int16 values.
inline constexpr std::size_t kWordBlockBytes = 2 * kNumActuators;
/// Bytes from kCurrent to the end of kTemp, read as one diagnostics block.
inline constexpr std::size_t kDiagnosticsBytes = reg::kTemp + kNumActuators - reg::kCurrent;

/// ANGLE_SET value that leaves an actuator's target as it is.
inline constexpr std::int16_t kAngleNoChange = -1;
inline constexpr std::int16_t kAngleClosed   = 0;
inline constexpr std::int16_t kAngleOpen     = 1000;

/// Flags in an actuator's ERROR byte.
enum class ErrorBit : std::uint8_t
{
    kLockedRotor   = 0x01,
    kOverTemp      = 0x02,
    kOverCurrent   = 0x04,
    kMotorAbnormal = 0x08,
    kComms         = 0x10,
};

/// An actuator's STATUS byte.
enum class Status : std::uint8_t
{
    kReleasing         = 0,  ///< opening
    kGrasping          = 1,  ///< closing
    kReachedTarget     = 2,  ///< stopped at its target
    kReachedForce      = 3,  ///< stopped closing at FORCE_SET
    kCurrentProtection = 5,
    kLockedRotor       = 6,
    kFault             = 7,
};

enum class Framing : std::uint8_t
{
    kInspire,
    kModbusRtu,
};

/**
 * @brief The framing a parameter names.
 *
 * @param name "inspire" or "modbus_rtu".
 * @return Empty for any other name.
 */
[[nodiscard]] std::optional<Framing> framingFromName(std::string_view name);

/// Longest request or reply this package builds or accepts, in bytes.
inline constexpr std::size_t kMaxFrameBytes = 64;
/// Longest data run one request may carry.
inline constexpr std::size_t kMaxDataBytes = 40;

/// A frame on the wire, in a fixed buffer so building one never allocates.
struct Frame
{
    std::array<std::uint8_t, kMaxFrameBytes> bytes{};
    std::size_t                              size{ 0 };

    [[nodiscard]] std::span<const std::uint8_t> view() const { return { bytes.data(), size }; }
};

/**
 * @brief The byte-sum checksum of an `EB 90` frame: every byte after the header, modulo 256.
 *
 * @param frame The frame without its checksum byte.
 */
[[nodiscard]] std::uint8_t inspireChecksum(std::span<const std::uint8_t> frame);

/**
 * @brief Modbus CRC-16 (polynomial 0xA001 reflected, initial 0xFFFF), sent low byte first.
 */
[[nodiscard]] std::uint16_t modbusCrc(std::span<const std::uint8_t> bytes);

/**
 * @brief A request to read @p length bytes from @p address.
 *
 * @param length Bytes, 1 to kMaxDataBytes; even for Modbus, whose registers hold two.
 * @return Empty when @p length is out of range.
 */
[[nodiscard]] std::optional<Frame>
encodeRead(Framing framing, std::uint8_t id, std::uint16_t address, std::size_t length);

/**
 * @brief A request to write @p data, in the hand's byte order, starting at @p address.
 *
 * @param data 1 to kMaxDataBytes bytes; an even count for Modbus.
 * @return Empty when @p data is out of range.
 */
[[nodiscard]] std::optional<Frame> encodeWrite(
    Framing framing, std::uint8_t id, std::uint16_t address, std::span<const std::uint8_t> data);

/// Bytes the hand sends back for a read of @p length bytes.
[[nodiscard]] std::size_t readReplySize(Framing framing, std::size_t length);

/// Bytes the hand sends back for a write.
[[nodiscard]] std::size_t writeReplySize(Framing framing);

/// Why a reply was refused, or empty for a good one. Points at static text.
using ReplyError = std::string_view;

/**
 * @brief Checks a read reply and copies its data out in the hand's byte order.
 *
 * @param reply What arrived, exactly readReplySize() bytes for a good reply.
 * @param out Receives the data; its size is the length that was asked for.
 * @return Empty on success, else why the reply was refused (wrong size, header, id, command,
 *         checksum, or a Modbus exception). The address an `EB 90` reply echoes is not checked:
 *         Unitree's own service reads replies without it.
 */
[[nodiscard]] ReplyError decodeReadReply(
    Framing framing, std::uint8_t id, std::span<const std::uint8_t> reply,
    std::span<std::uint8_t> out);

/**
 * @brief Checks a write reply.
 *
 * @return Empty on success, else why the reply was refused.
 */
[[nodiscard]] ReplyError decodeWriteReply(
    Framing framing, std::uint8_t id, std::uint16_t address, std::size_t length,
    std::span<const std::uint8_t> reply);

/// Six int16 values as the hand stores them, little-endian.
[[nodiscard]] std::array<std::uint8_t, kWordBlockBytes>
packWords(const std::array<std::int16_t, kNumActuators>& values);

/// The six int16 values in a block of the hand's bytes.
[[nodiscard]] std::array<std::int16_t, kNumActuators>
unpackWords(std::span<const std::uint8_t, kWordBlockBytes> bytes);

/**
 * @brief How one actuator's ANGLE value maps to its joint, and its force to an effort.
 *
 * Linear: ANGLE 1000 is @c open_rad and ANGLE 0 is @c closed_rad.
 */
struct ActuatorMap
{
    double open_rad{ 0.0 };
    double closed_rad{ 0.0 };
    /// m. A finger force acting this far from the joint loads it by the effort reported.
    double lever_arm_m{ 0.0 };
};

/**
 * @brief The ANGLE value for a joint angle, clamped to 0..1000.
 *
 * @param joint_rad Joint angle, rad. Must be finite: a caller holds its last target instead.
 */
[[nodiscard]] std::int16_t angleFromJoint(double joint_rad, const ActuatorMap& map);

/// The joint angle, rad, for an ANGLE value.
[[nodiscard]] double jointFromAngle(std::int16_t angle, const ActuatorMap& map);

/// The joint effort, N m, for a measured force in grams.
[[nodiscard]] double effortFromForce(std::int16_t force_g, const ActuatorMap& map);

}  // namespace g1_inspire_hand

#endif  // G1_INSPIRE_HAND__INSPIRE_PROTOCOL_HPP_
