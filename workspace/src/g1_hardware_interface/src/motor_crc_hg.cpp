/**
 * @file motor_crc_hg.cpp
 * @brief Vendored CRC32 implementation for checksumming unitree_hg LowCmd messages
 *        before they are published.
 */

#include "g1_hardware_interface/motor_crc_hg.hpp"

#include <array>
#include <cstddef>
#include <cstring>

namespace g1_hardware_interface::vendored
{

namespace
{
/**
 * @brief Raw, wire-layout mirror of one unitree_hg::msg::LowCmd motor_cmd entry.
 *
 * Mirrors unitree_hg::msg::LowCmd's field layout exactly (mode_pr, mode_machine,
 * motor_cmd[35], reserve[4], crc) -- naturally aligned, not literally packed, so the
 * compiler inserts real padding between/after these fields (see the static_assert
 * below, which only holds because of that padding). The CRC is computed over the raw
 * bytes including that padding, so the field order and types must match the
 * firmware/motion-service's own struct bit-for-bit, and the padding bytes themselves
 * must be zeroed explicitly (computeLowCmdCrc() does this via memset) rather than
 * relied on to come out zeroed from aggregate-init `{}`, which the standard doesn't
 * guarantee for padding.
 */
struct RawMotorCmd
{
    std::uint8_t  mode;
    float         q;
    float         dq;
    float         tau;
    float         kp;
    float         kd;
    std::uint32_t reserve;
};

struct RawLowCmd
{
    std::uint8_t                 mode_pr;
    std::uint8_t                 mode_machine;
    std::array<RawMotorCmd, 35>  motor_cmd;
    std::array<std::uint32_t, 4> reserve;
    std::uint32_t                crc;
};

static_assert(
    sizeof(RawLowCmd) == 1004,
    "RawLowCmd must mirror unitree_hg::msg::LowCmd's wire layout exactly -- if this fires, the "
    "struct no longer matches what the firmware/motion-service expects the CRC to cover");
}  // namespace

std::uint32_t crc32Core(const std::uint32_t* ptr, std::uint32_t len)
{
    std::uint32_t           crc32       = 0xFFFFFFFF;
    constexpr std::uint32_t kPolynomial = 0x04c11db7;
    for (std::uint32_t i = 0; i < len; ++i)
    {
        std::uint32_t       xbit = 1U << 31;
        const std::uint32_t data = ptr[i];
        for (std::uint32_t bit = 0; bit < 32; ++bit)
        {
            const bool msb_set = (crc32 & 0x80000000) != 0;
            crc32 <<= 1;
            if (msb_set)
            {
                crc32 ^= kPolynomial;
            }
            // Kept byte-for-byte as upstream writes it; see the file header.
            // NOLINTNEXTLINE(readability-implicit-bool-conversion)
            if (data & xbit)
            {
                crc32 ^= kPolynomial;
            }
            xbit >>= 1;
        }
    }
    return crc32;
}

void computeLowCmdCrc(unitree_hg::msg::LowCmd& msg)
{
    RawLowCmd raw;
    /*
     * Explicit, not aggregate-init `{}': guarantees the alignment padding
     * the CRC covers is actually zero, rather than relying on
     * compiler-specific (if universally observed) zero-init-of-padding
     * behavior the standard doesn't formally promise.
     */
    std::memset(&raw, 0, sizeof(raw));
    raw.mode_pr      = msg.mode_pr;
    raw.mode_machine = msg.mode_machine;
    for (std::size_t i = 0; i < raw.motor_cmd.size(); ++i)
    {
        raw.motor_cmd[i].mode    = msg.motor_cmd[i].mode;
        raw.motor_cmd[i].q       = msg.motor_cmd[i].q;
        raw.motor_cmd[i].dq      = msg.motor_cmd[i].dq;
        raw.motor_cmd[i].tau     = msg.motor_cmd[i].tau;
        raw.motor_cmd[i].kp      = msg.motor_cmd[i].kp;
        raw.motor_cmd[i].kd      = msg.motor_cmd[i].kd;
        raw.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
    }
    raw.reserve = msg.reserve;

    raw.crc = crc32Core(
        reinterpret_cast<const std::uint32_t*>(&raw),
        static_cast<std::uint32_t>(sizeof(RawLowCmd) >> 2) - 1);
    msg.crc = raw.crc;
}

}  // namespace g1_hardware_interface::vendored
