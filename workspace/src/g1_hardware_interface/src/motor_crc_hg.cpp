/**
 * @file motor_crc_hg.cpp
 * @brief Vendored CRC32 implementation for checksumming LowCmd frames before publishing.
 */

#include "g1_hardware_interface/motor_crc_hg.hpp"

namespace g1_hardware_interface::vendored
{

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

}  // namespace g1_hardware_interface::vendored
