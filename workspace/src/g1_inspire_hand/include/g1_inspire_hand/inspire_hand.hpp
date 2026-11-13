#ifndef G1_INSPIRE_HAND__INSPIRE_HAND_HPP_
#define G1_INSPIRE_HAND__INSPIRE_HAND_HPP_

/**
 * @file inspire_hand.hpp
 * @brief One hand's registers over a byte stream: a request, then its reply, one at a time.
 */

#include <array>
#include <chrono>
#include <cstdint>
#include <span>

#include "g1_inspire_hand/inspire_protocol.hpp"
#include "g1_inspire_hand/serial_port.hpp"

namespace g1_inspire_hand
{

/**
 * @brief Register reads and writes on one hand. Not thread-safe: one caller at a time.
 */
class InspireHand
{
public:
    /**
     * @param stream The line to the hand, borrowed: it must outlive this object.
     * @param id The hand's bus id, 1 by default.
     * @param reply_timeout How long a reply may take before the request counts as lost.
     */
    InspireHand(
        ByteStream& stream, Framing framing, std::uint8_t id,
        std::chrono::microseconds reply_timeout);

    /**
     * @brief Reads @p out.size() bytes from @p address, in the hand's byte order.
     *
     * @return Empty on success, else why it failed.
     */
    [[nodiscard]] ReplyError read(std::uint16_t address, std::span<std::uint8_t> out);

    /**
     * @brief Writes @p data, in the hand's byte order, from @p address.
     *
     * @return Empty on success, else why it failed.
     */
    [[nodiscard]] ReplyError write(std::uint16_t address, std::span<const std::uint8_t> data);

    /// Reads six int16 values, one per actuator.
    [[nodiscard]] ReplyError
    readWords(std::uint16_t address, std::array<std::int16_t, kNumActuators>& out);

    /// Writes six int16 values, one per actuator.
    [[nodiscard]] ReplyError
    writeWords(std::uint16_t address, const std::array<std::int16_t, kNumActuators>& values);

    /// Clears every actuator's latched error.
    [[nodiscard]] ReplyError clearErrors();

private:
    ByteStream*               stream_;  ///< borrowed, never null
    Framing                   framing_;
    std::uint8_t              id_;
    std::chrono::microseconds reply_timeout_;
};

}  // namespace g1_inspire_hand

#endif  // G1_INSPIRE_HAND__INSPIRE_HAND_HPP_
