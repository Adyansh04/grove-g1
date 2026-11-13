/**
 * @file inspire_hand.cpp
 * @brief Request and reply on one hand's line.
 */

#include "g1_inspire_hand/inspire_hand.hpp"

namespace g1_inspire_hand
{

InspireHand::InspireHand(
    ByteStream& stream, Framing framing, std::uint8_t id, std::chrono::microseconds reply_timeout)
  : stream_(&stream)
  , framing_(framing)
  , id_(id)
  , reply_timeout_(reply_timeout)
{}

ReplyError InspireHand::read(std::uint16_t address, std::span<std::uint8_t> out)
{
    const std::optional<Frame> request = encodeRead(framing_, id_, address, out.size());
    if (!request)
    {
        return "a read of a length the framing cannot carry";
    }
    // The tail of an earlier late reply would otherwise read as the start of this one.
    stream_->discardInput();
    if (!stream_->write(request->view()))
    {
        return "the line refused the request";
    }
    std::array<std::uint8_t, kMaxFrameBytes> reply{};
    const std::size_t                        want = readReplySize(framing_, out.size());
    const std::size_t                        got  = stream_->read(
        std::span{ reply.data(), want },
        std::chrono::steady_clock::now() + reply_timeout_);
    if (got == 0)
    {
        return "no reply";
    }
    return decodeReadReply(framing_, id_, std::span{ reply.data(), got }, out);
}

ReplyError InspireHand::write(std::uint16_t address, std::span<const std::uint8_t> data)
{
    const std::optional<Frame> request = encodeWrite(framing_, id_, address, data);
    if (!request)
    {
        return "a write of a length the framing cannot carry";
    }
    stream_->discardInput();
    if (!stream_->write(request->view()))
    {
        return "the line refused the request";
    }
    std::array<std::uint8_t, kMaxFrameBytes> reply{};
    const std::size_t                        want = writeReplySize(framing_);
    const std::size_t                        got  = stream_->read(
        std::span{ reply.data(), want },
        std::chrono::steady_clock::now() + reply_timeout_);
    if (got == 0)
    {
        return "no reply";
    }
    return decodeWriteReply(framing_, id_, address, data.size(), std::span{ reply.data(), got });
}

ReplyError
InspireHand::readWords(std::uint16_t address, std::array<std::int16_t, kNumActuators>& out)
{
    std::array<std::uint8_t, kWordBlockBytes> bytes{};
    const ReplyError                          error = read(address, bytes);
    if (error.empty())
    {
        out = unpackWords(bytes);
    }
    return error;
}

ReplyError
InspireHand::writeWords(std::uint16_t address, const std::array<std::int16_t, kNumActuators>& values)
{
    return write(address, packWords(values));
}

ReplyError InspireHand::clearErrors()
{
    // One byte in the hand's own frame. A Modbus register is two bytes: the second is the flash
    // save flag at 1005, and zero leaves the flash alone.
    constexpr std::array<std::uint8_t, 2> kClear{ 1, 0 };
    return write(
        reg::kClearError,
        framing_ == Framing::kInspire ? std::span<const std::uint8_t>{ kClear.data(), 1 } :
                                        std::span<const std::uint8_t>{ kClear });
}

}  // namespace g1_inspire_hand
