/**
 * @file test_protocol.cpp
 * @brief The hand's frames, against golden bytes: Unitree's G1 service for the hand's own frame,
 *        the Modbus RTU specification for the other.
 */

#include <gmock/gmock.h>

#include <vector>

#include "g1_inspire_hand/inspire_protocol.hpp"

namespace
{

using g1_inspire_hand::ActuatorMap;
using g1_inspire_hand::Framing;
using ::testing::ElementsAreArray;

std::vector<std::uint8_t> bytesOf(const std::optional<g1_inspire_hand::Frame>& frame)
{
    EXPECT_TRUE(frame.has_value());
    if (!frame)
    {
        return {};
    }
    const auto view = frame->view();
    return { view.begin(), view.end() };
}

// Six values that cover a full open, a close, "no change" and two in between.
constexpr std::array<std::int16_t, g1_inspire_hand::kNumActuators> kValues = { 1000, 500, 0,
                                                                               -1,   250, 750 };

TEST(InspireFrame, ReadMatchesUnitreesAngleRequest)
{
    // inspire.h GetPosition: EB 90 id 04 11 0A 06 0C, then the byte sum from the id on.
    EXPECT_THAT(
        bytesOf(
            g1_inspire_hand::encodeRead(Framing::kInspire, 1, g1_inspire_hand::reg::kAngleAct, 12)),
        ElementsAreArray({ 0xEB, 0x90, 0x01, 0x04, 0x11, 0x0A, 0x06, 0x0C, 0x32 }));
}

TEST(InspireFrame, WriteMatchesUnitreesOpenCommand)
{
    std::array<std::int16_t, g1_inspire_hand::kNumActuators> open{};
    open.fill(g1_inspire_hand::kAngleOpen);
    const auto data = g1_inspire_hand::packWords(open);
    EXPECT_THAT(
        bytesOf(g1_inspire_hand::encodeWrite(
            Framing::kInspire,
            1,
            g1_inspire_hand::reg::kAngleSet,
            data)),
        ElementsAreArray({ 0xEB, 0x90, 0x01, 0x0F, 0x12, 0xCE, 0x05, 0xE8, 0x03, 0xE8,
                           0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x77 }));
}

TEST(InspireFrame, ClearErrorMatchesUnitree)
{
    const std::array<std::uint8_t, 1> one{ 1 };
    EXPECT_THAT(
        bytesOf(g1_inspire_hand::encodeWrite(
            Framing::kInspire,
            1,
            g1_inspire_hand::reg::kClearError,
            one)),
        ElementsAreArray({ 0xEB, 0x90, 0x01, 0x04, 0x12, 0xEC, 0x03, 0x01, 0x07 }));
}

TEST(InspireFrame, ReadReplyGivesTheValuesLowByteFirst)
{
    const std::array<std::uint8_t, 20> reply = { 0x90, 0xEB, 0x01, 0x0F, 0x11, 0x0A, 0x06,
                                                 0xE8, 0x03, 0xF4, 0x01, 0x00, 0x00, 0xFF,
                                                 0xFF, 0xFA, 0x00, 0xEE, 0x02, 0xF9 };
    std::array<std::uint8_t, g1_inspire_hand::kWordBlockBytes> data{};
    EXPECT_TRUE(g1_inspire_hand::decodeReadReply(Framing::kInspire, 1, reply, data).empty());
    EXPECT_THAT(g1_inspire_hand::unpackWords(data), ElementsAreArray(kValues));
}

TEST(InspireFrame, ReadReplyRefusesACorruptOrForeignFrame)
{
    std::array<std::uint8_t, 20> reply = { 0x90, 0xEB, 0x01, 0x0F, 0x11, 0x0A, 0x06,
                                           0xE8, 0x03, 0xF4, 0x01, 0x00, 0x00, 0xFF,
                                           0xFF, 0xFA, 0x00, 0xEE, 0x02, 0xF9 };
    std::array<std::uint8_t, g1_inspire_hand::kWordBlockBytes> data{};
    EXPECT_FALSE(g1_inspire_hand::decodeReadReply(Framing::kInspire, 2, reply, data).empty());
    EXPECT_FALSE(
        g1_inspire_hand::decodeReadReply(Framing::kInspire, 1, std::span{ reply }.first(19), data)
            .empty());
    reply[9] ^= 0x01U;
    EXPECT_EQ(
        g1_inspire_hand::decodeReadReply(Framing::kInspire, 1, reply, data),
        "a checksum mismatch");
}

TEST(ModbusFrame, CrcMatchesTheSpecificationsExample)
{
    // Modbus over serial line, the read of ten registers from address 0: CRC C5 CD.
    const std::array<std::uint8_t, 6> request{ 0x01, 0x03, 0x00, 0x00, 0x00, 0x0A };
    EXPECT_EQ(g1_inspire_hand::modbusCrc(request), 0xCDC5);
}

TEST(ModbusFrame, ReadAsksForRegistersAtTheByteAddress)
{
    EXPECT_THAT(
        bytesOf(
            g1_inspire_hand::encodeRead(Framing::kModbusRtu, 1, g1_inspire_hand::reg::kAngleAct, 12)),
        ElementsAreArray({ 0x01, 0x03, 0x06, 0x0A, 0x00, 0x06, 0xE5, 0x42 }));
}

TEST(ModbusFrame, WriteSendsEachRegisterHighByteFirst)
{
    const auto data = g1_inspire_hand::packWords(kValues);
    EXPECT_THAT(
        bytesOf(g1_inspire_hand::encodeWrite(
            Framing::kModbusRtu,
            1,
            g1_inspire_hand::reg::kAngleSet,
            data)),
        ElementsAreArray({ 0x01, 0x10, 0x05, 0xCE, 0x00, 0x06, 0x0C, 0x03, 0xE8, 0x01, 0xF4,
                           0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFA, 0x02, 0xEE, 0x72, 0x12 }));
}

TEST(ModbusFrame, ReadReplyComesBackInTheHandsByteOrder)
{
    const std::array<std::uint8_t, 17> reply = { 0x01, 0x03, 0x0C, 0x03, 0xE8, 0x01,
                                                 0xF4, 0x00, 0x00, 0xFF, 0xFF, 0x00,
                                                 0xFA, 0x02, 0xEE, 0x2C, 0xD0 };
    std::array<std::uint8_t, g1_inspire_hand::kWordBlockBytes> data{};
    EXPECT_TRUE(g1_inspire_hand::decodeReadReply(Framing::kModbusRtu, 1, reply, data).empty());
    EXPECT_THAT(g1_inspire_hand::unpackWords(data), ElementsAreArray(kValues));
}

TEST(ModbusFrame, WriteReplyAndExceptionAreTold)
{
    const std::array<std::uint8_t, 8> ok = { 0x01, 0x10, 0x05, 0xCE, 0x00, 0x06, 0x21, 0x38 };
    EXPECT_TRUE(g1_inspire_hand::decodeWriteReply(
                    Framing::kModbusRtu,
                    1,
                    g1_inspire_hand::reg::kAngleSet,
                    12,
                    ok)
                    .empty());
    // The same reply cannot answer a write elsewhere.
    EXPECT_FALSE(g1_inspire_hand::decodeWriteReply(
                     Framing::kModbusRtu,
                     1,
                     g1_inspire_hand::reg::kForceSet,
                     12,
                     ok)
                     .empty());
    const std::array<std::uint8_t, 5> refused = { 0x01, 0x83, 0x02, 0xC0, 0xF1 };
    std::array<std::uint8_t, g1_inspire_hand::kWordBlockBytes> data{};
    EXPECT_EQ(
        g1_inspire_hand::decodeReadReply(Framing::kModbusRtu, 1, refused, data),
        "a Modbus exception");
}

TEST(Frame, LengthsTheFramingCannotCarryAreRefused)
{
    EXPECT_FALSE(g1_inspire_hand::encodeRead(Framing::kInspire, 1, 1546, 0).has_value());
    EXPECT_FALSE(
        g1_inspire_hand::encodeRead(Framing::kInspire, 1, 1546, g1_inspire_hand::kMaxDataBytes + 1)
            .has_value());
    // A Modbus register is two bytes, so an odd run cannot be asked for.
    EXPECT_FALSE(g1_inspire_hand::encodeRead(Framing::kModbusRtu, 1, 1546, 3).has_value());
    EXPECT_TRUE(g1_inspire_hand::encodeRead(Framing::kInspire, 1, 1546, 3).has_value());
}

TEST(Frame, NamesPickTheFraming)
{
    EXPECT_EQ(g1_inspire_hand::framingFromName("inspire"), Framing::kInspire);
    EXPECT_EQ(g1_inspire_hand::framingFromName("modbus_rtu"), Framing::kModbusRtu);
    EXPECT_FALSE(g1_inspire_hand::framingFromName("modbus").has_value());
}

TEST(ActuatorMapping, FingerAnglesFollowUnitreesMapping)
{
    // xr_teleoperate: value (1.7 - q) / 1.7, so 0 rad is open and 1.7 rad is closed.
    const ActuatorMap finger{ .open_rad = 0.0, .closed_rad = 1.7, .lever_arm_m = 0.05 };
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(0.0, finger), 1000);
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(1.7, finger), 0);
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(0.85, finger), 500);
    // Past either end clamps rather than wrapping into the other.
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(-0.3, finger), 1000);
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(2.5, finger), 0);
    EXPECT_NEAR(g1_inspire_hand::jointFromAngle(250, finger), 1.275, 1e-9);
}

TEST(ActuatorMapping, ThumbRotationRunsFromBesideTheIndexToOpposed)
{
    const ActuatorMap rotation{ .open_rad = -0.1, .closed_rad = 1.3, .lever_arm_m = 0.03 };
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(-0.1, rotation), 1000);
    EXPECT_EQ(g1_inspire_hand::angleFromJoint(1.3, rotation), 0);
    for (const double rad : { -0.1, 0.2, 0.6, 1.0, 1.3 })
    {
        // One count is 1.4 mrad.
        EXPECT_NEAR(
            g1_inspire_hand::jointFromAngle(g1_inspire_hand::angleFromJoint(rad, rotation), rotation),
            rad,
            1e-3);
    }
    // "No change" is never measured; it clamps into the range rather than past it.
    EXPECT_NEAR(g1_inspire_hand::jointFromAngle(-1, rotation), 1.3, 1e-9);
}

TEST(ActuatorMapping, ForceBecomesTheTorqueItWouldPutOnTheJoint)
{
    const ActuatorMap finger{ .open_rad = 0.0, .closed_rad = 1.7, .lever_arm_m = 0.05 };
    // 1000 g is 9.81 N, 50 mm out: 0.49 N m.
    EXPECT_NEAR(g1_inspire_hand::effortFromForce(1000, finger), 0.4903, 1e-4);
    EXPECT_NEAR(g1_inspire_hand::effortFromForce(-200, finger), -0.0981, 1e-4);
}

}  // namespace
