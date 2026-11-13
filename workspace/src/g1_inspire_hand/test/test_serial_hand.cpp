/**
 * @file test_serial_hand.cpp
 * @brief The serial line and the register client against a fake hand, in both framings.
 */

#include <gmock/gmock.h>

#include "fake_hand.hpp"
#include "g1_inspire_hand/inspire_hand.hpp"
#include "g1_inspire_hand/serial_port.hpp"

namespace
{

using g1_inspire_hand::Framing;
using g1_inspire_hand::InspireHand;
using g1_inspire_hand::SerialPort;
using g1_inspire_hand::test::FakeHand;
namespace reg = g1_inspire_hand::reg;
using namespace std::chrono_literals;

class SerialHand : public ::testing::TestWithParam<Framing>
{
protected:
    void SetUp() override
    {
        std::string error;
        port_ = SerialPort::open(fake_.port(), 115200, error);
        ASSERT_NE(port_, nullptr) << error;
        hand_ = std::make_unique<InspireHand>(*port_, GetParam(), 1, 50ms);
    }

    FakeHand                     fake_;
    std::unique_ptr<SerialPort>  port_;
    std::unique_ptr<InspireHand> hand_;
};

TEST_P(SerialHand, ReadsAndWritesTheRegisters)
{
    std::array<std::int16_t, 6> angles{};
    ASSERT_EQ(hand_->readWords(reg::kAngleAct, angles), "");
    EXPECT_THAT(angles, ::testing::Each(500));

    // -1 leaves the index where it was.
    ASSERT_EQ(hand_->writeWords(reg::kAngleSet, { 1000, 0, 250, -1, 750, 100 }), "");
    ASSERT_EQ(hand_->readWords(reg::kAngleAct, angles), "");
    EXPECT_THAT(angles, ::testing::ElementsAre(1000, 0, 250, 500, 750, 100));
    EXPECT_EQ(fake_.angleWrites(), 1);
}

TEST_P(SerialHand, ReadsTheOneByteRegisters)
{
    fake_.setByte(reg::kStatus + 1, 3);
    fake_.setByte(reg::kTemp + 5, 41);
    std::array<std::uint8_t, g1_inspire_hand::kDiagnosticsBytes> block{};
    ASSERT_EQ(hand_->read(reg::kCurrent, block), "");
    EXPECT_EQ(block[reg::kStatus - reg::kCurrent + 1], 3);
    EXPECT_EQ(block[reg::kTemp - reg::kCurrent + 5], 41);
}

TEST_P(SerialHand, ClearingErrorsZeroesThem)
{
    fake_.setByte(reg::kError + 2, 0x01);
    ASSERT_EQ(hand_->clearErrors(), "");
    EXPECT_EQ(fake_.byteAt(reg::kError + 2), 0);
    // The flash save flag next to it is left alone.
    EXPECT_EQ(fake_.byteAt(reg::kClearError + 1), 0);
}

TEST_P(SerialHand, ASilentHandCostsTheReplyTimeout)
{
    fake_.setSilent(true);
    std::array<std::int16_t, 6> angles{};
    const auto                  start = std::chrono::steady_clock::now();
    EXPECT_EQ(hand_->readWords(reg::kAngleAct, angles), "no reply");
    const auto spent = std::chrono::steady_clock::now() - start;
    EXPECT_GE(spent, 45ms);
    EXPECT_LT(spent, 500ms);
}

INSTANTIATE_TEST_SUITE_P(
    BothFramings, SerialHand, ::testing::Values(Framing::kInspire, Framing::kModbusRtu),
    [](const ::testing::TestParamInfo<Framing>& info) {
        return info.param == Framing::kInspire ? "Inspire" : "ModbusRtu";
    });

TEST(SerialLine, RefusesARateTheHandDoesNotTake)
{
    FakeHand    fake;
    std::string error;
    EXPECT_EQ(SerialPort::open(fake.port(), 12345, error), nullptr);
    EXPECT_THAT(error, ::testing::HasSubstr("12345"));
}

TEST(SerialLine, SaysWhichDeviceIsMissing)
{
    std::string error;
    EXPECT_EQ(SerialPort::open("/dev/no_such_hand", 115200, error), nullptr);
    EXPECT_THAT(error, ::testing::HasSubstr("/dev/no_such_hand"));
}

}  // namespace
