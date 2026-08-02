/**
 * @file test_loco_payloads.cpp
 * @brief Unit tests for the JSON payload builders/parser -- exact wire format, no 864000, and
 * malformed-input handling.
 */
#include <gmock/gmock.h>

#include "g1_locomotion/loco_payloads.hpp"

namespace g1_locomotion
{
namespace
{

// -------------------------------------------------------------------------
// buildSetFsmIdPayload() -- 7101
// -------------------------------------------------------------------------

TEST(BuildSetFsmIdPayload, ExactJsonForStart)
{
    EXPECT_EQ(buildSetFsmIdPayload(500), R"({"data":500})");
}

TEST(BuildSetFsmIdPayload, ExactJsonForDamp)
{
    EXPECT_EQ(buildSetFsmIdPayload(1), R"({"data":1})");
}

// -------------------------------------------------------------------------
// buildSetVelocityPayload() -- 7105
// -------------------------------------------------------------------------

TEST(BuildSetVelocityPayload, ExactJsonForCleanValues)
{
    // nlohmann::json orders object keys alphabetically regardless of insertion order, so
    // "duration" sorts before "velocity" -- this is exactly what BaseClient's own
    // js["velocity"] = ...; js["duration"] = ...; produces too (same library, same ordering).
    EXPECT_EQ(
        buildSetVelocityPayload(1.0F, 0.5F, -0.5F),
        R"({"duration":1.0,"velocity":[1.0,0.5,-0.5]})");
}

TEST(BuildSetVelocityPayload, ExactJsonForZero)
{
    EXPECT_EQ(
        buildSetVelocityPayload(0.0F, 0.0F, 0.0F),
        R"({"duration":1.0,"velocity":[0.0,0.0,0.0]})");
}

TEST(BuildSetVelocityPayload, DurationIsAlwaysKVelocityDurationS)
{
    for (const float vx : { -0.3F, 0.0F, 0.3F, 100.0F })
    {
        const auto payload = buildSetVelocityPayload(vx, 0.0F, 0.0F);
        EXPECT_THAT(payload, ::testing::HasSubstr(R"("duration":1.0)"))
            << "payload was: " << payload;
        // The vendor's "continuous move" sentinel must never appear anywhere this builder can
        // produce -- there is no parameter to pass it through in the first place.
        EXPECT_THAT(payload, ::testing::Not(::testing::HasSubstr("864000")));
    }
}

TEST(KVelocityDurationS, IsExactlyOneSecond) { EXPECT_FLOAT_EQ(kVelocityDurationS, 1.0F); }

// -------------------------------------------------------------------------
// parseFsmIdResponse() -- 7001's response data, including malformed input
// -------------------------------------------------------------------------

TEST(ParseFsmIdResponse, ParsesValidData)
{
    const auto fsm_id = parseFsmIdResponse(R"({"data":4})");
    ASSERT_TRUE(fsm_id.has_value());
    EXPECT_EQ(*fsm_id, 4);
}

TEST(ParseFsmIdResponse, ParsesStartValue)
{
    const auto fsm_id = parseFsmIdResponse(R"({"data":500})");
    ASSERT_TRUE(fsm_id.has_value());
    EXPECT_EQ(*fsm_id, 500);
}

TEST(ParseFsmIdResponse, EmptyStringIsMalformed)
{
    EXPECT_FALSE(parseFsmIdResponse("").has_value());
}

TEST(ParseFsmIdResponse, NotJsonIsMalformed)
{
    EXPECT_FALSE(parseFsmIdResponse("not json").has_value());
}

TEST(ParseFsmIdResponse, MissingDataFieldIsMalformed)
{
    EXPECT_FALSE(parseFsmIdResponse("{}").has_value());
}

TEST(ParseFsmIdResponse, NonIntegerDataFieldIsMalformed)
{
    EXPECT_FALSE(parseFsmIdResponse(R"({"data":"start"})").has_value());
    EXPECT_FALSE(parseFsmIdResponse(R"({"data":4.5})").has_value());
}

TEST(ParseFsmIdResponse, NonObjectTopLevelIsMalformed)
{
    EXPECT_FALSE(parseFsmIdResponse("[1,2,3]").has_value());
}

}  // namespace
}  // namespace g1_locomotion
