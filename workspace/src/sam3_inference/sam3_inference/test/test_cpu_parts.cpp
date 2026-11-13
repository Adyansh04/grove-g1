#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "sam3_inference/request_checks.hpp"
#include "sam3_inference/tensor_types.hpp"

namespace sam3_inference
{
namespace
{

std::uint16_t bf16Of(float value)
{
    return static_cast<std::uint16_t>(std::bit_cast<std::uint32_t>(value) >> 16);
}

class FloatConversion : public ::testing::TestWithParam<float>
{
};

// Values bf16 holds exactly: they survive both conversions bit for bit.
TEST_P(FloatConversion, Float32AndBfloat16KeepTheValue)
{
    const float value = GetParam();
    float       out   = 0.0F;
    toFloats(&value, ElementType::kFloat32, std::span<float>(&out, 1));
    const std::uint16_t half      = bf16Of(value);
    float               from_bf16 = 0.0F;
    toFloats(&half, ElementType::kBfloat16, std::span<float>(&from_bf16, 1));
    if (std::isnan(value))
    {
        EXPECT_TRUE(std::isnan(out));
        EXPECT_TRUE(std::isnan(from_bf16));
        return;
    }
    // Bit for bit, so -0 and 0 differ.
    EXPECT_EQ(std::bit_cast<std::uint32_t>(out), std::bit_cast<std::uint32_t>(value));
    EXPECT_EQ(std::bit_cast<std::uint32_t>(from_bf16), std::bit_cast<std::uint32_t>(value));
}

INSTANTIATE_TEST_SUITE_P(
    Values, FloatConversion,
    ::testing::Values(
        0.0F, -0.0F, 1.0F, -2.5F,
        // The smallest positive subnormal bf16 holds: 2^-133.
        std::ldexp(1.0F, -133), std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN(),
        // The largest finite bf16: (2 - 2^-7) * 2^127.
        std::ldexp(2.0F - std::ldexp(1.0F, -7), 127)));

TEST(TensorTypes, ConvertsWholeBuffersAndRefusesNonFloatTypes)
{
    const std::vector<std::uint16_t> halves{ bf16Of(1.0F), bf16Of(-3.0F), bf16Of(0.5F) };
    std::vector<float>               out(3);
    toFloats(halves.data(), ElementType::kBfloat16, out);
    EXPECT_EQ(out, (std::vector<float>{ 1.0F, -3.0F, 0.5F }));
    const std::array<std::int32_t, 1> ints{ 7 };
    EXPECT_THROW(toFloats(ints.data(), ElementType::kInt32, out), std::invalid_argument);
    EXPECT_EQ(elementSize(ElementType::kFloat32), 4U);
    EXPECT_EQ(elementSize(ElementType::kBfloat16), 2U);
    EXPECT_EQ(elementSize(ElementType::kInt64), 8U);
    EXPECT_EQ(elementSize(ElementType::kBool), 1U);
}

TEST(RequestChecks, RefusesNoPromptsAndBlankOnes)
{
    EXPECT_TRUE(checkPrompts(std::vector<std::string>{}).has_value());
    EXPECT_TRUE(checkPrompts(std::vector<std::string>{ "red cube", "" }).has_value());
    EXPECT_TRUE(checkPrompts(std::vector<std::string>{ "  \t" }).has_value());
    EXPECT_FALSE(checkPrompts(std::vector<std::string>{ "red cube", " t connector " }).has_value());
}

TEST(RequestChecks, ImageAgeFromTheStamp)
{
    EXPECT_FALSE(imageAgeS(-1, 0, 10.0).has_value());
    // NaN, the value_or default, fails both checks.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_NEAR(imageAgeS(8, 500000000U, 10.0).value_or(nan), 1.5, 1e-9);
    EXPECT_NEAR(imageAgeS(11, 0, 10.0).value_or(nan), -1.0, 1e-9);  // a stamp in the future
}

}  // namespace
}  // namespace sam3_inference
