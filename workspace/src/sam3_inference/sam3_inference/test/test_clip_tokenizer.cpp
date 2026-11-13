#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "sam3_inference/clip_tokenizer.hpp"

namespace sam3_inference
{
namespace
{

// Ids SAM 3's own Python tokenizer gives these prompts (context 32; zeros after the end left out).
struct Case
{
    std::string               prompt;
    std::vector<std::int64_t> ids;
};

const std::vector<Case>& cases()
{
    static const std::vector<Case> kCases{
        { "cup", { 49406, 1937, 49407 } },
        { "red cube", { 49406, 736, 11353, 49407 } },
        { "t connector", { 49406, 339, 8437, 8108, 49407 } },
        { "T-Connector", { 49406, 339, 268, 8437, 8108, 49407 } },
        { "blue cube", { 49406, 1746, 11353, 49407 } },
        { "battery module", { 49406, 7870, 16757, 49407 } },
        { "a green cube on the table", { 49406, 320, 1901, 11353, 525, 518, 2175, 49407 } },
        { "part_mrx", { 49406, 1551, 318, 3139, 343, 49407 } },
        { "the robot's gripper", { 49406, 518, 8797, 568, 2169, 2440, 49407 } },
        { "cube 3", { 49406, 11353, 274, 49407 } },
        { "  Yellow   CUBE  ", { 49406, 4481, 11353, 49407 } },
        { "connector plate", { 49406, 8437, 8108, 5135, 49407 } },
    };
    return kCases;
}

TEST(ClipTokenizer, MatchesSam3sPythonTokenizer)
{
    const ClipTokenizer tokenizer(SAM3_VOCABULARY);
    for (const Case& c : cases())
    {
        std::vector<std::int64_t> expected = c.ids;
        expected.resize(32, 0);
        EXPECT_EQ(tokenizer.encode(c.prompt), expected) << c.prompt;
    }
}

TEST(ClipTokenizer, LongTextIsCutAndStillEnds)
{
    const ClipTokenizer tokenizer(SAM3_VOCABULARY);
    std::string         text;
    for (int i = 0; i < 40; ++i)
    {
        text += "cube ";
    }
    const auto ids = tokenizer.encode(text);
    ASSERT_EQ(ids.size(), 32U);
    EXPECT_EQ(ids.front(), ClipTokenizer::kStartOfText);
    EXPECT_EQ(ids.back(), ClipTokenizer::kEndOfText);
    EXPECT_EQ(ids[1], 11353);
}

TEST(ClipTokenizer, EmptyTextIsStartAndEnd)
{
    const ClipTokenizer tokenizer(SAM3_VOCABULARY);
    const auto          ids = tokenizer.encode("   ");
    EXPECT_EQ(ids[0], ClipTokenizer::kStartOfText);
    EXPECT_EQ(ids[1], ClipTokenizer::kEndOfText);
    EXPECT_EQ(ids[2], 0);
}

TEST(ClipTokenizer, ContextLengthsBelowTwoAreRefused)
{
    const ClipTokenizer tokenizer(SAM3_VOCABULARY);
    EXPECT_THROW(static_cast<void>(tokenizer.encode("cube", 0)), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(tokenizer.encode("cube", 1)), std::invalid_argument);
    const auto ids = tokenizer.encode("red cube", 2);
    EXPECT_EQ(
        ids,
        (std::vector<std::int64_t>{ ClipTokenizer::kStartOfText, ClipTokenizer::kEndOfText }));
}

}  // namespace
}  // namespace sam3_inference
