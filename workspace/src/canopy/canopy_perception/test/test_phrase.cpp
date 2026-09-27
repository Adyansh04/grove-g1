/**
 * @file test_phrase.cpp
 * @brief A detector phrase becomes the id it is published under.
 */

#include <gmock/gmock.h>

#include "canopy_perception/phrase.hpp"

namespace
{

using canopy_perception::slugify;

TEST(Slugify, MakesAPhraseIntoAnObjectId)
{
    EXPECT_EQ(slugify("Red Block"), "red_block");
    EXPECT_EQ(slugify("red  block!"), "red_block");
    EXPECT_EQ(slugify("  "), "");
}

}  // namespace
