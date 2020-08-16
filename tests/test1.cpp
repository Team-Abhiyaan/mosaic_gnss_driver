#include <gtest/gtest.h>
#include <mosaic_gnss_driver/countOnes.h>

struct Test1 : public ::testing::Test
{
};

TEST(Test1, CheckOutput)
{
    EXPECT_EQ(countOnes(99), 4);
}