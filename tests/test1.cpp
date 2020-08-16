#include <gtest/gtest.h>
#include <mosaic_gnss_driver/countOnes.h>

struct Test1 : public ::testing::Test
{
};

TEST(Test1, CheckOutput)
{
    CountOnes c(99);
    EXPECT_EQ(c.getCount(), 4);
}
