#include <gtest/gtest.h>
#include <ros/ros.h>
#include <mosaic_gnss_driver/countOnes.h>

TEST(TestSuite, testCaseSample)
{
    CountOnes c(99);
    EXPECT_EQ(c.getCount(), 4);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sample_test_suite", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}