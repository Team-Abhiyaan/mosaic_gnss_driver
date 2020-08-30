#include <gtest/gtest.h>
#include <ros/ros.h>
#include <mosaic_gnss_driver/mosaic_gnss.h>

TEST(PcapTestSuite, testCaseInit)
{
    mosaic_gnss_driver::MosaicGNSS gnss;

    ASSERT_FALSE(gnss.isConnected());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcapTestSuite", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}