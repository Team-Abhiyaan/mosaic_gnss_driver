#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <gtest/gtest.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

TEST(PcapTestSuite, testCasePcapFileConnection)
{
    mosaic_gnss_driver::MosaicGNSS gnss;
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");

    ASSERT_FALSE(gnss.isConnected());

    ASSERT_TRUE(gnss.connect(thisPackagePath + "/test/data/mosaic_capture_001.pcap", mosaic_gnss_driver::MosaicGNSS::PCAP));

    while(gnss.isConnected() && gnss.processData() == mosaic_gnss_driver::MosaicGNSS::READ_SUCCESS)
    {
        ;
    }

    gnss.disconnect();

    ASSERT_FALSE(gnss.isConnected());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcapTestSuite", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}