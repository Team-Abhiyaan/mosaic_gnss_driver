#include <mosaic_gnss_driver/mosaic_gnss.h>
#include <mosaic_gnss_driver/connections/pcap.h>
#include <mosaic_gnss_driver/parsers/sbf/sbf.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>

TEST(PcapTestSuite, testCasePcapFileConnection)
{
    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF> gnss;
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");

    ASSERT_FALSE(gnss.is_connected());

    ASSERT_TRUE(gnss.conn.connect(thisPackagePath + "/test/data/sbf/capture_001.pcap"));

    while (gnss.is_connected() && gnss.tick())
        ;

    gnss.conn.disconnect();

    ASSERT_FALSE(gnss.is_connected());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcapTestSuite", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
