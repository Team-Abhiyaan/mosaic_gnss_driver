#include <gtest/gtest.h>
#include <ros/package.h>

#include "mosaic_gnss_driver/connections/pcap.h"
#include "mosaic_gnss_driver/mosaic_gnss.h"
#include "mosaic_gnss_driver/parsers/sbf/sbf.h"

TEST(PcapTestSuite, testCasePcapFileConnection)
{
    mosaic_gnss_driver::DataBuffers db;
    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF> gnss(db);
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");

    ASSERT_FALSE(gnss.is_connected());
    ASSERT_TRUE(gnss.connect(thisPackagePath + "/test/data/sbf/001.pcap"));
    ASSERT_TRUE(gnss.is_connected());
    gnss.disconnect();
    ASSERT_FALSE(gnss.is_connected());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
