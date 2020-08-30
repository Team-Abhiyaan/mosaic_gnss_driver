#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>

TEST(PcapTestSuite, testCaseInit)
{
    mosaic_gnss_driver::MosaicGNSS gnss;

    ASSERT_FALSE(gnss.isConnected());

}

TEST(PcapTestSuite, testCaseFileConnection)
{
    mosaic_gnss_driver::MosaicGNSS gnss;
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");

    ASSERT_TRUE(gnss.connect(thisPackagePath + "/test/data/sample_data.pcap", mosaic_gnss_driver::MosaicGNSS::PCAP));

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