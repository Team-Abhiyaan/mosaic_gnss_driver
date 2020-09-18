#include <mosaic_gnss_driver/mosaic_gnss.h>
#include <mosaic_gnss_driver/connections/pcap.h>
#include <mosaic_gnss_driver/connections/tcp.h>
#include <mosaic_gnss_driver/connections/udp.h>
#include <mosaic_gnss_driver/connections/serial.h>
#include <mosaic_gnss_driver/parsers/sbf/sbf.h>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>

// #define RUN_SERIAL_TEST
// #define RUN_TCP_TEST
// #define RUN_UDP_TEST
#define RUN_PCAP_TEST

#if defined RUN_SERIAL_TEST
TEST(TcpTestSuite, testCaseTcpConnection)
{
	std::string fileHandle = "/dev/ttyUSB0";

    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::Serial, sbf::SBF> gnss;

    ASSERT_FALSE(gnss.is_connected());

    ASSERT_TRUE(gnss.connect(fileHandle));

    while (gnss.is_connected() && gnss.tick())
        ;

    gnss.disconnect();

    ASSERT_FALSE(gnss.is_connected());
}
#endif 

#if defined RUN_TCP_TEST
TEST(TcpTestSuite, testCaseTcpConnection)
{
	std::string endpoint = "192.168.3.1:9999";

    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::TCP, sbf::SBF> gnss;

    ASSERT_FALSE(gnss.is_connected());

    ASSERT_TRUE(gnss.connect(endpoint));

    while (gnss.is_connected() && gnss.tick())
        ;

    gnss.disconnect();

    ASSERT_FALSE(gnss.is_connected());
}
#endif 

#if defined RUN_UDP_TEST
TEST(UdpTestSuite, testCaseTcpConnection)
{
	std::string endpoint = "192.168.3.1:9999";

    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::UDP, sbf::SBF> gnss;

    ASSERT_FALSE(gnss.is_connected());

    ASSERT_TRUE(gnss.connect(endpoint));

    while (gnss.is_connected() && gnss.tick())
        ;

    gnss.disconnect();

    ASSERT_FALSE(gnss.is_connected());
}
#endif 

#if defined RUN_PCAP_TEST
TEST(PcapTestSuite, testCasePcapFileConnection)
{
    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF> gnss;
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");

    ASSERT_FALSE(gnss.is_connected());

    ASSERT_TRUE(gnss.connect(thisPackagePath + "/test/data/sbf/capture_001.pcap"));

    while (gnss.is_connected() && gnss.tick())
        ;

    gnss.disconnect();

    ASSERT_FALSE(gnss.is_connected());
}
#endif

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcapTestSuite", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
