#include <ros/package.h>
#include <ros/ros.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

void connectViaPcap(void)
{
    mosaic_gnss_driver::MosaicGNSS gnss;

    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");
    std::string testFile = thisPackagePath + "/test/data/mosaic_capture_001.pcap";

    gnss.connect(testFile, mosaic_gnss_driver::MosaicGNSS::PCAP);

    gnss.processData();

    gnss.disconnect();

}

void connectViaTcp(void)
{
    mosaic_gnss_driver::MosaicGNSS gnss;

    std::string device = "192.168.3.1:80";

    gnss.connect(device, mosaic_gnss_driver::MosaicGNSS::TCP);

    gnss.processData();

    gnss.disconnect();

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sample_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    // connectViaPcap();
    connectViaTcp();

    return 0;
}