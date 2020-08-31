#include <ros/package.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

int main(int argc, char **argv)
{
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");
    std::string testFile = thisPackagePath + "/test/data/mosaic_capture_001.pcap";

    mosaic_gnss_driver::MosaicGNSS gnss;

    gnss.connect(testFile, mosaic_gnss_driver::MosaicGNSS::PCAP);

    gnss.processData();

    gnss.disconnect();

    return 0;
}