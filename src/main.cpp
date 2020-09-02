#include <ros/package.h>
#include <ros/ros.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

void connectViaPcap(void)
{
    mosaic_gnss_driver::MosaicGNSS gnss;

    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");
    std::string testFile = thisPackagePath + "/test/data/mosaic_real.pcap";

    gnss.connect(testFile, mosaic_gnss_driver::MosaicGNSS::PCAP);

    gnss.processData();

    gnss.disconnect();

}

void connectViaTcp(void)
{
    mosaic_gnss_driver::MosaicGNSS gnss;

    std::string device = "192.168.3.1:3001";

    gnss.connect(device, mosaic_gnss_driver::MosaicGNSS::TCP);

	for (int i = 0; i < 500; i++)
	{
		bool success = gnss.isConnected() && gnss.processData() == mosaic_gnss_driver::MosaicGNSS::READ_SUCCESS;

		if (!success){
			std::cout << "Error, non successful termination" << std::endl;
			break;
		} else {
			std::cout << "Got stream: " << i + 1 << std::endl;
		}
	}

	gnss.bufferDump();
	std::cout << gnss.errorMsg() << std::endl;
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
