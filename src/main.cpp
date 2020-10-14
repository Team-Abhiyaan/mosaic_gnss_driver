#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <mosaic_gnss_driver/parsers/nmeaparse/NMEAParser.h>
#include <mosaic_gnss_driver/parsers/nmeaparse/GPSService.h>
#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <mosaic_gnss_driver/parsers/sbf/sbf.h>
#include <mosaic_gnss_driver/connections/pcap.h>

void connectViaPcap(const std::string &data_type)
{
    std::string thisPackagePath = ros::package::getPath("mosaic_gnss_driver");
    if(data_type == "sbf"){
        std::string testFile = thisPackagePath + "/test/data/capture_002.pcap";
        mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF> gnss{};
        if (!gnss.connect(testFile)) return;
        while (gnss.tick());
    }
    if(data_type == "nmea"){
        std::string testFile = thisPackagePath + "/test/data/nmea/capture_004.pcap";
        mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, nmea::NMEAParser> gnss{};
        if (!gnss.connect(testFile)) return;
        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<sensor_msgs::NavSatFix>("NavSatFix", 10);
        sensor_msgs::NavSatFix navSatFix;
        while (gnss.nmea_tick(navSatFix)) {
        
            pub.publish(navSatFix);

            ros::spinOnce();
        };
    }

    

    // gnss.disconnect(); // Destructors automatically disconnect

}

// void connectViaTcp()
// {
//     mosaic_gnss_driver::MosaicGNSS gnss;
//
//     std::string device = "192.168.3.1:3001";
//
//     gnss.connect(device, mosaic_gnss_driver::MosaicGNSS::TCP);
//
// 	for (int i = 0; i < 50; i++)
// 	{
// 		bool success = gnss.isConnected() && gnss.processData() == mosaic_gnss_driver::MosaicGNSS::READ_SUCCESS;
//
// 		if (!success)
//         {
// 			std::cout << "Error, non successful termination" << std::endl;
//            	std::cout << gnss.errorMsg() << std::endl;
//
// 			break;
// 		}
//         else
//         {
// 			std::cout << "Got stream: " << i + 1 << std::endl;
// 		}
// 	}
//
// 	gnss.bufferDump();
//     gnss.disconnect();
//
// }

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sample_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    std::string data_type;
    nh.param<std::string>("data_type",data_type,"nmea");

    connectViaPcap(data_type);
    // connectViaTcp();

    return 0;
}
