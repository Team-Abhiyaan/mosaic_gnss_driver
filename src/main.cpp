#include <mosaic_gnss_driver/node.hpp>
#include <ros/package.h>

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error Could not find system header "<filesystem>" or "<experimental/filesystem>"
#endif

template <typename c_Tp, typename p_Tp>
void launchDriver(const std::string& device)
{
    ros::NodeHandle nh, pnh("~");

    MosaicGNSSDriverNode<c_Tp, p_Tp> node(nh, pnh);
    if (!node.registerDevice(device))
    {
        ROS_FATAL("Unable to register device");
        exit(EXIT_FAILURE);
    }

    node.spin();
}

template <typename p_Tp>
void initDriver(std::string& device, const std::string& conn)
{
    if (conn == "pcap")
    {
        // safely join paths
        fs::path dir = ros::package::getPath("mosaic_gnss_driver");
        fs::path file = device;
        fs::path full_path = dir / file;
        device = full_path.string();

        launchDriver<mosaic_gnss_driver::connections::PCAP, p_Tp>(device);
    } else if (conn == "serial")
    {
        launchDriver<mosaic_gnss_driver::connections::Serial, p_Tp>(device);
    } else if (conn == "tcp")
    {
        launchDriver<mosaic_gnss_driver::connections::TCP, p_Tp>(device);
    } else if (conn == "udp")
    {
        launchDriver<mosaic_gnss_driver::connections::UDP, p_Tp>(device);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mosaic_gnss");
    ros::NodeHandle pnh("~");

    bool deviceRegistered = false;
    std::string device, stream, connection;

    if (!pnh.getParam("device", device))
    {
        ROS_FATAL("No device specified");
        return EXIT_FAILURE;
    }
    if (!pnh.getParam("stream", stream))
    {
        ROS_FATAL("Stream type not specified");
        return EXIT_FAILURE;
    }
    if (!pnh.getParam("connection", connection))
    {
        ROS_FATAL("Connection type not specified");
        return EXIT_FAILURE;
    }

    if (stream == "sbf")
    {
        initDriver<sbf::SBF>(device, connection);
    } else if (stream == "nmea")
    {
        initDriver<nmea::NMEAParser>(device, connection);
    } else
    {
        ROS_FATAL("Invalid parser type set.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}