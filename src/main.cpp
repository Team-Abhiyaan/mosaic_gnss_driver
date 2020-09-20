#include <ros/package.h>
#include <ros/ros.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <mosaic_gnss_driver/parsers/sbf/sbf.h>
#include <mosaic_gnss_driver/connections/pcap.h>
#include <mosaic_gnss_driver/connections/tcp.h>
#include <mosaic_gnss_driver/connections/udp.h>
#include <mosaic_gnss_driver/connections/serial.h>

template<typename conn_type>
void start(const std::string &device) {
    mosaic_gnss_driver::GNSS<conn_type, sbf::SBF> gnss{};
    if (!gnss.connect(device)) return;
    while (gnss.tick());
    // gnss.disconnect(); // Destructors automatically disconnect
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sample_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    std::string device;
    if (!nh.getParam("device", device))
        device = "";

    std::string type;
    nh.getParam("conn", type);

    if (type.empty()) {
        ROS_FATAL("No connection type set.");
        return 1;
    } else if (type == "pcap") {
        if (device.empty()) device = ros::package::getPath("mosaic_gnss_driver") + "/test/data/sbf/capture_001.pcap";
        start<mosaic_gnss_driver::connections::PCAP>(device);
    } else if (type == "serial") {
        if (device.empty()) device = "/dev/ACM0";
        start<mosaic_gnss_driver::connections::Serial>(device);
    } else if (type == "tcp") {
        if (device.empty()) device = "192.168.1.101";
        start<mosaic_gnss_driver::connections::TCP>(device);
    } else if (type == "udp") {
        if (device.empty()) device = "192.168.1.101";
        start<mosaic_gnss_driver::connections::UDP>(device);
    }

    return 0;
}
