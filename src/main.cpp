#include <ros/ros.h>
#include <ros/package.h>

#include <mosaic_gnss_driver/data_buffers.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <mosaic_gnss_driver/connections/pcap.h>
#include <mosaic_gnss_driver/connections/tcp.h>
#include <mosaic_gnss_driver/connections/udp.h>
#include <mosaic_gnss_driver/connections/serial.h>

#include <mosaic_gnss_driver/parsers/nmeaparse/NMEAParser.h>
#include <mosaic_gnss_driver/parsers/sbf/sbf.h>

template<typename conn_type, typename parser_type>
void start(const std::string &device) {
    ros::NodeHandle nh;
    mosaic_gnss_driver::DataBuffers buf;

    buf.nav_sat_fix.pub = nh.advertise<sensor_msgs::NavSatFix>("nav_sat_fix", 5, false);
    // buf.nav_sat_fix.pub = nh.advertise<decltype(buf.nav_sat_fix.ptr.get())>("nav_sat_fix", 5, false);

    mosaic_gnss_driver::GNSS<conn_type, parser_type> gnss{buf};
    if (!gnss.connect(device)) return;

    auto start_time = ros::Time::now();
    const auto duration = ros::Duration(0.2);

    while (gnss.tick()) {
        ros::Duration(0.1).sleep(); // Dummy

        const auto cur = ros::Time::now();
        if (cur - start_time > duration) { // We should publish
            start_time = cur;

            // Publish fields
            { // Nav Sat Fix
                auto &field = buf.nav_sat_fix;
                if (!field.ptr) {
                    ROS_WARN("Not enough msg");
                } else {
                    field.pub.publish(field.ptr);
                    field.ptr.reset();
                }
            }

        }

        ros::spinOnce();
    }
    // gnss.disconnect(); // Destructors automatically disconnect
}

template<typename parser_type>
void start(std::string &device, const std::string &type) {
    if (type.empty()) {
        ROS_FATAL("No connection type set.");
    } else if (type == "pcap") {
        if (device.empty()) device = ros::package::getPath("mosaic_gnss_driver") + "/test/data/sbf/capture_001.pcap";
        start<mosaic_gnss_driver::connections::PCAP, parser_type>(device);
    } else if (type == "serial") {
        if (device.empty()) device = "/dev/ACM0";
        start<mosaic_gnss_driver::connections::Serial, parser_type>(device);
    } else if (type == "tcp") {
        if (device.empty()) device = "192.168.1.101";
        start<mosaic_gnss_driver::connections::TCP, parser_type>(device);
    } else if (type == "udp") {
        if (device.empty()) device = "192.168.1.101";
        start<mosaic_gnss_driver::connections::UDP, parser_type>(device);
    } else {
        ROS_FATAL("Invalid connection type set.");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mosaic_gnss"); //, ros::init_options::AnonymousName);
    ros::NodeHandle nh, pnh("~");

    std::string device, parser, type;

    if (!pnh.getParam("device", device))
        device = "";
    if (!pnh.getParam("parser", parser))
        parser = "";
    if (!pnh.getParam("conn", type))
        type = "";

    if (parser.empty()) {
        ROS_FATAL("No parser type set.");
    } else if (parser == "sbf") {
        start<sbf::SBF>(device, type);
    } else if (parser == "nmea") {
        start<nmea::NMEAParser>(device, type);
    } else {
        ROS_FATAL("Invalid parser type set.");
    }


    return 0;
}


