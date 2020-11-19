#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <mosaic_gnss_driver/data_buffers.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <mosaic_gnss_driver/connections/pcap.h>
#include <mosaic_gnss_driver/connections/tcp.h>
#include <mosaic_gnss_driver/connections/udp.h>
#include <mosaic_gnss_driver/connections/serial.h>

#include <mosaic_gnss_driver/parsers/nmeaparse/NMEAParser.h>
#include <mosaic_gnss_driver/parsers/sbf/sbf.h>

template<typename conn_type, typename parser_type>
void start(const std::string &device)
{
    ros::NodeHandle nh, pnh("~");
    mosaic_gnss_driver::DataBuffers buf;

    mosaic_gnss_driver::GNSS<conn_type, parser_type> gnss{buf};
    if (!gnss.connect(device)) return;

    if constexpr (std::is_same<parser_type, sbf::SBF>::value)
    {
        bool geodetic;
        const auto type = pnh.param("sbf_pvt_type", std::string{"geodetic"});

        if (type == "cartesian")
        {
            geodetic = false;
        } else
        {
            geodetic = true;
            if (type != "geodetic")
                ROS_WARN("Invalid pvt type, assuming geodetic.");
        }

        buf.velocity.init(nh, "velocity", 5, false);

        if (geodetic)
        {
            buf.nav_sat_fix.init(nh, "nav_sat_fix", 5, false);
            gnss.p.parsers.enable_geodetic();

        } else
        {
            buf.pose.init(nh, "pose", 5, false);
            gnss.p.parsers.enable_cartesian();

        }


    } else if constexpr (std::is_same<parser_type, nmea::NMEAParser>::value)
    {
        buf.nav_sat_fix.init(nh, "nav_sat_fix", 5, false);
        buf.pose.init(nh, "pose", 5, false);
        buf.velocity.init(nh, "velocity", 5, false);
        buf.time_reference.init(nh, "time_reference", 5, false);

        bool pub_nmea_msg;
        ros::param::get("/mosaic_gnss/pub_nmea_msg", pub_nmea_msg);
        if (pub_nmea_msg)
        {
            buf.nmea_sentence.init(nh, "nmea_sentence", 5, false);
        }
    }

    auto start_time = ros::Time::now();

    const auto publish_duration = ros::Duration(0.2);

    while (ros::ok() && gnss.tick())
    {

#ifdef MOSAIC_GNSS_FAKE_SLEEP_TIME
        ros::Duration(MOSAIC_GNSS_FAKE_SLEEP_TIME).sleep();
#endif

        const auto cur = ros::Time::now();
        if (cur - start_time > publish_duration)
        { // We should publish
            start_time = cur;

            // Publish fields
            buf.publish_all();
        }

        ros::spinOnce();
    }
}

template<typename parser_type>
void start(std::string &device, const std::string &type)
{
    if (type.empty())
    {
        ROS_FATAL("No connection type set.");
    } else if (type == "pcap")
    {
        if (device.empty()) device = ros::package::getPath("mosaic_gnss_driver") + "/test/data/nmea/capture_004.pcap";
        start<mosaic_gnss_driver::connections::PCAP, parser_type>(device);
    } else if (type == "serial")
    {
        if (device.empty()) device = "/dev/ACM0";
        start<mosaic_gnss_driver::connections::Serial, parser_type>(device);
    } else if (type == "tcp")
    {
        if (device.empty()) device = "192.168.1.101";
        start<mosaic_gnss_driver::connections::TCP, parser_type>(device);
    } else if (type == "udp")
    {
        if (device.empty()) device = "192.168.1.101";
        start<mosaic_gnss_driver::connections::UDP, parser_type>(device);
    } else
    {
        ROS_FATAL("Invalid connection type set.");
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mosaic_gnss"); //, ros::init_options::AnonymousName);
    ros::NodeHandle nh, pnh("~");

    std::string device, parser, type, gps_link;
    bool pub_nmea_msg;

    if (!pnh.getParam("device", device))
        device = "";
    if (!pnh.getParam("parser", parser))
        parser = "";
    if (!pnh.getParam("conn", type))
        type = "";

    if (!pnh.getParam("pub_nmea_msg", pub_nmea_msg))
        pnh.setParam("pub_nmea_msg", 0);
    if (!pnh.hasParam("frame_id"))
        pnh.setParam("frame_id", "gps_link");

    if (parser.empty())
    {
        ROS_FATAL("No parser type set.");
    } else if (parser == "sbf")
    {
        start<sbf::SBF>(device, type);
    } else if (parser == "nmea")
    {
        start<nmea::NMEAParser>(device, type);
    } else
    {
        ROS_FATAL("Invalid parser type set.");
    }


    return 0;
}


