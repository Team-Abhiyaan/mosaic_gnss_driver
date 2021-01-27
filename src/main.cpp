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

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

template <typename c_Tp, typename p_Tp>
struct driverImpl
{
    static void run(const std::string &device) {}
};

template <typename conn_type, typename parser_type>
void driver(const std::string &device)
{
    driverImpl<conn_type, parser_type>::run(device);
}

template <typename conn_type>
struct driverImpl<conn_type, sbf::SBF>
{
    static void run(const std::string &device)
    {
        ros::NodeHandle nh, pnh("~");
        mosaic_gnss_driver::DataBuffers buf;

        mosaic_gnss_driver::GNSS<conn_type, sbf::SBF> gnss{buf};
        if (!gnss.connect(device))
            return;

        bool geodetic;
        const auto type = pnh.param("sbf_pvt_type", std::string{"geodetic"});

        if (type == "cartesian")
        {
            geodetic = false;
        }
        else
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
        }
        else
        {
            buf.pose.init(nh, "pose", 5, false);
            gnss.p.parsers.enable_cartesian();
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
};

template <typename conn_type>
struct driverImpl<conn_type, nmea::NMEAParser>
{
    static void run(const std::string &device)
    {
        ros::NodeHandle nh, pnh("~");
        mosaic_gnss_driver::DataBuffers buf;

        mosaic_gnss_driver::GNSS<conn_type, nmea::NMEAParser> gnss{buf};
        if (!gnss.connect(device))
            return;

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
};

template <typename parser_type>
void start(std::string &device, const std::string &type)
{
    if (type == "pcap")
    {
        // safely join paths
        fs::path dir(ros::package::getPath("mosaic_gnss_driver"));
        fs::path file(device);
        fs::path full_path = dir / file;
        device = full_path.string();

        driver<mosaic_gnss_driver::connections::PCAP, parser_type>(device);
    }
    else if (type == "serial")
    {
        driver<mosaic_gnss_driver::connections::Serial, parser_type>(device);
    }
    else if (type == "tcp")
    {
        driver<mosaic_gnss_driver::connections::TCP, parser_type>(device);
    }
    else if (type == "udp")
    {
        driver<mosaic_gnss_driver::connections::UDP, parser_type>(device);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "mosaic_gnss"); //, ros::init_options::AnonymousName);
    ros::NodeHandle nh, pnh("~");

    std::string device, parser, type, gps_link;
    bool pub_nmea_msg;

    if (!pnh.getParam("device", device))
    {
        ROS_FATAL("No device specified");
        return EXIT_FAILURE;
    }
    if (!pnh.getParam("stream", parser))
    {
        ROS_FATAL("Stream type not specified");
        return EXIT_FAILURE;
    }
    if (!pnh.getParam("connection", type))
    {
        ROS_FATAL("Connection type not specified");
        return EXIT_FAILURE;
    }

    if (!pnh.getParam("pub_nmea_msg", pub_nmea_msg))
        pnh.setParam("pub_nmea_msg", 0);

    if (!pnh.hasParam("frame_id"))
        pnh.setParam("frame_id", "gps_link");

    if (parser == "sbf")
    {
        start<sbf::SBF>(device, type);
    }
    else if (parser == "nmea")
    {
        start<nmea::NMEAParser>(device, type);
    }
    else
    {
        ROS_FATAL("Invalid parser type set.");
    }

    return 0;
}
