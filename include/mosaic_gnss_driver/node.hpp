#ifndef __MOSAIC_GNSS_DRIVER_NODE_H__
#define __MOSAIC_GNSS_DRIVER_NODE_H__

#include <ros/ros.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>
#include <mosaic_gnss_driver/data_buffers.h>

#include <mosaic_gnss_driver/connections/pcap.h>
#include <mosaic_gnss_driver/connections/tcp.h>
#include <mosaic_gnss_driver/connections/udp.h>
#include <mosaic_gnss_driver/connections/serial.h>

#include <mosaic_gnss_driver/parsers/nmeaparse/NMEAParser.h>
#include <mosaic_gnss_driver/parsers/sbf/sbf.h>

template <typename c_Tp, typename p_Tp>
class MosaicGNSSDriverNode
{
public:
    MosaicGNSSDriverNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : m_nh(nh), m_pnh(pnh),
          m_sbf_pvt_type("geodetic"),
          m_publish_navsatfix(false),
          m_publish_velocity(false),
          m_publish_pose(false),
          m_publish_nmea_sentence(false),
          m_publish_time_reference(false)
    {
        _initParams();
        _initPublishers();
    }

    bool registerDevice(const std::string &device)
    {
        return m_gnss.connect(device);
    }

    void spin()
    {
        auto start_time = ros::Time::now();
        const auto publish_duration = ros::Duration(0.2);
        while (ros::ok() && m_gnss.tick())
        {

#ifdef MOSAIC_GNSS_FAKE_SLEEP_TIME
            ros::Duration(MOSAIC_GNSS_FAKE_SLEEP_TIME).sleep();
#endif

            const auto cur = ros::Time::now();
            if (cur - start_time > publish_duration)
            { // We should publish
                start_time = cur;

                // Publish fields
                m_dBuff.publish_all();
            }

            ros::spinOnce();
        }
    }

private:
    void _initParams()
    {
        m_pnh.getParam("sbf_pvt_type", m_sbf_pvt_type);
        m_pnh.getParam("publish/navsatfix", m_publish_navsatfix);
        m_pnh.getParam("publish/velocity", m_publish_velocity);
        m_pnh.getParam("publish/pose", m_publish_pose);
        m_pnh.getParam("publish/nmea_sentence", m_publish_nmea_sentence);
        m_pnh.getParam("publish/time_reference", m_publish_time_reference);
    }

    void _initPublishers()
    {

        if constexpr (std::is_same<p_Tp, sbf::SBF>::value)
        {
            if (m_sbf_pvt_type == "cartesian")
            {
                m_gnss.p.parsers.enable_cartesian();
                if (!(m_publish_pose || m_publish_velocity))
                {
                    // If no output is enabled, enable all the outputs
                    m_publish_pose = true;
                    m_publish_velocity = true;
                }
            }
            else
            {
                if (!m_sbf_pvt_type.empty() && m_sbf_pvt_type != "geodetic")
                    ROS_WARN("Invalid pvt type, assuming geodetic.");

                m_gnss.p.parsers.enable_geodetic();
                if (!(m_publish_navsatfix || m_publish_velocity))
                {
                    // If no output is enabled, enable all the outputs
                    m_publish_navsatfix = true;
                    m_publish_velocity = true;
                }
            }
        }

        if (m_publish_navsatfix)
        {
            m_dBuff.nav_sat_fix.init(m_nh, "navsatfix", 5, false);
        }
        if (m_publish_velocity)
        {
            m_dBuff.velocity.init(m_nh, "velocity", 5, false);
        }
        if (m_publish_pose)
        {
            m_dBuff.pose.init(m_nh, "pose", 5, false);
        }
        if (m_publish_nmea_sentence)
        {
            m_dBuff.nmea_sentence.init(m_nh, "nmea_sentence", 5, false);
        }
        if (m_publish_time_reference)
        {
            m_dBuff.time_reference.init(m_nh, "time_reference", 5, false);
        }
    }

    ros::NodeHandle m_nh, m_pnh;

    mosaic_gnss_driver::DataBuffers m_dBuff;

    mosaic_gnss_driver::GNSS<c_Tp, p_Tp> m_gnss{m_dBuff};

    std::string m_sbf_pvt_type;
    bool m_publish_navsatfix;
    bool m_publish_velocity;
    bool m_publish_pose;
    bool m_publish_nmea_sentence;
    bool m_publish_time_reference;
};

#endif