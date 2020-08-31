#include <netinet/ip.h>
#include <net/ethernet.h>
#include <iostream>

#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <ros/ros.h>

namespace mosaic_gnss_driver
{
    MosaicGNSS::MosaicGNSS() : m_cConnectionType(SERIAL),
                               m_bIsConnected(false),
                               m_pPcap(nullptr)
    {
    }

    MosaicGNSS::~MosaicGNSS()
    {
        disconnect();
    }

    MosaicGNSS::ConnectionType MosaicGNSS::parseConnection(const std::string &connection)
    {
        if (connection == "serial")
        {
            return SERIAL;
        }
        else if (connection == "udp")
        {
            return UDP;
        }
        else if (connection == "tcp")
        {
            return TCP;
        }
        else if (connection == "pcap")
        {
            return PCAP;
        }

        return INVALID;
    }

    void MosaicGNSS::disconnect()
    {
        // todo : add serial, tcp, udp disconnections and socket closing

        if (m_cConnectionType == PCAP)
        {
            if (m_pPcap != nullptr)
            {
                pcap_close(m_pPcap);
                m_pPcap = nullptr;
            }
        }

        m_bIsConnected = false;
    }

    bool MosaicGNSS::connect(const std::string &device, ConnectionType connection)
    {
        MosaicGNSSMessageOpts opts;

        // TODO : Set default options here

        return connect(device, connection, opts);
    }

    bool MosaicGNSS::connect(const std::string &device, ConnectionType connection, MosaicGNSSMessageOpts const &opts)
    {
        disconnect();

        // todo: add serial, udp, tcp connections

        m_cConnectionType = connection;

        if (connection == PCAP)
        {
            return _createPcapConnection(device, opts);
        }
    }

    bool MosaicGNSS::_createPcapConnection(const std::string &device, MosaicGNSSMessageOpts const &opts)
    {
        ROS_INFO("Opening pcap file: %s", device.c_str());

        if ((m_pPcap = pcap_open_offline(device.c_str(), m_cPcapErrBuffer)) == nullptr)
        {
            ROS_FATAL("Unable to open pcap file");
            m_bIsConnected = false;
            return false;
        }

        pcap_compile(m_pPcap, &m_PcapPacketFilter, "tcp dst port 3001", 1, PCAP_NETMASK_UNKNOWN);
        m_bIsConnected = true;

        return true;
    }

    MosaicGNSS::ReadResult MosaicGNSS::processData()
    {
        // todo complete this method
        MosaicGNSS::ReadResult readResult = _readData();

        if (readResult != READ_SUCCESS)
        {
            return readResult;
        }

        return READ_SUCCESS;
    }

    bool MosaicGNSS::_createSerialConnection() {}

    bool MosaicGNSS::_createIpConnection() {}

    MosaicGNSS::ReadResult MosaicGNSS::_readData()
    {
        switch (m_cConnectionType)
        {
        case SERIAL:
            return _readSerialData();
        case TCP:
            return _readIpData();
        case UDP:
            return _readIpData();
        case PCAP:
            return _readPcapData();

        default:
            return READ_ERROR;
        }
    }

    MosaicGNSS::ReadResult MosaicGNSS::_readSerialData() {}

    MosaicGNSS::ReadResult MosaicGNSS::_readIpData() {}

    MosaicGNSS::ReadResult MosaicGNSS::_readPcapData()
    {
        struct pcap_pkthdr *header;
        const u_char *packetData;

        int result;

        // Read next packet
        result = pcap_next_ex(m_pPcap, &header, &packetData);

        if (result >= 0)
        {
            auto ipHeader = reinterpret_cast<const iphdr *>(packetData + sizeof(struct ethhdr));
            uint32_t ipHeaderLength = ipHeader->ihl * 4u;

            switch (ipHeader->protocol)
            {
                case 6: // TCP
                {
                    
                    if (header->len == 54)
                    {
                        // Empty packet, skip it.
                        return READ_SUCCESS;
                    }

                    bool storePacket = true;

                    if (!m_vLastTcpPacket.empty())
                    {
                        auto tcpHeader = reinterpret_cast<const tcphdr *>(packetData + ipHeaderLength + sizeof(struct ethhdr));
                        auto lastIpHeader = reinterpret_cast<const iphdr *>(&(m_vLastTcpPacket[0]));
                        uint32_t lastIpHeaderLength = lastIpHeader->ihl * 4u;
                        auto lastTcpHeader = reinterpret_cast<const tcphdr *>(&(m_vLastTcpPacket[0]) + lastIpHeaderLength);
                        uint16_t lastLength = ntohs(static_cast<uint16_t>(lastIpHeader->tot_len));
                        uint16_t newLength = ntohs(static_cast<uint16_t>(ipHeader->tot_len));
                        uint32_t lastSeq = ntohl(lastTcpHeader->seq);
                        uint32_t newSeq = ntohl(tcpHeader->seq);

                        if (newSeq != lastSeq)
                        {
                            uint32_t dataOffset = lastTcpHeader->doff * 4;
                            m_vDataBuffer.insert(m_vDataBuffer.end(), m_vLastTcpPacket.begin() + lastIpHeaderLength + dataOffset, m_vLastTcpPacket.end());
                        }
                        else if (newLength <= lastLength)
                        {
                            storePacket = false;
                        }
                    }

                    if (storePacket)
                    {
                        m_vLastTcpPacket.clear();
                        m_vLastTcpPacket.insert(m_vLastTcpPacket.end(), packetData + sizeof(struct ethhdr), packetData + header->len);
                    }

                    break;
                }
                case 17: // UDP
                {
                    // todo: add udp protocol handling
                    break;
                }
                case 128: // SSCOPMCE
                {
                    // todo : research and handle
                    break;
                }
                default:
                    ROS_WARN("Unexpected protocol: %u", ipHeader->protocol);
                    return READ_ERROR;
            }

            ros::Duration(0.0001).sleep();

            return READ_SUCCESS;
        }
        else if (result == -2)
        {
            ROS_INFO("Done reading pcap file.");

            if (!m_vLastTcpPacket.empty())
            {
                auto lastIpHeader = reinterpret_cast<const iphdr*>(&(m_vLastTcpPacket[0]));
                uint32_t ipHeaderLength = lastIpHeader->ihl * 4u;

                auto lastTcpHeader = reinterpret_cast<const tcphdr*>(&(m_vLastTcpPacket[0]) + ipHeaderLength);
                uint32_t dataOffset = lastTcpHeader->doff * 4u;

                m_vDataBuffer.insert(m_vDataBuffer.end(),
                    m_vLastTcpPacket.begin() + ipHeaderLength + dataOffset,
                    m_vLastTcpPacket.end()
                );

                m_vLastTcpPacket.clear();
            }
            disconnect();
            return READ_SUCCESS;
        }
        else
        {
            ROS_WARN("Error reading pcap data: %s", pcap_geterr(m_pPcap));
            return READ_ERROR;
        }        
    }
} // namespace mosaic_gnss_driver