#include "mosaic_gnss_driver/connections/pcap.h"

#include <ros/ros.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <net/ethernet.h>

using namespace mosaic_gnss_driver::connections;

bool PCAP::connect(const std::string &device, const Options &opts)
{
    if (is_connected())
        return true;
    ROS_INFO("Opening pcap file: %s", device.c_str());

    if ((dev = pcap_open_offline(device.c_str(), err_buffer)) == nullptr)
    {
        ROS_FATAL("Unable to open pcap file");
        return false;
    }

    pcap_compile(dev, &packet_filter, "tcp dst port 3001", 1, PCAP_NETMASK_UNKNOWN);
    connected = true;
    return true;
}

PCAP::~PCAP()
{
    PCAP::disconnect();
}

ReadResult PCAP::read()
{
    if (!is_connected())
        return READ_ERROR;
    struct pcap_pkthdr *header;
    const u_char *packetData;

    int result;

    // Read next packet
    result = pcap_next_ex(dev, &header, &packetData);

    if (result >= 0)
    {
        auto ipHeader = reinterpret_cast<const iphdr *>(packetData + sizeof(struct ethhdr));
        uint32_t ipHeaderLength = ipHeader->ihl * 4u;

        // handle by protocol id, refer https://www.iana.org/assignments/protocol-numbers/protocol-numbers.xhtml
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

                if (!last_packet.empty())
                {
                    auto tcpHeader = reinterpret_cast<const tcphdr *>(packetData + ipHeaderLength +
                                                                      sizeof(struct ethhdr));
                    auto lastIpHeader = reinterpret_cast<const iphdr *>(&(last_packet[0]));
                    uint32_t lastIpHeaderLength = lastIpHeader->ihl * 4u;
                    auto lastTcpHeader = reinterpret_cast<const tcphdr *>(&(last_packet[0]) + lastIpHeaderLength);
                    uint16_t lastLength = ntohs(static_cast<uint16_t>(lastIpHeader->tot_len));
                    uint16_t newLength = ntohs(static_cast<uint16_t>(ipHeader->tot_len));
                    uint32_t lastSeq = ntohl(lastTcpHeader->seq);
                    uint32_t newSeq = ntohl(tcpHeader->seq);

                    if (newSeq != lastSeq)
                    {
                        uint32_t dataOffset = lastTcpHeader->doff * 4;
                        buffer.insert(buffer.end(), last_packet.begin() + lastIpHeaderLength + dataOffset,
                                      last_packet.end());
                    } else if (newLength <= lastLength)
                    {
                        storePacket = false;
                    }
                }

                if (storePacket)
                {
                    last_packet.clear();
                    last_packet.insert(last_packet.end(), packetData + sizeof(struct ethhdr), packetData + header->len);
                }

                break;
            }
            case 17: // UDP
            {
                uint16_t fragOff = ntohs(static_cast<uint16_t>(ipHeader->frag_off));

                uint16_t fragmentOffset = fragOff & static_cast<uint16_t>(0x1FFF);
                size_t headerSize;

                // UDP packets may be fragmented; this isn't really "correct", but for
                // simplicity's sake we'll assume we get fragments in the right order.
                if (fragmentOffset == 0)
                {
                    headerSize = sizeof(struct ethhdr) + ipHeaderLength + sizeof(struct udphdr);
                } else
                {
                    headerSize = sizeof(struct ethhdr) + ipHeaderLength;
                }

                buffer.insert(buffer.end(), packetData + headerSize, packetData + header->len);

                break;
            }
            case 128: // SSCOPMCE
            {
                ROS_WARN("Recieved data via SSCOPMCE protocol");
                // TODO : research and handle
                // Got this protocol while testing with the pcap file, no clue what this does
                break;
            }
            default:
                ROS_WARN("Unexpected protocol: %u", ipHeader->protocol);
                return READ_ERROR;
        }

        // Add a slight delay after reading packets; if the node is being tested offline
        // and this loop is hammering the TCP, logs won't output properly.
        ros::Duration(0.0001).sleep();

        return READ_SUCCESS;
    } else if (result == -2)
    {
        ROS_INFO("Done reading pcap file.");

        if (!last_packet.empty())
        {
            auto lastIpHeader = reinterpret_cast<const iphdr *>(&(last_packet[0]));
            uint32_t ipHeaderLength = lastIpHeader->ihl * 4u;

            auto lastTcpHeader = reinterpret_cast<const tcphdr *>(&(last_packet[0]) + ipHeaderLength);
            uint32_t dataOffset = lastTcpHeader->doff * 4u;

            buffer.insert(buffer.end(),
                          last_packet.begin() + ipHeaderLength + dataOffset,
                          last_packet.end());

            last_packet.clear();
        }
        disconnect();
        return READ_SUCCESS;
    } else
    {
        ROS_WARN("Error reading pcap data: %s", pcap_geterr(dev));
        return READ_ERROR;
    }
}

bool PCAP::write(const std::string &command)
{
    ROS_WARN_ONCE("Pcap files do not support writing");
    return true;
}

void PCAP::disconnect()
{
    if (dev)
    {
        pcap_close(dev);
        dev = nullptr;
    }
}
