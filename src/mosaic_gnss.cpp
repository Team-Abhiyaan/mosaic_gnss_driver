#include <netinet/ip.h>
#include <net/ethernet.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#include <iostream>

#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <ros/ros.h>

namespace mosaic_gnss_driver
{
    MosaicGNSS::MosaicGNSS() : m_ConnectionType(SERIAL),
                               m_bIsConnected(false),
                               m_SerialBaud(115200),
                               m_TcpSocket(m_IoService),
                               m_Pcap(nullptr)
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
        switch (m_ConnectionType)
        {
        case SERIAL:
        {
            m_SerialPort.serialClose();
            break;
        }

        case TCP:
        {
            m_TcpSocket.close();
            break;
        }

        case UDP:
        {
            if (m_UdpSocket)
            {
                m_UdpSocket->close();
                m_UdpSocket.reset();
            }
            if (m_UdpEndpoint)
            {
                m_UdpEndpoint.reset();
            }
            break;
        }

        case PCAP:
        {
            if (m_Pcap != nullptr)
            {
                pcap_close(m_Pcap);
                m_Pcap = nullptr;
            }
            break;
        }
        default:
            break;
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
        m_ConnectionType = connection;

        switch (m_ConnectionType)
        {
        case SERIAL:
            return _createSerialConnection(device, opts);

        case TCP:
            return _createIpConnection(device, opts);

        case UDP:
            return _createIpConnection(device, opts);

        case PCAP:
            return _createPcapConnection(device, opts);

        default:
            m_sErrorMessage = "Invalid Connection type.";
            return false;
        }
    }

    MosaicGNSS::ReadResult MosaicGNSS::processData()
    {
        // TODO: complete this method
        MosaicGNSS::ReadResult readResult = _readData();

        if (readResult != READ_SUCCESS)
        {
            return readResult;
        }

        ros::Time stamp = ros::Time::now();

        if (!m_vDataBuffer.empty())
        {
            ;
        }

        return READ_SUCCESS;
    }

    void MosaicGNSS::setSerialBaud(int32_t serialBaud)
    {
        m_SerialBaud = serialBaud;
        ROS_INFO("Serial baud rate: %d", serialBaud);
    }

    bool MosaicGNSS::_configure(MosaicGNSSMessageOpts const &opts)
    {
        // TODO: Complete this
        return true;
    }

    bool MosaicGNSS::_write(const std::string &command)
    {
        std::vector<uint8_t> bytes(command.begin(), command.end());

        switch (m_ConnectionType)
        {
        case SERIAL:
        {
            int32_t written = m_SerialPort.serialWrite(bytes);
            bool success = (written == (int32_t)command.length());

            if (!success)
            {
                ROS_ERROR("Failed to send command: %s", command.c_str());
            }

            return success;
        }
        case TCP:
        case UDP:
        {
            boost::system::error_code error;

            try
            {
                size_t written; // to store number of bytes written

                if (m_ConnectionType == TCP)
                {
                    written = boost::asio::write(m_TcpSocket, boost::asio::buffer(bytes), error);
                }
                else
                {
                    written = m_UdpSocket->send_to(boost::asio::buffer(bytes), *m_UdpEndpoint, 0, error);
                }

                if (error)
                {
                    ROS_ERROR("Error writing IP data: %s", error.message().c_str());
                    disconnect();
                }
                ROS_DEBUG("Wrote %lu bytes", written);

                return (written == (int32_t)command.length());
            }
            catch (std::exception &e)
            {
                disconnect();
                ROS_ERROR("Exception writing IP data: %s", e.what());
                return false;
            }
        }
        case PCAP:
        {
            ROS_WARN_ONCE("Writing data is unsupported in PCAP mode.");
            return true;
        }
        default:
            return false;
        }
    }

    bool MosaicGNSS::_createPcapConnection(const std::string &device, MosaicGNSSMessageOpts const &opts)
    {
        ROS_INFO("Opening pcap file: %s", device.c_str());

        if ((m_Pcap = pcap_open_offline(device.c_str(), m_cPcapErrBuffer)) == nullptr)
        {
            ROS_FATAL("Unable to open pcap file");
            m_bIsConnected = false;
            return false;
        }

        pcap_compile(m_Pcap, &m_PcapPacketFilter, "tcp dst port 3001", 1, PCAP_NETMASK_UNKNOWN);
        m_bIsConnected = true;

        return true;
    }

    bool MosaicGNSS::_createSerialConnection(const std::string &device, MosaicGNSSMessageOpts const &opts)
    {
        serial_util::Config config;
        config.m_BaudRate = m_SerialBaud;
        config.m_Parity = serial_util::Config::NO_PARITY;
        config.m_FlowControl = false;
        config.m_DataBits = 8;
        config.m_StopBits = 1;
        config.m_LowLatencyMode = false;
        config.m_Writable = true;

        bool success = m_SerialPort.serialOpen(device, config);

        if (success)
        {
            m_bIsConnected = true;
            if (_configure(opts))
            {
                // We will not kill the connection here, because the device may already
                // be setup to communicate correctly, but we will print a warning
                ROS_ERROR("Failed to configure module. This port may be read only, or the "
                          "device may not be functioning as expected; however, the "
                          "driver may still function correctly if the port has already "
                          "been pre-configured.");
            }
        }
        else
        {
            m_sErrorMessage = m_SerialPort.errorMsg();
        }

        return success;
    }

    bool MosaicGNSS::_createIpConnection(const std::string &endpoint, MosaicGNSSMessageOpts const &opts)
    {
        std::string ip;
        std::string port;
        uint16_t numPort;
        size_t separatorPosition = endpoint.find(':');

        if (separatorPosition == std::string::npos || separatorPosition == endpoint.size() - 1)
        {
            ROS_INFO("Using default port");
            std::stringstream ss;

            switch (m_ConnectionType)
            {
            case TCP:
                numPort = DEFAULT_TCP_PORT;
                break;
            case UDP:
                numPort = DEFAULT_UDP_PORT;
                break;
            }

            ss << numPort;
            port = ss.str();
        }
        else
        {
            port = endpoint.substr(separatorPosition + 1);
        }

        if (separatorPosition != 0)
        {
            ip = endpoint.substr(0, separatorPosition);
        }

        ROS_INFO("Trying to connect to IP: %s at PORT: %s", ip.c_str(), port.c_str());

        try
        {
            if (!ip.empty())
            {
                if (m_ConnectionType == TCP)
                {
                    boost::asio::ip::tcp::resolver resolver(m_IoService);
                    boost::asio::ip::tcp::resolver::query query(ip, port);
                    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

                    boost::asio::connect(m_TcpSocket, iter);

                    ROS_INFO("Connecting via TCP to %s:%s", ip.c_str(), port.c_str());
                }
                else
                {
                    boost::asio::ip::udp::resolver resolver(m_IoService);
                    boost::asio::ip::udp::resolver::query query(ip, port);
                    m_UdpEndpoint = boost::make_shared<boost::asio::ip::udp::endpoint>(*resolver.resolve(query));

                    m_UdpSocket.reset(new boost::asio::ip::udp::socket(m_IoService));
                    m_UdpSocket->open(boost::asio::ip::udp::v4());

                    ROS_INFO("Connecting via UDP to %s:%s", ip.c_str(), port.c_str());
                }
            }
            else
            {
                auto portNumber = static_cast<uint16_t>(strtoll(port.c_str(), nullptr, 10));

                if (m_ConnectionType == TCP)
                {
                    boost::asio::ip::tcp::acceptor acceptor(m_IoService, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), portNumber));

                    ROS_INFO("Listening to TCP port %s", port.c_str());

                    acceptor.accept(m_TcpSocket);

                    ROS_INFO("Accepted TCP Connection from client: %s", m_TcpSocket.remote_endpoint().address().to_string().c_str());
                }
                else
                {
                    m_UdpSocket.reset(new boost::asio::ip::udp::socket(m_IoService, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), portNumber)));

                    boost::array<char, 1> recvBuffer;

                    m_UdpEndpoint = boost::make_shared<boost::asio::ip::udp::endpoint>();
                    boost::system::error_code error;

                    ROS_INFO("Listening to UDP port %s", port.c_str());

                    m_UdpSocket->receive_from(boost::asio::buffer(recvBuffer), *m_UdpEndpoint, 0, error);

                    if (error && error != boost::asio::error::message_size)
                    {
                        throw boost::system::system_error(error);
                    }

                    ROS_INFO("Accepted UDP Connection from client: %s", m_UdpEndpoint->address().to_string().c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            m_sErrorMessage = e.what();
            ROS_ERROR("Unable to connect: %s", e.what());
            return false;
        }

        m_bIsConnected = true;

        if (_configure(opts))
        {
            ROS_INFO("Configured Mosaic module.");
        }
        else
        {
            // We will not kill the connection here, because the device may already
            // be setup to communicate correctly, but we will print a warning
            ROS_ERROR("Failed to configure GPS. This port may be read only, or the "
                      "device may not be functioning as expected; however, the "
                      "driver may still function correctly if the port has already "
                      "been pre-configured.");
        }
        return true;
    }

    MosaicGNSS::ReadResult MosaicGNSS::_readData()
    {
        switch (m_ConnectionType)
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

    MosaicGNSS::ReadResult MosaicGNSS::_readSerialData()
    {
        serial_util::SerialPort::ReadResult result = m_SerialPort.readBytes(m_vDataBuffer, 0, 1000);

        if (result == serial_util::SerialPort::ERROR)
        {
            m_sErrorMessage = m_SerialPort.errorMsg();
            return READ_ERROR;
        }
        else if (result == serial_util::SerialPort::TIMEOUT)
        {
            m_sErrorMessage = "Timed out waiting for serial device.";
            return READ_TIMEOUT;
        }
        else if (result == serial_util::SerialPort::INTERRUPTED)
        {
            m_sErrorMessage = "Interrupted during read from serial device.";
            return READ_INTERRUPTED;
        }

        return READ_SUCCESS;
    }

    MosaicGNSS::ReadResult MosaicGNSS::_readIpData()
    {
        try
        {
            boost::system::error_code error;
            // number of bytes read from the socket
            size_t length;

            if (m_ConnectionType == TCP)
            {
                length = m_TcpSocket.read_some(boost::asio::buffer(m_SocketBuffer), error);
            }
            else
            {
                boost::asio::ip::udp::endpoint remoteEndpoint;
                length = m_UdpSocket->receive_from(boost::asio::buffer(m_SocketBuffer), remoteEndpoint);
            }
            m_vDataBuffer.insert(m_vDataBuffer.end(), m_SocketBuffer.begin(), m_SocketBuffer.begin() + length);

            if (error)
            {
                m_sErrorMessage = error.message();
                ROS_ERROR("Error occured in TCP connection: %s", m_sErrorMessage.c_str());

                disconnect();
                return READ_ERROR;
            }
            return READ_SUCCESS;
        }
        catch (const std::exception &e)
        {
            ROS_WARN("TCP Connection error: %s", e.what());
            return READ_ERROR;
        }
    }

    MosaicGNSS::ReadResult MosaicGNSS::_readPcapData()
    {
        struct pcap_pkthdr *header;
        const u_char *packetData;

        int result;

        // Read next packet
        result = pcap_next_ex(m_Pcap, &header, &packetData);

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
                uint16_t fragOff = ntohs(static_cast<uint16_t>(ipHeader->frag_off));

                uint16_t fragmentOffset = fragOff & static_cast<uint16_t>(0x1FFF);
                size_t headerSize;

                // UDP packets may be fragmented; this isn't really "correct", but for
                // simplicity's sake we'll assume we get fragments in the right order.
                if (fragmentOffset == 0)
                {
                    headerSize = sizeof(struct ethhdr) + ipHeaderLength + sizeof(struct udphdr);
                }
                else
                {
                    headerSize = sizeof(struct ethhdr) + ipHeaderLength;
                }

                m_vDataBuffer.insert(m_vDataBuffer.end(), packetData + headerSize, packetData + header->len);

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
        }
        else if (result == -2)
        {
            ROS_INFO("Done reading pcap file.");

            if (!m_vLastTcpPacket.empty())
            {
                auto lastIpHeader = reinterpret_cast<const iphdr *>(&(m_vLastTcpPacket[0]));
                uint32_t ipHeaderLength = lastIpHeader->ihl * 4u;

                auto lastTcpHeader = reinterpret_cast<const tcphdr *>(&(m_vLastTcpPacket[0]) + ipHeaderLength);
                uint32_t dataOffset = lastTcpHeader->doff * 4u;

                m_vDataBuffer.insert(m_vDataBuffer.end(),
                                     m_vLastTcpPacket.begin() + ipHeaderLength + dataOffset,
                                     m_vLastTcpPacket.end());

                m_vLastTcpPacket.clear();
            }
            disconnect();
            return READ_SUCCESS;
        }
        else
        {
            ROS_WARN("Error reading pcap data: %s", pcap_geterr(m_Pcap));
            return READ_ERROR;
        }
    }

    void MosaicGNSS::bufferDump()
    {
        std::cout << "MosaicGNSS::bufferDump() -> Use of this method is not encouraged and is for development purposes only" << std::endl;
        std::cout << "\n\n";
        for (auto &data : m_vDataBuffer)
        {
            std::cout << data;
        }
        std::cout << "\n\n";
    }
} // namespace mosaic_gnss_driver
