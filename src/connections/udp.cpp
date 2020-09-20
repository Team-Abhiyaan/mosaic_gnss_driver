#include <mosaic_gnss_driver/connections/udp.h>

#include <ros/ros.h>

using namespace mosaic_gnss_driver::connections;

UDP::UDP(buffer_t &buf) : Connection(buf)
{
}

bool UDP::connect(const std::string &endpoint, const Options &opts)
{
    std::string ip;
    std::string port;
    uint16_t numPort;
    size_t separatorPosition = endpoint.find(":");

    if (separatorPosition == std::string::npos || separatorPosition == endpoint.size() - 1)
    {
        ROS_INFO("Using default port");
        std::stringstream ss;

        numPort = DEFAULT_UDP_PORT;

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

    ROS_INFO("Trying to connect to IP: %s at PORT: %s via UDP", ip.c_str(), port.c_str());

    try
    {
        if (!ip.empty())
        {

            boost::asio::ip::udp::resolver resolver(m_IoService);
            boost::asio::ip::udp::resolver::query query(ip, port);
            m_UdpEndpoint = boost::make_shared<boost::asio::ip::udp::endpoint>(*resolver.resolve(query));

            m_UdpSocket.reset(new boost::asio::ip::udp::socket(m_IoService));
            m_UdpSocket->open(boost::asio::ip::udp::v4());

            ROS_INFO("Connecting via UDP to %s:%s", ip.c_str(), port.c_str());
        }
        else
        {
            auto portNumber = static_cast<uint16_t>(strtoll(port.c_str(), nullptr, 10));

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
    catch (const std::exception &e)
    {
        ROS_ERROR("Unable to connect: %s", e.what());
        return false;
    }

    connected = true;

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

bool UDP::_configure(const Options &opts)
{
    // TODO: complete this
    return true;
}

void UDP::disconnect()
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
}

ReadResult UDP::read()
{
    try
    {
        boost::system::error_code error;
        // number of bytes read from the socket
        size_t length;

        boost::asio::ip::udp::endpoint remoteEndpoint;
        length = m_UdpSocket->receive_from(boost::asio::buffer(m_SocketBuffer), remoteEndpoint);

        buffer.insert(buffer.end(), m_SocketBuffer.begin(), m_SocketBuffer.begin() + length);

        if (error)
        {
            ROS_ERROR("Error occured in UDP connection: %s", error.message().c_str());

            disconnect();
            return READ_ERROR;
        }
        return READ_SUCCESS;
    }
    catch (const std::exception &e)
    {
        ROS_WARN("UDP Connection error: %s", e.what());
    }
}

bool UDP::write(const std::string &command)
{
    std::vector<uint8_t> bytes(command.begin(), command.end());

    boost::system::error_code error;

    try
    {
        size_t written; // to store number of bytes written

        written = m_UdpSocket->send_to(boost::asio::buffer(bytes), *m_UdpEndpoint, 0, error);

        if (error)
        {
            ROS_ERROR("Error writing UDP data: %s", error.message().c_str());
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

UDP::~UDP()
{
    disconnect();
}