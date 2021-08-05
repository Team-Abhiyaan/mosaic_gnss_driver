#include <mosaic_gnss_driver/connections/tcp.h>
#include <ros/ros.h>

using namespace mosaic_gnss_driver::connections;

TCP::TCP(buffer_t& buf) : Connection(buf), m_TcpSocket(m_IoService) {}

bool TCP::connect(const std::string& endpoint, const Options& opts)
{

    if (is_connected())
        return true;

    std::string ip, port;
    uint16_t numPort;
    size_t separatorPosition = endpoint.find(":");

    if (separatorPosition == std::string::npos || separatorPosition == endpoint.size() - 1)
    {
        ROS_INFO("Using default port");
        std::stringstream ss;

        numPort = DEFAULT_TCP_PORT;

        ss << numPort;
        port = ss.str();
    } else
    {
        port = endpoint.substr(separatorPosition + 1);
    }

    if (separatorPosition != 0)
    {
        ip = endpoint.substr(0, separatorPosition);
    }

    ROS_INFO("Trying to connect to IP: %s at PORT: %s via TCP", ip.c_str(), port.c_str());

    try
    {
        if (!ip.empty())
        {
            asio::ip::tcp::resolver resolver(m_IoService);
            asio::ip::tcp::resolver::query query(ip, port);
            asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

            asio::connect(m_TcpSocket, iter);

            ROS_INFO("Connecting via TCP to %s:%s", ip.c_str(), port.c_str());
        } else
        {
            auto portNumber = static_cast<uint16_t>(strtoll(port.c_str(), nullptr, 10));

            asio::ip::tcp::acceptor acceptor(
                m_IoService, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), portNumber));

            ROS_INFO("Listening to TCP port %s", port.c_str());

            acceptor.accept(m_TcpSocket);

            ROS_INFO("Accepted TCP Connection from client: %s",
                     m_TcpSocket.remote_endpoint().address().to_string().c_str());
        }
    } catch (const std::exception& e)
    {
        ROS_ERROR("Unable to connect: %s", e.what());
        return false;
    }

    connected = true;

    if (_configure(opts))
    {
        ROS_INFO("Configured Mosaic module.");
    } else
    {
        // We will not kill the connection here, because the device may already
        // be setup to communicate correctly, but we will print a warning
        ROS_ERROR(
            "Failed to configure GPS. This port may be read only, or the "
            "device may not be functioning as expected; however, the "
            "driver may still function correctly if the port has already "
            "been pre-configured.");
    }
    return true;
}

bool TCP::_configure(const Options& opts)
{
    // TODO: Complete this
    return true;
}

void TCP::disconnect() { m_TcpSocket.close(); }

ReadResult TCP::read()
{
    try
    {
        std::error_code error;
        // number of bytes read from the socket
        size_t length;

        length = m_TcpSocket.read_some(asio::buffer(m_SocketBuffer), error);

        buffer.insert(buffer.end(), m_SocketBuffer.begin(), m_SocketBuffer.begin() + length);

        if (error)
        {
            ROS_ERROR("Error occured in TCP connection: %s", error.message().c_str());

            disconnect();
            return READ_ERROR;
        }
        return READ_SUCCESS;
    } catch (const std::exception& e)
    {
        ROS_WARN("TCP Connection error: %s", e.what());
        return READ_ERROR;
    }
}

bool TCP::write(const std::string& command)
{
    std::vector<uint8_t> bytes(command.begin(), command.end());

    std::error_code error;

    try
    {
        size_t written; // to store number of bytes written

        written = asio::write(m_TcpSocket, asio::buffer(bytes), error);

        if (error)
        {
            ROS_ERROR("Error writing TCP data: %s", error.message().c_str());
            disconnect();
        }
        ROS_DEBUG("Wrote %lu bytes", written);

        return (written == (int32_t)command.length());
    } catch (std::exception& e)
    {
        disconnect();
        ROS_ERROR("Exception writing IP data: %s", e.what());
        return false;
    }
}

TCP::~TCP() { disconnect(); }
