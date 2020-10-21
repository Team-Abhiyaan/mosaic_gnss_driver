#ifndef MOSAIC_GNSS_DRIVER_UDP_H
#define MOSAIC_GNSS_DRIVER_UDP_H

#include <mosaic_gnss_driver/connections/connection.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>

namespace mosaic_gnss_driver::connections
{
    /**
     * Represents a UDP connection
     */
    class UDP : public Connection
    {
    private:
        static constexpr const char *const type = "UDP";

        boost::asio::io_service m_IoService;
        boost::shared_ptr<boost::asio::ip::udp::socket> m_UdpSocket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> m_UdpEndpoint;
        boost::array<uint8_t, 10000> m_SocketBuffer;

        /**
         * (Re)configure the driver with a set of message options
         * 
         * @param opts: Configuration options
         * 
         * @return True on success, false otherwise
         */
        bool _configure(const Options &opts);

    protected:
        static const size_t DEFAULT_UDP_PORT = 3002;

    public:
        /// Constructor
        explicit UDP(buffer_t &buf);

        /**
         * Attempts to connect to the module via UDP
         * 
         * @param device: A host:port specification eg: 192.168.3.1:3002
         * @param opts: Configuration options
         * 
         * @return True if successful, false otherwise
         */
        bool connect(const std::string &device, const Options &opts = {}) override;

        void disconnect() override;

        bool is_connected() const override
        { return connected; }

        ReadResult read() override;

        bool write(const std::string &command) override;

        /// Destructor
        ~UDP();
    };
} // namespace mosaic_gnss_driver::connections
#endif //MOSAIC_GNSS_DRIVER_UDP_H
