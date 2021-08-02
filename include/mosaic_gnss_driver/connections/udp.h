#ifndef MOSAIC_GNSS_DRIVER_UDP_H
#define MOSAIC_GNSS_DRIVER_UDP_H

#include <array>
#include <asio.hpp>
#include <memory>
#include <mosaic_gnss_driver/connections/connection.h>

namespace mosaic_gnss_driver::connections {
    /**
     * Represents a UDP connection
     */
    class UDP : public Connection
    {
    private:
        static constexpr const char* const type = "UDP";

        asio::io_service m_IoService;
        std::shared_ptr<asio::ip::udp::socket> m_UdpSocket;
        std::shared_ptr<asio::ip::udp::endpoint> m_UdpEndpoint;
        std::array<uint8_t, 10000> m_SocketBuffer;

        /**
         * (Re)configure the driver with a set of message options
         *
         * @param opts: Configuration options
         *
         * @return True on success, false otherwise
         */
        bool _configure(const Options& opts);

    protected:
        static const size_t DEFAULT_UDP_PORT = 3002;

    public:
        /// Constructor
        explicit UDP(buffer_t& buf);

        /**
         * Attempts to connect to the module via UDP
         *
         * @param device: A host:port specification eg: 192.168.3.1:3002
         * @param opts: Configuration options
         *
         * @return True if successful, false otherwise
         */
        bool connect(const std::string& device, const Options& opts = {}) override;

        void disconnect() override;

        bool is_connected() const override { return connected; }

        ReadResult read() override;

        bool write(const std::string& command) override;

        /// Destructor
        ~UDP();
    };
} // namespace mosaic_gnss_driver::connections
#endif // MOSAIC_GNSS_DRIVER_UDP_H
