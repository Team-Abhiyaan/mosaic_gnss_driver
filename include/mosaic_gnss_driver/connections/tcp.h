#ifndef MOSAIC_GNSS_DRIVER_TCP_H
#define MOSAIC_GNSS_DRIVER_TCP_H

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <mosaic_gnss_driver/connections/connection.h>

namespace mosaic_gnss_driver::connections {
    /**
     * Represents a TCP connection
     */
    class TCP : public Connection
    {
    private:
        static constexpr const char* const type = "TCP";

        /**
         * (Re)configure the driver with a set of message options
         *
         * @param opts: Configuration options
         *
         * @return True on success, false otherwise
         */
        bool _configure(const Options& opts);

        boost::asio::io_service m_IoService;
        boost::asio::ip::tcp::socket m_TcpSocket;
        boost::array<uint8_t, 10000> m_SocketBuffer;

    protected:
        static const size_t DEFAULT_TCP_PORT = 3001;

    public:
        /// Constructor
        explicit TCP(buffer_t& buf);

        /**
         * Attempts to connect to the module via TCP
         *
         * @param device: A host:port specification eg: 192.168.3.1:3001
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
        ~TCP();
    };
} // namespace mosaic_gnss_driver::connections
#endif // MOSAIC_GNSS_DRIVER_TCP_H
