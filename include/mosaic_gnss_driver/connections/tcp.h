#ifndef MOSAIC_GNSS_DRIVER_TCP_H
#define MOSAIC_GNSS_DRIVER_TCP_H

#include <mosaic_gnss_driver/connections/connection.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>

namespace mosaic_gnss_driver::connections
{
    class TCP : public Connection
    {
    private:
        static constexpr const char *const type = "TCP";

        bool _configure(const Options &opts);

        boost::asio::io_service m_IoService;
        boost::asio::ip::tcp::socket m_TcpSocket;
        boost::array<uint8_t, 10000> m_SocketBuffer;

    protected:
        static const size_t DEFAULT_TCP_PORT = 3001;

    public:
        explicit TCP(buffer_t &buf);

        bool connect(const std::string &device, const Options &opts = {}) override;

        void disconnect() override;

        // TODO: change this
        bool is_connected() const override { return connected; }

        ReadResult read() override;

        bool write(const std::string &command) override;

        ~TCP();
    };
} // namespace mosaic_gnss_driver::connections
#endif //MOSAIC_GNSS_DRIVER_TCP_H
