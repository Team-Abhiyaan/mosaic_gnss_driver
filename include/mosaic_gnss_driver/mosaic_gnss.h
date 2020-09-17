#ifndef MOSAIC_GNSS_H_
#define MOSAIC_GNSS_H_

#include <map>
#include <string>
#include <vector>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <mosaic_utils/serial.h>

#include <mosaic_gnss_driver/connections/connection.h>
#include <mosaic_gnss_driver/connections/serial.h>
#include <mosaic_gnss_driver/connections/pcap.h>

struct ParseConnection
{
    using type = mosaic_gnss_driver::connections::Serial;

    ParseConnection(const std::string &connection)
    {
        if (connection == "pcap")
        {
            using type = mosaic_gnss_driver::connections::PCAP;
        }
    }
};

namespace mosaic_gnss_driver
{
    /**
     * Main driver class
     * 
     * @tparam Connection Connection type, from  mosaic_gnss_driver::connections
     * @tparam Parser An object with the method parse(buffer::type * , size_t)
     */
    template <typename Connection, typename Parser>
    class GNSS
    {
        // Internal Buffer for receiving data to and parsing from
        using buffer_t = typename Connection::buffer_t;
        buffer_t buffer;
        Parser p;

        Connection conn;

    public:

        GNSS() : conn{buffer}, p{}
        {
            static_assert(std::is_base_of<connections::Connection, Connection>::value,
                          "Connection should be subclasss of connections::Connection");
            // static_assert("PARse method is not ther)
        };

        

        bool connect(const std::string& device, const connections::Connection::Options &opts={})
        {
            return conn.connect(device, opts);
        }

        void disconnect() { conn.disconnect(); }

        /**
         * Receives data from the GNSS and parses it.
         * @return Success
         */
        bool tick()
        {
            if (!conn.is_connected())
                return false;
            if (conn.read() != Connection::read_result::READ_SUCCESS)
                return false;
            p.parse(buffer.data(), buffer.size());
            return true;
        }

        bool is_connected() const { return conn.is_connected(); }
    };
} // namespace mosaic_gnss_driver

// forcing the compiler to build to avoid linking issues
// template class mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF>;

#endif // MOSAIC_GNSS_H_
