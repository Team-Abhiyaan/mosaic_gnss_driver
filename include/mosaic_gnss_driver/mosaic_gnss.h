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

namespace mosaic_gnss_driver
{
    /**
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

    public:
        Connection conn;

        GNSS() : conn{buffer}, p{}
        {
            static_assert(std::is_base_of<connections::Connection, Connection>::value,
                          "Connection should be subclasss of connections::Connection");
            // static_assert("PARse method is not ther)
        };

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
    };
} // namespace mosaic_gnss_driver

// forcing the compiler to build to avoid linking issues
// template class mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF>;

#endif // MOSAIC_GNSS_H_
