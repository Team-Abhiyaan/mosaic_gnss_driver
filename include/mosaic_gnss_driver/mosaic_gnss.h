#ifndef MOSAIC_GNSS_H_
#define MOSAIC_GNSS_H_

#include <string>

#include <mosaic_gnss_driver/data_buffers.h>
#include <mosaic_gnss_driver/connections/connection.h>

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
        /// Internal Buffer for storing raw received data
        using buffer_t = typename Connection::buffer_t;
        buffer_t buffer;

        /// Structure for storing parsed messages.
        mosaic_gnss_driver::DataBuffers &data_buf;

    public:
        Parser p; /// Message parser

        Connection conn; /// Connection to Module

        /**
         * Only constructor
         *
         * @param buffers stores the parsed data for further use
         */
        explicit GNSS(mosaic_gnss_driver::DataBuffers &buffers) : data_buf{buffers}, conn{buffer}, p{data_buf}
        {
            static_assert(std::is_base_of<connections::Connection, Connection>::value,
                          "Connection should be subclass of connections::Connection");
            // static_assert("TODO: Check that parser has the `parse(const uint8_t *data, size_t size)` method")
        };

        /**
         * Attempts to connect to module and configures it
         * 
         * @param device: For serial connections, it is the device path. eg: /dev/TTYUSB0 \n
         *                For IP connections, a host:port specification eg: 192.168.3.1:3001 \n
         *                For PCAP connection, filename
         * @param opts: Configuration options
         * 
         * @return success
         */
        bool connect(const std::string &device, const connections::Connection::Options &opts = {})
        {
            return conn.connect(device, opts);
        }

        void disconnect()
        {
            conn.disconnect();
        }

        /**
         * Receive data from the GNSS and parse it.
         * 
         * @return device usable, i.e. if False, need to reset the driver
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

        bool is_connected() const
        {
            return conn.is_connected();
        }
    };
} // namespace mosaic_gnss_driver

// forcing the compiler to build to avoid linking issues
// template class mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF>;

#endif // MOSAIC_GNSS_H_
