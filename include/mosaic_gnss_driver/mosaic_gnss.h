#ifndef MOSAIC_GNSS_H_
#define MOSAIC_GNSS_H_

#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <mosaic_utils/serial.h>
#include <mosaic_gnss_driver/parsers/nmeaparse/NMEAParser.h>
#include <mosaic_gnss_driver/parsers/nmeaparse/GPSService.h>
#include <mosaic_gnss_driver/connections/connection.h>
#include <mosaic_gnss_driver/connections/serial.h>
#include <mosaic_gnss_driver/connections/pcap.h>
#include "data_buffers.h"

namespace mosaic_gnss_driver {
    /**
     * Main driver class
     * 
     * @tparam Connection Connection type, from  mosaic_gnss_driver::connections
     * @tparam Parser An object with the method parse(buffer::type * , size_t)
     */
    template<typename Connection, typename Parser>
    class GNSS {
        using buffer_t = typename Connection::buffer_t;
        /// Internal Buffer for storing raw data before parsing
        buffer_t buffer;
        /// Buffer for passing messages to ROS
        mosaic_gnss_driver::DataBuffers &data_buf;
        /// Message parser
        Parser p;
        /// Represents the connection to the module
        Connection conn;

    public:
        /// Constructor
        explicit GNSS(mosaic_gnss_driver::DataBuffers &buffers) :  data_buf{buffers}, conn{buffer}, p{data_buf} {
            static_assert(std::is_base_of<connections::Connection, Connection>::value,
                          "Connection should be subclasss of connections::Connection");
            // static_assert("PARse method is not ther)
        };

        /**
         * Attempts to connect to module and configure using given options
         * 
         * @param device: For serial connections, it is a filehandle. eg: /dev/TTYUSB0 \n
         *                For IP connections, a host:port specification eg: 192.168.3.1:3001 \n
         *                For PCAP connection, filename
         * @param opts: Configuration options
         * 
         * @return True on success, false otherwise
         */
        bool connect(const std::string &device, const connections::Connection::Options &opts = {}) {
            return conn.connect(device, opts);
        }

        /**
         * Disconnect from module
         */
        void disconnect() { conn.disconnect(); }

        /**
         * Receives data from the GNSS and parses it.
         * 
         * @return True on success, false otherwise
         */
        bool tick() {
            if (!conn.is_connected())
                return false;
            if (conn.read() != Connection::read_result::READ_SUCCESS)
                return false;
            p.parse(buffer.data(), buffer.size());
            return true;
        }

        /**
         * Check if a connection to module exists
         * 
         * @return True if connection exists, false otherwise
         */
        bool is_connected() const { return conn.is_connected(); }
    };
} // namespace mosaic_gnss_driver

// forcing the compiler to build to avoid linking issues
// template class mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF>;

#endif // MOSAIC_GNSS_H_
