#ifndef MOSAIC_GNSS_DRIVER_CONNECTION_H
#define MOSAIC_GNSS_DRIVER_CONNECTION_H

#include <map>
#include <string>
#include <vector>

namespace mosaic_gnss_driver::connections {
    enum ReadResult
    {
        /// Data read successfully
        READ_SUCCESS = 0,
        READ_INSUFFICIENT_DATA = 1,
        READ_TIMEOUT = 2,
        READ_INTERRUPTED = 3,
        READ_ERROR = -1,
        /// Unable to parse data, parsing error
        READ_PARSE_FAILED = -2
    };

    /**
     * Represents a connection to the GNSS module
     */
    class Connection
    {
        /// store type of connection
        static const char* const type;

    public:
        using buffer_t = std::vector<uint8_t>;
        /// To pass options to the driver while initialising
        using Options = std::map<std::string, double>;
        using read_result = ReadResult;

    protected:
        bool connected{false};
        /// Buffer to which any read raw data will be written to
        buffer_t& buffer;

    public:
        /**
         * Constructor
         *
         * @param buf: a vector to which incoming data is stored
         */
        explicit Connection(buffer_t& buf) : buffer{buf} {}

        /**
         * Check if a connection to module exists
         *
         * @return True if connection exists, false otherwise
         */
        virtual bool is_connected() const { return connected; };

        /**
         * Attempts to connect to the GNSS Device
         *
         * @param device: address of the GNSS device (refer subclass documentation for specific
         * details)
         * @param opts: Configuration options
         *
         * @return True if successful, false otherwise
         */
        virtual bool connect(const std::string& device, const Options& opts) = 0;

        /**
         * Disconnect from the module
         */
        virtual void disconnect() = 0;

        /**
         * Reads incoming data from the module, any read data will be appended to `buffer`
         *
         * @return A code indicating status of operation
         */
        virtual ReadResult read() = 0;

        /**
         * Write the given command to the connected module
         *
         * @param command: The command to be written
         *
         * @return True on success, false otherwise
         */
        virtual bool write(const std::string& command) = 0;

        /**
         * Get existing mode of connection
         *
         * @return String representing current mode of connection
         */
        static const char* get_type() { return type; }
    };
} // namespace mosaic_gnss_driver::connections

#endif // MOSAIC_GNSS_DRIVER_CONNECTION_H
