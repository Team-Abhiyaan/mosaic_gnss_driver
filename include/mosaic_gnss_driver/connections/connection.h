#ifndef MOSAIC_GNSS_DRIVER_CONNECTION_H
#define MOSAIC_GNSS_DRIVER_CONNECTION_H

#include <string>
#include <map>
#include <vector>

namespace mosaic_gnss_driver::connections {
    enum ReadResult {
        /// Data read successfully
        READ_SUCCESS = 0,
        READ_INSUFFICIENT_DATA = 1,
        READ_TIMEOUT = 2,
        READ_INTERRUPTED = 3,
        READ_ERROR = -1,
        /// Unable to parse data, parsing error
        READ_PARSE_FAILED = -2
    };


    class Connection {
        static const char *const type;
    public:
        using buffer_t = std::vector<uint8_t>;
        using Options = std::map<std::string, double>;
    protected:
        bool connected{false};
        buffer_t &buffer;
    public:
        using read_result = ReadResult;

        explicit Connection(buffer_t &buf) : buffer{buf} {}

        virtual bool is_connected() const { return connected; };

        virtual bool connect(const std::string &device, const Options &opts) = 0;

        virtual void disconnect() = 0;


        virtual ReadResult read() = 0;


        static const char *get_type() { return type; }

        // ~Connection() {disconnect();};
    };
}
#endif //MOSAIC_GNSS_DRIVER_CONNECTION_H
