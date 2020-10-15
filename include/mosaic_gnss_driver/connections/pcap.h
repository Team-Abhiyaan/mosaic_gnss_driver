#ifndef MOSAIC_GNSS_DRIVER_PCAP_H
#define MOSAIC_GNSS_DRIVER_PCAP_H

#include <mosaic_gnss_driver/connections/connection.h>

#include <pcap/pcap.h>
#include <memory>
#include <vector>

namespace mosaic_gnss_driver::connections
{
    /**
     * Represents a PCAP connection
     */
    class PCAP : public Connection
    {
        static constexpr const char *const type = "PCAP";

    protected:
        static const size_t BUFFER_SIZE = 100;

        pcap_t *dev{nullptr};
        bpf_program packet_filter{};
        char err_buffer[BUFFER_SIZE]{};
        std::vector<uint8_t> last_packet;

    public:
        using Connection::Connection; // Use superclass constructor

        /**
         * Tries to connect to a PCAP file
         * 
         * @param device: Path of file to connect to
         * @param opts: Configuration options, redundant in this case
         *
         * @return True if successful, false otherwise
         */
        bool connect(const std::string &device, const Options &opts = {}) override;

        void disconnect() override;

        bool is_connected() const override
        { return dev; }

        ReadResult read() override;

        bool write(const std::string &command) override;

        /// Destructor
        ~PCAP();
    };
} // namespace mosaic_gnss_driver::connections

#endif //MOSAIC_GNSS_DRIVER_PCAP_H
