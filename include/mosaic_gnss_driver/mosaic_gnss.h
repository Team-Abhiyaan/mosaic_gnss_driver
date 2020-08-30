#ifndef MOSAIC_GNSS_H_
#define MOSAIC_GNSS_H_

#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <pcap/pcap.h>

namespace mosaic_gnss_driver
{
    /// Options 
    typedef std::map<std::string, double> MosaicGNSSOpts;

    class MosaicGNSS
    {
    public:
        enum ConnectionType
        {
            /// For connection via Serial port
            SERIAL,
            /// For connection via TCP
            TCP,
            /// For connection via UDP
            UDP,
            /// Read data from pcap file, for development and testing
            PCAP,
            /// Invalid connection mode
            INVALID
        };
        /// Constructor
        MosaicGNSS();
        /// Destructor
        ~MosaicGNSS();

        /** 
         * Connect to module and configure using default options
         *
         * @param device: For serial connections, it is a filehandle. eg: /dev/TTYUSB0 \n
         *                For IP connections, a host:port specification eg: 192.168.3.1:3001 \n
         *                For PCAP connection, filename
         * @param connection: The Type of connection
         * @return True on success
         */
        bool connect(const std::string &device, ConnectionType connection);

        /**
         * Connect to module and configure using given options
         * 
         * @param device: For serial connections, it is a filehandle. eg: /dev/TTYUSB0 \n
         *                For IP connections, a host:port specification eg: 192.168.3.1:3001 \n
         *                For PCAP connection, filename
         * @param connection: The Type of connection
         * @param opts: Configuration options
         * @return True on success
         */
        bool connect(const std::string &device, ConnectionType connection, MosaicGNSSOpts const &opts);

        /**
         * Disconnect from module
         */
        void disconnect();

        /**
         * Check if a connection to module exists
         * 
         * @return True if connection exists
         */
        bool isConnected()
        {
            return m_bIsConnected;
        }

        /**
         * Convert the strings "udp", "tcp" or "serial" to the corresponding enum values
         * 
         * @param connection: A string indicating the connection type
         * @return Corresponding enum value
         */
        static ConnectionType parseConnection(const std::string& connection);

    private:
        /**
         * Create a PCAP device for playing back recorded data
         * 
         * @param device: filename
         * @param opts: options
         * @return True on success
         */
        bool _createPcapConnection(const std::string &device, MosaicGNSSOpts const &opts);

        bool _createSerialConnection();
        bool _createIpConnection();

        static constexpr uint16_t DEFAULT_TCP_PORT = 3001;
        static constexpr uint16_t DEFAULT_UDP_PORT = 3002;
        static constexpr size_t MAX_BUFFER_SIZE = 100;

        ConnectionType m_cConnectionType;
        bool m_bIsConnected;

        // Pcap device is used for testing and development
        pcap_t *m_pPcap;
        bpf_program m_PcapPacketFilter;
        char m_cPcapErrBuffer[MAX_BUFFER_SIZE];
        std::vector<uint8_t> m_vLastTcpPacket;
    };
} // namespace mosaic_gnss_driver

#endif // MOSAIC_GNSS_H_