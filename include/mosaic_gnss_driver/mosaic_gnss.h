#ifndef MOSAIC_GNSS_H_
#define MOSAIC_GNSS_H_

#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <pcap/pcap.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <mosaic_utils/serial.h>

namespace mosaic_gnss_driver
{
    /// Options
    typedef std::map<std::string, double> MosaicGNSSMessageOpts;

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
        bool connect(const std::string &device, ConnectionType connection, MosaicGNSSMessageOpts const &opts);

        /**
         * Disconnect from module
         */
        void disconnect();

        /**
         * Check if a connection to module exists
         * 
         * @return True if connection exists
         */
        bool isConnected() const
        {
            return m_bIsConnected;
        }

        /**
         * Get the most recent error message
         * 
         * @return The most recent error message
         */
        std::string errorMsg() const
        {
            return m_sErrorMessage;
        }

        /**
         * Convert the strings "udp", "tcp" or "serial" to the corresponding enum values
         * 
         * @param connection: A string indicating the connection type
         * @return Corresponding enum value
         */
        static ConnectionType parseConnection(const std::string &connection);

        /**
         * Process data that has been recieved from the device
         * 
         * @return A code indicating result of operation
         */
        ReadResult processData();

        /**
         * Sets the serial baud rate; should be called before configuring a serial connection.
         * 
         * @param serialBaud: The serial baud rate.
         */
        void setSerialBaud(int32_t serialBaud);

        /**
         * Only for development purposes, to be removed in release version
         * 
         * Spits out data buffer into stdout
         * 
         * // TODO: remove this in release version
         */
        void bufferDump();

    private:
        /**
         * (Re)configure the driver with a set of message options
         * 
         * @param opts: // TODO
         * 
         * @return True on success
         */
        bool _configure(MosaicGNSSMessageOpts const &opts);

        /**
         * Write the given string of characters to the connected mosaic module.
         * 
         * @param command: String to transmit
         * 
         * @return True on success, false otherwise.
         */
        bool _write(const std::string &command);

        /**
         * Create a PCAP device for playing back recorded data
         * 
         * @param device: PCAP file path
         * @param opts: options
         * @return True on success
         */
        bool _createPcapConnection(const std::string &device, MosaicGNSSMessageOpts const &opts);

        /**
         * Establishes a serial port connection with the mosaic module.
         *
         * It will create a connection set at the baud set by setSerialBaud(), no parity, 
         * no flow control, and 8N1 bits, then it will call _configure() on that connection.  After
         * this method has succeeded, _readData() and serialWrite() can be used to communicate with
         * the device.
         *
         * @param device:  The device node to connect to; e. g., "/dev/ttyUSB0"
         * @param opts: // TODO
         * 
         * @return True on success
         */
        bool _createSerialConnection(const std::string &device, MosaicGNSSMessageOpts const &opts);

        /**
       * Establishes an IP connection with the Mosaic module.
       *
       * Based on the current value in connection_, this will create either a TCP or UDP socket;
       * then, based on the value in endpoint, it will either create a connection to the module
       * on a specific port or wait for a connection from one.  In any case, this method will 
       * block until a connection is either established or failed.
       *
       * @param endpoint: A host and port specification; e. g., "192.168.1.10:1000".  If the host
       * host is omitted, it will listen for a connection the specified port.  If the port is
       * omitted, DEFAULT_TCP_PORT will be used for TCP connections and DEFAULT_UDP_PORT
       * for UDP connections.
       * @param opts :TODO 
       * @return false if it failed to create a connection, true otherwise.
       */
        bool _createIpConnection(const std::string &endpoint, MosaicGNSSMessageOpts const &opts);

        /**
         * Read data from module. Data appended will be appended to data buffer.
         * 
         * @return Status of read operation
         */
        ReadResult _readData();

        /**
         * Read data from serial port
         * 
         * Read data will be added to buffer. Called internally by _readData()
         */
        ReadResult _readSerialData();

        /**
         * Read data from socket. For TCP and UDP connections
         * 
         * Read data will be added to buffer. Called internally by _readData()
         */
        ReadResult _readIpData();

        /**
         * Read data from pcap file, for testing and development
         * 
         * Read data will be added to buffer. Called internally by _readData()
         */
        ReadResult _readPcapData();

        static constexpr uint16_t DEFAULT_TCP_PORT = 3001;
        static constexpr uint16_t DEFAULT_UDP_PORT = 3002;
        static constexpr size_t MAX_BUFFER_SIZE = 100;

        ConnectionType m_ConnectionType;
        bool m_bIsConnected;

        std::string m_sErrorMessage;

        // Serial connection
        int32_t m_SerialBaud;
        serial_util::SerialPort m_SerialPort;

        // IP (TCP / UDP) Connections
        boost::asio::io_service m_IoService;
        boost::asio::ip::tcp::socket m_TcpSocket;
        boost::shared_ptr<boost::asio::ip::udp::socket> m_UdpSocket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> m_UdpEndpoint;

        // Pcap device is used for testing and development
        pcap_t *m_Pcap;
        bpf_program m_PcapPacketFilter;
        char m_cPcapErrBuffer[MAX_BUFFER_SIZE];
        std::vector<uint8_t> m_vLastTcpPacket;

        /*************** Buffers ***************/
        // Buffer for holding read, raw data
        std::vector<uint8_t> m_vDataBuffer;
        // Fixed size buffer for reading directly from sockets
        boost::array<uint8_t, 10000> m_SocketBuffer;
    };
} // namespace mosaic_gnss_driver

#endif // MOSAIC_GNSS_H_
