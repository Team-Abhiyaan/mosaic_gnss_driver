#ifndef MOSAIC_GNSS_H_
#define MOSAIC_GNSS_H_

#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <pcap/pcap.h>

namespace mosaic_gnss_driver
{

    typedef std::map<std::string, double> MosaicGNSSOpts;

    class MosaicGNSS
    {
    public:
        enum ConnectionType
        {
            SERIAL,
            TCP,
            UDP,
            PCAP,
            INVALID
        };

        MosaicGNSS();
        ~MosaicGNSS();

        bool connect(const std::string &device, ConnectionType connection);

        bool connect(const std::string &device, ConnectionType connection, MosaicGNSSOpts const &opts);

        void disconnect();

        bool isConnected()
        {
            return m_bIsConnected;
        }

    private:
        bool _createPcapConnection(const std::string &device, MosaicGNSSOpts const &opts);

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