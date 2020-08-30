#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <ros/ros.h>

namespace mosaic_gnss_driver
{
    MosaicGNSS::MosaicGNSS() : m_cConnectionType(SERIAL),
                               m_bIsConnected(false),
                               m_pPcap(nullptr)
    {
    }

    MosaicGNSS::~MosaicGNSS()
    {
        disconnect();
    }

    void MosaicGNSS::disconnect()
    {
        // todo : add serial, tcp, udp disconnections and socket closing

        if (m_cConnectionType == PCAP)
        {
            if (m_pPcap != nullptr)
            {
                pcap_close(m_pPcap);
                m_pPcap = nullptr;
            }
        }

        m_bIsConnected = false;
    }

    bool MosaicGNSS::connect(const std::string &device, ConnectionType connection)
    {
        MosaicGNSSOpts opts;

        // TODO : Set default options here

        return connect(device, connection, opts);
    }

    bool MosaicGNSS::connect(const std::string &device, ConnectionType connection, MosaicGNSSOpts const &opts)
    {
        disconnect();

        // todo: add serial, udp, tcp connections

        m_cConnectionType = connection;

        if (connection == PCAP)
        {
            return _createPcapConnection(device, opts);
        }
    }

    bool MosaicGNSS::_createPcapConnection(const std::string &device, MosaicGNSSOpts const &opts)
    {
        ;
    }

} // namespace mosaic_gnss_driver