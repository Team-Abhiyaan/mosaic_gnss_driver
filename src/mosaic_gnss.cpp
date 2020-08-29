#include <mosaic_gnss_driver/mosaic_gnss.h>

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

    bool MosaicGNSS::connect(const std::string &device, ConnectionType connection)
    {
        // TODO : create another connect with options as parameter, and here load default params and give to connect method with params
        ;
    }

    bool MosaicGNSS::createPcapConnection(const std::string& device)
    {
        ;
    }

} // namespace mosaic_gnss_driver