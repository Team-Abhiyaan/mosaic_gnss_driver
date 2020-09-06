#include <serial.h>
#include <unistd.h>

namespace serial_util
{
    Config::Config() : m_BaudRate(115200),
                       m_DataBits(8),
                       m_StopBits(1),
                       m_Parity(NO_PARITY),
                       m_FlowControl(false),
                       m_LowLatencyMode(false),
                       m_Writable(false)
    {
    }

    Config::Config(int32_t baudRate,
                   int32_t dataBits,
                   int32_t stopBits,
                   Parity parity,
                   bool flowControl,
                   bool lowLatencyMode,
                   bool writable) : m_BaudRate(baudRate),
                                    m_DataBits(dataBits),
                                    m_StopBits(stopBits),
                                    m_Parity(parity),
                                    m_FlowControl(flowControl),
                                    m_LowLatencyMode(lowLatencyMode),
                                    m_Writable(writable)
    {
    }

    SerialPort::SerialPort() : m_Fd(-1),
                               m_ErrorMessage("")
    {
    }

    SerialPort::~SerialPort()
    {
        serialClose();
    }

    void SerialPort::serialClose()
    {
        if (m_Fd < 0)
            return;

        close(m_Fd);
        m_Fd = -1;
    }

    bool SerialPort::open(const std::string &device, Config config) {}

    SerialPort::Result SerialPort::readBytes(std::vector<uint8_t> &output, size_t maxBytes, int32_t timeout) {}

    int32_t SerialPort::write(const std::vector<uint8_t> &input) {}

    bool SerialPort::_setLowLatencyMode() {}

    int32_t SerialPort::_parseBaudRate(int32_t baudRate) {}

} // namespace serial_util
