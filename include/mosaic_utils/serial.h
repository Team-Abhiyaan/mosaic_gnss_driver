#ifndef MOSAIC_SERIAL_H_
#define MOSAIC_SERIAL_H_

#include <string>
#include <vector>

namespace serial_util
{
    struct Config
    {
        enum Parity
        {
            NO_PARITY,
            EVEN_PARITY,
            ODD_PARITY
        };

        /**
         * Configure with default settings, settings as mentioned in the mosaic firmware guide
         */
        Config();

        /**
         * Configure using custom settings
         * 
         * TODO: add params description
         */
        Config(int32_t baudRate, int32_t dataBits, int32_t stopBits, Parity parity, bool flowControl, bool lowLatencyMode, bool writable);

        int32_t m_BaudRate;
        int32_t m_DataBits;
        int32_t m_StopBits;
        Parity m_Parity;
        bool m_FlowControl;
        bool m_LowLatencyMode;
        bool m_Writable;
    };

    class SerialPort
    {
    public:
        enum Result
        {
            SUCCESS,
            TIMEOUT,
            INTERRUPTED,
            ERROR
        };

        // File descriptor
        int m_Fd;
        // For latest error message
        std::string m_ErrorMessage;

        /**
         * Constructor
         */
        SerialPort();

        /**
         * Destructor
         */
        ~SerialPort();

        bool open(const std::string &device, Config config = Config());

        void serialClose();

        Result readBytes(std::vector<uint8_t> &output, size_t maxBytes, int32_t timeout);

        int32_t write(const std::vector<uint8_t> &input);

        std::string errorMsg() const
        {
            return m_ErrorMessage;
        }

    private:
        bool _setLowLatencyMode();

        int32_t _parseBaudRate(int32_t baudRate);
    };

} // namespace serial_util

#endif // MOSAIC_SERIAL_H_