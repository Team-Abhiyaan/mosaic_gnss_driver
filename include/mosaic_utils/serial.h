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
         * 
         * Baud rate = 115200
         * Data bits = 8
         * Stop bits = 1
         * Parity = NO_PARITY
         * Flow Control = false
         * Low latency mode = false
         * Writable = false
         */
        Config();

        /**
         * Configure using custom settings
         * 
         * // TODO: add params description
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
         * 
         * Closes serial port if open.
         */
        ~SerialPort();

        /**
         * Open and configure the serial port
         * 
         * The default configuration is: 
         *      Baud rate = 115200
         *      Data bits = 8
         *      Stop bits = 1
         *      Parity = NO_PARITY
         *      Flow Control = false
         *      Low latency mode = false
         *      Writable = false
         * 
         * @param device: OS path of the device
         * @param config: Port configuration settings
         * 
         * @return True on success
         */
        bool SerialOpen(const std::string &device, Config config = Config());

        /**
         * Close serial port if open
         */
        void serialClose();

        /**
         * // TODO: Put description
         * 
         */
        Result readBytes(std::vector<uint8_t> &output, size_t maxBytes, int32_t timeout);

        /**
         * // TODO: Put description
         * 
         */
        int32_t serialWrite(const std::vector<uint8_t> &input);

        /**
         * Get the most recent error message.
         */
        std::string errorMsg() const
        {
            return m_ErrorMessage;
        }

    private:
        /**
         * Attempts to put serial port in low latency mode.
         * 
         * @return True on success
         */
        bool _setLowLatencyMode();

        /**
         * Parses integer and enumerated baud rates into enumerated baud rates.
         * 
         * @param baudRate: The baud rate (either integer or enumerated)
         * 
         * @return The enumerated baud rate, -1 if invalid.
         */
        int32_t _parseBaudRate(int32_t baudRate) const;
    };

} // namespace serial_util

#endif // MOSAIC_SERIAL_H_