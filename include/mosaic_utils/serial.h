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
         * @param baudRate: Baud rate - maximum amount of bits transferable per second
         * @param dataBits: Number of data bits
         * @param stopBits: Number of stop bits
         * @param parity: Type of parity checking
         * @param flowControl: Flow control - Ability of device to tell another device to stop sending data for a while
         * @param lowLatencyMode: Set to low latency mode
         * @param writable: Make port writable, used to configure the module
         * 
         * Low-latency mode
         * ----------------
         * Serial ports on linux are "wrapped" into unix-style terminal constructs, which hits you with 1 tick lag, i.e. 10ms.
         * On a PC, you can go hardcore and talk to standard serial ports directly, unbind linux driver from serial port hardware 
         * and control the port via inb/outb to port registers.
         * The downside is you don't get interrupts when data arrives and you have to poll the register. often. Hence not preferred.
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
        enum ReadResult
        {
            SUCCESS,
            TIMEOUT,
            INTERRUPTED,
            ERROR
        };

        /// File descriptor representing the serial port
        int m_Fd;
        /// For latest error message
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
         * Read bytes from the serial port
         * 
         * Appends upto maxBytes into the provided vector. If maxBytes is 0, it reads all available bytes.
         * 
         * @param output: The output buffer to which read bytes are to be saved
         * @param maxBytes: The maximum number of bytes to read. If set to 0, all available bytes are read.
         * @param timeout: The maximum time to block in milliseconds
         * 
         * @return A code indicating result of operation (SUCCESS, TIMEOUT, INTERRUPTED, ERROR)
         */
        ReadResult readBytes(std::vector<uint8_t> &output, size_t maxBytes, int32_t timeout);

        /**
         * Configure the device
         * 
         * @return Number of bytes written on success. On error, -1 is returned, and errno is set to indicate the cause of the error.
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