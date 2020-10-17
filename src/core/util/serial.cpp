#include <mosaic_utils/serial.h>

#include <boost/lexical_cast.hpp>

// Linux headers
#include <linux/serial.h> // For latency control and low-latency mode
#include <sys/ioctl.h>    // I/O Interface for device control
#include <termios.h>      // Contains POSIX terminal control definitions
#include <unistd.h>       // write(), read(), close()
#include <fcntl.h>        // Contains file controls like O_RDWR, O_RDONLY
#include <errno.h>        // Error integer and strerror() function
#include <poll.h>         // For event handling on file descriptors and polling ports

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

    bool SerialPort::serialOpen(const std::string &device, Config config)
    {
        serialClose();

        // Validate baud rate
        int32_t baud = _parseBaudRate(config.m_BaudRate);
        if (baud == -1)
        {
            m_ErrorMessage = "Invalid baud rate: " + boost::lexical_cast<std::string>(config.m_BaudRate);

            return false;
        }

        // Validate stop bits
        if (config.m_StopBits != 1 && config.m_StopBits != 2)
        {
            m_ErrorMessage = "Invalid stop bits: " + boost::lexical_cast<std::string>(config.m_StopBits);

            return false;
        }

        // Validate data bits
        if (config.m_DataBits != 7 && config.m_DataBits != 8)
        {
            m_ErrorMessage = "Invalid data bits: " + boost::lexical_cast<std::string>(config.m_DataBits);

            return false;
        }

        // Validate parity checking
        if (config.m_Parity != Config::NO_PARITY && config.m_Parity != Config::EVEN_PARITY &&
            config.m_Parity != Config::ODD_PARITY)
        {
            m_ErrorMessage = "Invalid parity mode";

            return false;
        }

        // Open serial port, handle error if any
        m_Fd = open(device.c_str(), config.m_Writable ? O_RDWR : O_RDONLY);
        if (m_Fd < 0)
        {
            m_ErrorMessage = "Error opening serial port <" + device + ">: " + strerror(errno);

            return false;
        }

        struct termios term;

        // Read in existing settings, and handle any error
        if (tcgetattr(m_Fd, &term) < 0)
        {
            m_ErrorMessage = "Unable to set serial attributes <" + device + ">: " + strerror(errno);
            serialClose();

            return false;
        }

        // Apply fresh configuration flags
        cfmakeraw(&term);

        if (config.m_StopBits == 2)
        {
            term.c_cflag |= CSTOPB;
        } else
        {
            term.c_cflag &= ~CSTOPB;
        }

        switch (config.m_Parity)
        {
            case Config::EVEN_PARITY:
                term.c_cflag |= PARENB;
                term.c_cflag &= ~PARODD;
                break;

            case Config::ODD_PARITY:
                term.c_cflag |= PARENB;
                term.c_cflag |= PARODD;
                break;

            case Config::NO_PARITY:
                term.c_cflag &= ~PARENB;
                term.c_cflag &= ~PARODD;
                break;

            default:
                // will never reach here, case taken care of during validation
                break;
        }

        if (config.m_DataBits == 8)
        {
            term.c_cflag &= ~CSIZE;
            term.c_cflag |= CS8;
        } else
        {
            term.c_cflag &= ~CSIZE;
            term.c_cflag |= CS7;
        }

        if (cfsetspeed(&term, config.m_BaudRate) < 0)
        {
            m_ErrorMessage = "Invalid baud rate: " + boost::lexical_cast<std::string>(config.m_BaudRate);
            serialClose();

            return false;
        }

        if (tcsetattr(m_Fd, TCSAFLUSH, &term) < 0)
        {
            m_ErrorMessage = "Unable to set serial port attributes <" + device + ">: " + strerror(errno);
            serialClose();

            return false;
        }

        if (config.m_LowLatencyMode && !_setLowLatencyMode())
        {
            // Error taken care of in serialClose() method
            serialClose();

            return false;
        }

        return true;
    }

    SerialPort::ReadResult SerialPort::readBytes(std::vector<uint8_t> &output, size_t maxBytes, int32_t timeout)
    {
        // Check if device is open
        if (m_Fd < 0)
        {
            m_ErrorMessage = "Device not open";
            return ERROR;
        }

        struct pollfd fds[1];
        fds[0].fd = m_Fd;       // Specify file descriptor
        fds[0].events = POLLIN; // Specify event

        // Wait till file descriptor is ready for I/O
        //
        // Upon successful completion, poll() shall return a non-negative value. A positive value indicates the
        // total number of file descriptors that have been selected (that is, file descriptors for which the
        // revents member is non-zero)
        int pollReturn = poll(fds, 1, timeout);

        if (pollReturn == 0) // Timeout
        {
            m_ErrorMessage = "Timed out while waiting for data";
            return TIMEOUT;
        } else if (pollReturn < 0) // Failure
        {
            int errorNumber = errno;
            switch (errorNumber)
            {
                case EINTR:
                    return INTERRUPTED;
                default:
                    m_ErrorMessage = "Error polling serial port: " + std::string(strerror(errno));
                    return ERROR;
            }
        }

        size_t toRead = maxBytes; // number of bytes to be read
        if (toRead <= 0)
        {
            int bytes;
            // ioctl - control device
            // FIONREAD specifies it to get the number of bytes that are immediately available for reading
            ioctl(m_Fd, FIONREAD, &bytes);
            if (bytes < 0)
            {
                m_ErrorMessage =
                        "Error getting number of available bytes from serial port: " + std::string(strerror(errno));
                return ERROR;
            }

            toRead = static_cast<size_t>(bytes);
        }

        size_t outputSize = output.size();
        output.resize(outputSize + toRead);

        int result = read(m_Fd, output.data() + outputSize, toRead); // read data, returns number of bytes read

        if (result > 0)
        {
            output.resize(outputSize + result);
        } else
        {
            output.resize(outputSize);
        }

        if (result > 0) // Success
        {
            return SUCCESS;
        } else if (result == 0) // Interruption
        {
            return INTERRUPTED;
        } else
        {
            int errorNumber = errno;
            switch (errorNumber)
            {
                case EINTR:
                    return INTERRUPTED;

                default:
                    m_ErrorMessage = "Error reading from serial port: " + std::string(strerror(errno));
                    return ERROR;
            }
        }
    }

    int32_t SerialPort::serialWrite(const std::vector<uint8_t> &input)
    {
        int32_t result = write(m_Fd, input.data(), input.size());

        if (result < 0)
        {
            m_ErrorMessage = "Failed to write to serial port: " + std::string(strerror(errno));
        }

        return result;
    }

    bool SerialPort::_setLowLatencyMode()
    {
        if (m_Fd < 0)
        {
            m_ErrorMessage = "Device not open.";
            return false;
        }

        struct serial_struct serialInfo;

        if (ioctl(m_Fd, TIOCGSERIAL, &serialInfo) < 0)
        {
            m_ErrorMessage =
                    "Failed to set low latency mode.  Cannot get serial configuration: " + std::string(strerror(errno));
            return false;
        }

        serialInfo.flags |= ASYNC_LOW_LATENCY;

        if (ioctl(m_Fd, TIOCSSERIAL, &serialInfo) < 0)
        {
            m_ErrorMessage =
                    "Failed to set low latency mode.  Cannot set serial configuration: " + std::string(strerror(errno));
            return false;
        }

        return true;
    }

    int32_t SerialPort::_parseBaudRate(int32_t baudRate) const
    {
        int32_t value = -1;

        if (baudRate == B50 || baudRate == 50)
        {
            value = B50;
        } else if (baudRate == B75 || baudRate == 75)
        {
            value = B75;
        } else if (baudRate == B110 || baudRate == 110)
        {
            value = B110;
        } else if (baudRate == B134 || baudRate == 134)
        {
            value = B134;
        } else if (baudRate == B150 || baudRate == 150)
        {
            value = B150;
        } else if (baudRate == B200 || baudRate == 200)
        {
            value = B200;
        } else if (baudRate == B300 || baudRate == 300)
        {
            value = B300;
        } else if (baudRate == B600 || baudRate == 600)
        {
            value = B600;
        } else if (baudRate == B1200 || baudRate == 1200)
        {
            value = B1200;
        } else if (baudRate == B1800 || baudRate == 1800)
        {
            value = B1800;
        } else if (baudRate == B2400 || baudRate == 2400)
        {
            value = B2400;
        } else if (baudRate == B4800 || baudRate == 4800)
        {
            value = B4800;
        } else if (baudRate == B9600 || baudRate == 9600)
        {
            value = B9600;
        } else if (baudRate == B19200 || baudRate == 19200)
        {
            value = B19200;
        } else if (baudRate == B38400 || baudRate == 38400)
        {
            value = B38400;
        } else if (baudRate == B57600 || baudRate == 57600)
        {
            value = B57600;
        } else if (baudRate == B115200 || baudRate == 115200)
        {
            value = B115200;
        } else if (baudRate == B230400 || baudRate == 230400)
        {
            value = B230400;
        } else if (baudRate == B460800 || baudRate == 460800)
        {
            value = B460800;
        } else if (baudRate == B576000 || baudRate == 576000)
        {
            value = B576000;
        } else if (baudRate == B921600 || baudRate == 921600)
        {
            value = B921600;
        } else if (baudRate == B1000000 || baudRate == 1000000)
        {
            value = B1000000;
        } else if (baudRate == B1152000 || baudRate == 1152000)
        {
            value = B1152000;
        } else if (baudRate == B1500000 || baudRate == 1500000)
        {
            value = B1500000;
        } else if (baudRate == B2000000 || baudRate == 2000000)
        {
            value = B2000000;
        } else if (baudRate == B2500000 || baudRate == 2500000)
        {
            value = B2500000;
        } else if (baudRate == B3000000 || baudRate == 3000000)
        {
            value = B3000000;
        } else if (baudRate == B3500000 || baudRate == 3500000)
        {
            value = B3500000;
        } else if (baudRate == B4000000 || baudRate == 4000000)
        {
            value = B4000000;
        }

        return value;
    }

} // namespace serial_util
