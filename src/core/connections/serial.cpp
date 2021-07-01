#include <mosaic_gnss_driver/connections/serial.h>

using namespace mosaic_gnss_driver::connections;

bool Serial::write(const std::string& command)
{
    std::vector<uint8_t> bytes(command.begin(), command.end());
    int32_t written = serial_port.serialWrite(bytes);
    if (written != static_cast<int32_t>(bytes.size()))
    {
        ROS_ERROR("Failed to send command: %s", command.c_str());
        return false;
    }
    return true;
}

void Serial::disconnect() { serial_port.serialClose(); }

Serial::~Serial() { Serial::disconnect(); }

bool Serial::connect(const std::string& device, const Options& opts)
{
    serial_util::Config config;
    config.m_BaudRate = baud;
    config.m_Parity = serial_util::Config::NO_PARITY;
    config.m_FlowControl = false;
    config.m_DataBits = 8;
    config.m_StopBits = 1;
    config.m_LowLatencyMode = false;
    config.m_Writable = true;

    connected = serial_port.serialOpen(device, config);

    if (!is_connected())
    {
        ROS_ERROR("%s", serial_port.errorMsg().c_str());
    }

    return connected;
}

ReadResult Serial::read()
{
    if (!is_connected())
        return READ_ERROR;

    serial_util::SerialPort::ReadResult result = serial_port.readBytes(buffer, 0, 1000);

    if (result == serial_util::SerialPort::ERROR)
    {
        ROS_ERROR("%s", serial_port.errorMsg().c_str());
        return READ_ERROR;
    } else if (result == serial_util::SerialPort::TIMEOUT)
    {
        ROS_ERROR("Timed out waiting for serial device.");
        return READ_TIMEOUT;
    } else if (result == serial_util::SerialPort::INTERRUPTED)
    {
        ROS_ERROR("Interrupted during read from serial device.");
        return READ_INTERRUPTED;
    }

    return READ_SUCCESS;
}
