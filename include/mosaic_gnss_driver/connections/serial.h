#ifndef MOSAIC_GNSS_DRIVER_SERIAL_H
#define MOSAIC_GNSS_DRIVER_SERIAL_H

#include <mosaic_gnss_driver/connections/connection.h>
#include <mosaic_utils/serial.h>
#include <ros/ros.h>

namespace mosaic_gnss_driver::connections
{
    /**
     * Represents a connection via Serial port
     */
    class Serial : public Connection
    {
        static constexpr const char *const type = "Serial";

    protected:
        int32_t baud{9600};
        serial_util::SerialPort serial_port;

    public:
        using Connection::Connection; // Use superclass constructor

        /**
         * Attempts to connect to the module via a serial port
         * 
         * @param device: A filehandle. eg: /dev/TTYUSB0
         * @param opts: Configuration options
         * 
         * @return True if successful, false otherwise
         */
        bool connect(const std::string &device, const Options &opts = {}) override;

        void disconnect() override;

        ReadResult read() override;

        /**
         * Sets the serial baud rate; should be called before configuring a serial connection.
         * 
         * @param baud_rate: The serial baud rate.
         */
        void setSerialBaud(int32_t baud_rate)
        {
            baud = baud_rate;
            ROS_INFO("Serial baud rate: %d", baud_rate);
        }

        bool write(const std::string &command) override;

        /// Destructor
        ~Serial();
    };
} // namespace mosaic_gnss_driver::connections

#endif //MOSAIC_GNSS_DRIVER_SERIAL_H
