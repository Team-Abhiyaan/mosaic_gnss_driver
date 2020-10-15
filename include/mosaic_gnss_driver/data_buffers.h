#ifndef MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
#define MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H

// #include <mutex>

#include <sensor_msgs/NavSatFix.h>

#include <ros/publisher.h>

namespace mosaic_gnss_driver {
    struct DataBuffers {
        struct NavSatFix {
            // std::mutex mutex; // NOTE: We need a mutex if the publishers run on another thread
            sensor_msgs::NavSatFix::Ptr ptr;
            ros::Publisher pub;
        } nav_sat_fix;
    };
}

#endif //MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
