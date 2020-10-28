#ifndef MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
#define MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H

// #include <mutex>

#include <sensor_msgs/NavSatFix.h>

#ifndef MOSAIC_GNSS_CORE_ONLY

#include <ros/ros.h>
#include <ros/publisher.h>

#else
#include <boost/make_shared.hpp>
#endif

namespace mosaic_gnss_driver
{
    template<typename msg_type>
    struct Buffer
    {
    // public:
        using ptr_t = std::unique_ptr<msg_type>;
    // private:
        // std::mutex mutex; // NOTE: We need a mutex if the publishers run on another thread
        ptr_t ptr;
        using shared_ptr_t = typename msg_type::Ptr;
    // public:
        ptr_t get_new_ptr()
        {
            // TODO: Reuse old ptr if not sent yet ?
            return std::make_unique<msg_type>();
        }

        void set_ptr(ptr_t new_ptr)
        {
            ptr = std::move(new_ptr);
        }

// We do this to compile core library without ros.
// The core library never creates an object of this type, it only calls the above functions.`
#ifndef MOSAIC_GNSS_CORE_ONLY
        ros::Publisher pub;

        void init(ros::NodeHandle &nh, const char *topic, size_t queue, bool latch = false)
        {
            pub = nh.advertise<msg_type>(topic, queue, latch);
        }

        void publish()
        {
            // std::lock_guard<std::mutex> lock(mutex);
            if (!ptr)
            {
                ROS_WARN("Not enough msg");
            } else
            {
                shared_ptr_t shr = std::move(ptr);
                pub.publish(shr);
            }
        }

#endif // MOSAIC_GNSS_CORE_ONLY
    };

    struct DataBuffers
    {
        Buffer<sensor_msgs::NavSatFix> nav_sat_fix;
    };
}

#endif //MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
