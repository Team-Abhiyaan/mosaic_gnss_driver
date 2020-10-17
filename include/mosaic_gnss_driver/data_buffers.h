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
        // std::mutex mutex; // NOTE: We need a mutex if the publishers run on another thread
        typename msg_type::Ptr ptr;

        auto get_new_ptr()
        {
            // TODO: Reuse old ptr if not sent yet ?
            return boost::make_shared<msg_type>();
        }

        void set_ptr(typename msg_type::Ptr new_ptr)
        {
            ptr = new_ptr;
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
                pub.publish(ptr);
                ptr.reset();
            }
        }

#endif // MOSAIC_GNSS_CORE_ONLY
    };

    struct DataBuffers
    {
        Buffer<sensor_msgs::NavSatFix> nav_sat_fix;
        void aa() {
        }
    };
}

#endif //MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
