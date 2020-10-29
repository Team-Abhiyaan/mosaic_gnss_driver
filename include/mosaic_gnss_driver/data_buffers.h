#ifndef MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
#define MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H

// #include <mutex>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#ifdef __JETBRAINS_IDE__
#undef MOSAIC_GNSS_CORE_ONLY
#endif

#ifndef MOSAIC_GNSS_CORE_ONLY

#include <ros/ros.h>
#include <ros/publisher.h>

#endif

namespace mosaic_gnss_driver
{
    template<typename msg_type>
    struct Buffer
    {
    public:
        using ptr_t = std::unique_ptr<msg_type>;
    private:
        // std::mutex mutex; // NOTE: We need a mutex if the publishers run on another thread
        ptr_t ptr;
    public:

        /**
         * Returns a pointer to an instance of the message. This should be filled and passed to set_ptr
         * Not guaranteed to be zero initialized, i.e. implementation may reuse previous unsent message.
         *
         * @return message pointer
         */
        ptr_t get_new_ptr()
        {
            // TODO: Reuse old ptr if not sent yet ?
            return std::make_unique<msg_type>();
        }

        void set_ptr(ptr_t new_ptr)
        {
            ptr = std::move(new_ptr);
        }

        /**
         * Get the stored message.
         * After this the buffer will be empty.
         *
         * @return message, may be nullptr
         */
        ptr_t get()
        {
            return std::move(ptr);
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
                // TODO: Check if publisher ready
                typename msg_type::Ptr shared_ptr = std::move(ptr);
                pub.publish(shared_ptr);
            }
        }

#endif // MOSAIC_GNSS_CORE_ONLY
    };

    struct DataBuffers
    {
        Buffer<sensor_msgs::NavSatFix> nav_sat_fix;
        Buffer<geometry_msgs::TwistWithCovarianceStamped> velocity;
    };
}

#endif //MOSAIC_GNSS_DRIVER_DATA_BUFFERS_H
