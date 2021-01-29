#include <gtest/gtest.h>

#include <ros/ros.h>

class AnyMessage
{
};
typedef boost::shared_ptr<AnyMessage> AnyMessagePtr;
typedef boost::shared_ptr<AnyMessage const> AnyMessageConstPtr;

struct AnyHelper
{
    AnyHelper() : count(0)
    {
    }

    void cb(const AnyMessageConstPtr &msg)
    {
        count++;
    }

    uint32_t count;
};

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_subs");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}