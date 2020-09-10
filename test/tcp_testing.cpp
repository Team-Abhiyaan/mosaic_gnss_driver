#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <gtest/gtest.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>

// TODO: Add TCP test suite


int main(int argc, char **argv)
{
	ros::init(argc, argv, "tcpTestSuite", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}

