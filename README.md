# mosaic_gnss_driver

A ROS driver for the Septentrio mosaic development kit GNSS/GPS module

## Links

[Module details](https://shop.septentrio.com/en/shop/mosaictm-development-kit)

[Septentrio request for info](https://customersupport.septentrio.com/s/case/500f300001R3MOlAAN/configuration-setup-for-the-mosaic-dev-kit)

[ROSCon 2012 - Writing Hardware Drivers](https://www.youtube.com/watch?v=pagC2WXT1x0)

[Slides for the above](https://docs.google.com/presentation/d/13yyOB5CXOzpvMa0_wYxDvNzjb_9dfMjDuVo-CvBcoRw/edit#slide=id.p)

## General workflow of writing hardware drivers

**As from the ROSCon video**

* Starting driver

[Existing ROS drivers for pose estimation components - ROS Wiki](https://wiki.ros.org/Sensors#Pose_Estimation_.28GPS.2FIMU.29)

[NMEA Navsat driver - GitHub *Found this really helpful](https://github.com/ros-drivers/nmea_navsat_driver)

* Writing a standalone library

* ROS wrapper

* Dynamic reconfigure

* Diagnostics