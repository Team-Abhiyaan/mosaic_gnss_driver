# mosaic_gnss_driver

A ROS driver for the Septentrio mosaic development kit GNSS/GPS module

---

**Please add any info that you think might be useful**

---
## Links

[**Module details**](https://shop.septentrio.com/en/shop/mosaictm-development-kit)

[**Google drive link**](https://drive.google.com/drive/folders/14KQpB4tbFVY6TrVSzioFhG_bZOaW4NAf?usp=sharing)

[Septentrio request for info](https://customersupport.septentrio.com/s/case/500f300001R3MOlAAN/configuration-setup-for-the-mosaic-dev-kit)

[ROSCon 2012 - Writing Hardware Drivers](https://www.youtube.com/watch?v=pagC2WXT1x0)

[Slides for the above](https://docs.google.com/presentation/d/13yyOB5CXOzpvMa0_wYxDvNzjb_9dfMjDuVo-CvBcoRw/edit#slide=id.p)

---

[4 part tutorial in writing ROS driver packages](https://roboticsbackend.com/create-a-ros-driver-package-introduction-what-is-a-ros-wrapper-1-4/)

### Other open source drivers for reference

[NMEA Navsat driver - GitHub : Python](https://github.com/ros-drivers/nmea_navsat_driver)

[Novatel GPS Driver - GitHub : C++](https://github.com/swri-robotics/novatel_gps_driver)

## General workflow of writing hardware drivers

**As from the ROSCon video**

* Starting driver

  * [Existing ROS drivers for pose estimation components - ROS Wiki](https://wiki.ros.org/Sensors#Pose_Estimation_.28GPS.2FIMU.29)


* Writing a standalone library

* ROS wrapper

* Dynamic reconfigure

* Diagnostics
