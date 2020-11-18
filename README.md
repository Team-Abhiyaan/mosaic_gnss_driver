# mosaic_gnss_driver

## Description

ROS Driver Package for the [Septentrio Mosaic development kit](https://shop.septentrio.com/en/shop/mosaictm-development-kit)

## Current status (master branch)

- Core library in progress
- Communication with the module is functional for all modes of connection, viz SERIAL, TCP, UDP and PCAP
- Parsers for SBF and NMEA in progress

## Navigation

- [Build workspace](#build-workspace)

- [Other subcommands](#other-commands)
  <br>

- [Wireshark Installation](#wireshark-installation-for-capturing-traffic-from-module)

- [C++ libraries for data playback](#install-c-libraries-for-traffic-playback)

---

## Install dependencies

- C++ library for handling pcap files

```bash
sudo apt install libpcap-dev
```

## Build workspace

- Clone this repository

- Build

```bash
make build
```

### Other commands

- Cleanup

```bash
make clean
```

- Generate documentation

```bash
make gendoc
```

- Run unit tests

```bash
make test
```

## Notes

### Wireshark installation for capturing traffic from module

- Install wireshark

```bash
sudo add-apt-repository ppa:wireshark-dev/stable
sudo apt update
sudo apt -y install wireshark
```

> When asked whether to allow non-superusers to capture packets, select **<Yes\>** and finish installation.

- Configure wireshark

```bash
sudo usermod -aG wireshark $USER
```

**NOTE : Relogin may be required for the changes to take effect**

- Change `dumpcap` binary file permissions

```bash
sudo chgrp wireshark /usr/bin/dumpcap
sudo chmod 750 /usr/bin/dumpcap
sudo setcap cap_net_raw,cap_net_admin=eip /usr/bin/dumpcap
```

- Verify configuration

```bash
sudo getcap /usr/bin/dumpcap
```

Expected output : `/usr/bin/dumpcap = cap_net_admin,cap_net_raw+eip`
<br>

### Install C++ libraries for traffic playback

```bash
sudo apt install libpcap-dev
```
### ROS Wrapper

1. **ROS Parameters**
    - `frame_id`: ROS TF frame to place in the header of published messages.
        - Default: gps_link
    - `parser`: `sbf` to set the parser to use the sbf blocks sent by the module.
            	`nmea` to set the parser to use the NMEA Sentence sent by the module.  	
    - `conn`: ` type ` to set the type of connection used to connect with the module.
	 Set type as:
           -`serial` for serial connection type. 
           -`tcp` for tcp connection protocol.
           -`udp` for udp connection protocol.
           -`pcap` to read packets from a pcap file.
    - `device`: Used to set the Address for TCP/UDP connection 
         - Default: `192.168.1.101`
          For pcap ,this is used to give the path of the pcap file.
          For serial connection, this is used to set the port.
         - Default:`/dev/ACM0`
    - `pub_nmea_msg`: `true` to publish NMEA Sentences sent by the mosaic module.
         - Default: `false`	    
2. **Published ROS Topics**
    - `/nav_sat_fix` *(sensor_msgs/NavSatFix)*
          - **Note**:  GPSFix messages will always be published regardless of what parser type is used.        
    - `/pose` *(geometry_msgs/PoseWithCovarianceStamped)*: From SBF Blocks
    - `/velocity` *(geometry_msgs/TwistWithCovarianceStamped)*: Published by both SBF and NMEA Parser.
	 - **Note**:  NMEA Parser can only publish linear velocity in x,y directions due to less information from NMEA Sentence.       
    - `/nmea_sentence` *(nmea_msgs/Sentence)*: Publishes NMEA Sentence sent by the module if "pub_nmea_msg" parameter is set to true.
    - `/time_reference` *(sensor_msgs/TimeReference)*: Publishes Unix Epoch Time given by NMEA data. 

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

- Starting driver

  - [Existing ROS drivers for pose estimation components - ROS Wiki](https://wiki.ros.org/Sensors#Pose_Estimation_.28GPS.2FIMU.29)

- Writing a standalone library

- ROS wrapper

- Dynamic reconfigure

- Diagnostics
