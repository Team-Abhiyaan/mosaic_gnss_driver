# mosaic_gnss_driver

[![ros-ci](https://github.com/Team-Abhiyaan/mosaic_gnss_driver/actions/workflows/ros-ci.yml/badge.svg)](https://github.com/Team-Abhiyaan/mosaic_gnss_driver/actions/workflows/ros-ci.yml)

Lightweight ROS Driver Package for the [Septentrio Mosaic development kit](https://shop.septentrio.com/en/shop/mosaictm-development-kit)

---

## Installation

```bash
git clone https://github.com/Team-Abhiyaan/mosaic_gnss_driver.git
rosdep install --from-paths mosaic_gnss_driver
```

## Usage

- Setup a stream from the mosaic webinterface
- Edit parameters in the [config/driver.yml](config/driver.yml) configuration file
- Launch driver

```bash
roslaunch mosaic_gnss_driver mosaic_gnss.launch
```

1. **ROS Parameters**

   - `connection`: Type of connection. Supported ones are tcp, udp, serial and pcap.
   - `device`:
     - for IP (TCP/UDP): A host:port specification. eg: 192.168.3.1:9999
     - for Serial: A filehandle. eg: /dev/TTYUSB0
     - for PCAP: Path to pcap file relative to package.
   - `serial/baudrate`: The Baud rate in case of serial connection.
   - `stream`: Type of stream. Supported ones are sbf, nmea.
   - `frame_id`: ROS TF frame to place in the header of published messages.
   - `use_gnss_time`: if set to true, header's time field will be constructed from incoming data.
   - `sbf_pvt_type`: `"type"` set pvt type used by module `geodetic` or `cartesian`.

2. **Published ROS Topics**
   - `/navsatfix` _(sensor_msgs/NavSatFix)_<br/>
     - **Note**: Published by NMEA and SBF Geodetic streams.
   - `/pose` _(geometry_msgs/PoseWithCovarianceStamped)_: Only from SBF Cartesian streams.
   - `/velocity` _(geometry_msgs/TwistWithCovarianceStamped)_:<br/>
     - **Note**: NMEA Parser can only publish linear velocity in x,y directions due to less information from NMEA Sentence.
   - `/time_reference` _(sensor_msgs/TimeReference)_: Publishes Unix Epoch Time given by NMEA data.
   - `/nmea_sentence` _(nmea_msgs/Sentence)_: Publishes NMEA Sentence sent by the module if "pub_nmea_msg" parameter is set to true.
