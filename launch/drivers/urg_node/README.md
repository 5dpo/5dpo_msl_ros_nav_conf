# urg_node

ROS node to provide access to SCIP 2.0-compliant Hokuyo laser range finders
(including 04LX), fully REP-138 compliant, and compatible with
Ethernet/MultiEcho Hokuyos.

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [diagnostic_updater](https://wiki.ros.org/diagnostic_updater)
- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure)
- [laser_proc](https://wiki.ros.org/laser_proc)
- [message_generation](https://wiki.ros.org/message_generation)
- [nodelet](https://wiki.ros.org/nodelet)
- [rosconsole](https://wiki.ros.org/rosconsole)
- [sensor_msgs](https://wiki.ros.org/sensor_msgs)
- [std_msgs](https://wiki.ros.org/std_msgs)
- [std_srvs](https://wiki.ros.org/std_srvs)
- [tf](https://wiki.ros.org/tf)
- [urg_c](https://wiki.ros.org/urg_c)
- [message_runtime](https://wiki.ros.org/message_runtime) (_runtime_)
- [xacro](https://wiki.ros.org/xacro) (_runtime_)
- [roslint](https://wiki.ros.org/roslint) (_test_)
- [roslaunch](https://wiki.ros.org/roslaunch) (_test_)

### Parameters

- ip_address (`string = ""`): ip address of the Hokuyo laser with an Ethernet
  connection (UST-10LX: `"192.168.0.10"`)
- ip_port (`int = 10940`): port (UST-10LX: `10940`)
- serial_port (`string = "/dev/ttyACM0"`): serial port name
- serial_baud (`int = 115200`): baudrate of the serial connection (bits/s)
- calibrate_time (`bool = false`): calibration of the time offset between the
  laser and the computer on startup for producing accurate time stamps on scans
- synchronize_time (`bool = false`): hardware clock synchronization
- publish_intensity (`bool = true`): return the scan intensity values
- publish_multiecho (`bool = false`): multi-echo functionality to help cancel
  noise in the measurement field if supported by the hardware (multiecho laser
  scanners)
- error_limit (`int = 4`)
- diagnostics_tolerance (`double = 0.05`)
- diagnostics_window_time (`double = 5.0`)
- get_detailed_status (`bool = false`)

### Subscribes

None.

### Publishes

- laser_status
  ([`Status.msg`](https://docs.ros.org/en/noetic/api/urg_node/html/msg/Status.html))
- scan
  ([`LaserScan.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))

### Services

- update_laser_status
  ([`Trigger.srv`](https://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html))

### Actions

None.

## Usage

### Installation

```sh
# Upgrade the installed packages on the system
sudo apt update
sudo apt dist-upgrade

# Install ROS package
sudo apt install ros-noetic-urg-node
```

### Setup

1. Set a static IP address for the Ethernet network interface
   1. Show Applications > Settings > Network > Wired Settings
   2. See IPv4 Settings
      - Address: `192.168.0.1`
      - Mask: `255.255.255.0`
2. Launch the node

### Launch

**[UST-10LX](https://www.hokuyo-usa.com/products/lidar-obstacle-detection/ust-10lx)**

```sh
roslaunch sdpo_msl_ros_nav_conf run_urg_node_ust_10lx.launch
```
