# [hector_mapping](https://wiki.ros.org/hector_mapping)

SLAM approach that can be used without odometry as well as on platforms that
exhibit roll/pitch motion (of the sensor, the platform or both). It leverages
the high update rate of modern LIDAR systems like the
[Hokuyo UTM-30LX](https://www.hokuyo-usa.com/products/lidar-obstacle-detection/utm-30lx)
and provides 2D pose estimates at scan rate of the sensors (40Hz for the
UTM-30LX). While the system does not provide explicit loop closing ability, it
is sufficiently accurate for many real world scenarios.

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [boost](https://index.ros.org/d/boost/)
- [eigen](https://wiki.ros.org/eigen3)
- [laser_geometry](https://wiki.ros.org/laser_geometry)
- [message_filters](https://wiki.ros.org/message_filters)
- [message_generation](https://wiki.ros.org/message_generation)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [tf](https://wiki.ros.org/tf)
- [visualization_msgs](https://wiki.ros.org/visualization_msgs)

### Parameters

See more in https://wiki.ros.org/hector_mapping#Parameters.

### Subscribes

- scan
  ([LaserScan.msg](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))
- tf (N/A)
  - base_frame > laser

### Publishes

- map
  ([OccupancyGrid.msg](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))
- poseupdate
  ([PoseWithCovarianceStamped.msg](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
- slam_out_pose
  ([PoseStamped.msg](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))

### Services

See more in https://wiki.ros.org/hector_mapping#Services.

### Actions

None.

## Usage

### Installation

```sh
# Upgrade the installed packages on the system
sudo apt update
sudo apt dist-upgrade

# Install ROS package
sudo apt install ros-noetic-hector-slam
```

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf run_hector_mapping.launch
```
