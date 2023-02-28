# [slam_toolbox](https://wiki.ros.org/slam_toolbox)

Slam Toolbox is a set of tools and capabilities for 2D SLAM built by
[Steve Macenski](https://www.linkedin.com/in/steven-macenski-41a985101) while at
[Simbe Robotics](https://www.simberobotics.com/) and in his free time.

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [eigen](https://wiki.ros.org/eigen3)
- [interactive_markers](https://wiki.ros.org/interactive_markers)
- [libceres-dev](https://index.ros.org/d/libceres-dev)
- [libg2o](https://index.ros.org/d/libg2o)
- [liblapack-dev](https://index.ros.org/d/liblapack-dev/)
- [libqt5-core](https://index.ros.org/d/libqt5-core/)
- [libqt5-widgets](https://index.ros.org/d/libqt5-widgets/)
- [map_server](https://wiki.ros.org/map_server)
- [message_filters](https://wiki.ros.org/message_filters)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [pluginlib](https://wiki.ros.org/pluginlib)
- [qtbase5-dev](https://index.ros.org/d/qtbase5-dev/)
- [rosconsole](https://wiki.ros.org/rosconsole)
- [sensor_msgs](https://wiki.ros.org/sensor_msgs)
- [slam_toolbox_msgs](https://github.com/SteveMacenski/slam_toolbox/tree/noetic-devel/slam_toolbox_msgs)
- [sparse_bundle_adjustment](https://github.com/ros-perception/sparse_bundle_adjustment)
- [std_msgs](https://wiki.ros.org/std_msgs)
- [std_srvs](https://wiki.ros.org/std_srvs)
- [suitesparse](https://wiki.ros.org/suitesparse)
- [tbb](https://index.ros.org/d/tbb/)
- [tf](https://wiki.ros.org/tf)
- [tf2](https://wiki.ros.org/tf2)
- [tf2_geometry_msgs](https://wiki.ros.org/tf2_geometry_msgs)
- [tf2_ros](https://wiki.ros.org/tf2_ros)
- [visualization_msgs](https://wiki.ros.org/visualization_msgs)

### Parameters

See more in
https://github.com/SteveMacenski/slam_toolbox/tree/noetic-devel#configuration.

### Subscribes

- scan
  ([LaserScan.msg](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))
- tf (N/A)
  - odom_frame - base_frame

### Publishes

- map
  ([OccupancyGrid.msg](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))

### Services

See more in
https://github.com/SteveMacenski/slam_toolbox/tree/noetic-devel#exposed-services.

### Actions

None.

## Usage

### Installation

```sh
# Upgrade the installed packages on the system
sudo apt update
sudo apt dist-upgrade

# Install ROS package
sudo apt install ros-noetic-slam-toolbox
```

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf run_urg_node_ust_10lx.launch
```
