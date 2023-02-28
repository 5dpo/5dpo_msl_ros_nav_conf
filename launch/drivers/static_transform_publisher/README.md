# static_transform_publisher

Publish a static coordinate transform to tf using an x/y/z offset in meters and
yaw/pitch/roll in radians (yaw is rotation about Z, pitch is rotation about Y,
and roll is rotation about X).
Or publish a static coordinate transform to tf using an x/y/z offset in meters
and quaternion. The period, in milliseconds, specifies how often to send a
transform (100ms usually is a good value).

(source:
[https://wiki.ros.org/tf#static_transform_publisher](https://wiki.ros.org/tf#static_transform_publisher))

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [tf](https://wiki.ros.org/tf)

## Usage

### Run

**Yaw/Pitch/Roll**

```sh
rosrun tf static_transform_publisher x y z raw yaw pitch roll frame_id child_frame_id period_in_ms
```

**Quaternion**

```sh
rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_m
```

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf run_static_transform_publisher.launch
```
