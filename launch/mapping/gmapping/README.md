# [gmapping](https://wiki.ros.org/gmapping)

ROS wrapper for OpenSlam's Gmapping. The gmapping package provides laser-based
SLAM (Simultaneous Localization and Mapping), as a ROS node called
`slam_gmapping`. Using `slam_gmapping`, you can create a 2-D occupancy grid map
(like a building floorplan) from laser and pose data collected by a mobile
robot.

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [nodelet](https://wiki.ros.org/nodelet)
- [openslam_gmapping](https://wiki.ros.org/openslam_gmapping)
- [rostest](https://wiki.ros.org/rostest)
- [tf](https://wiki.ros.org/tf)

### Parameters

- base_frame (`std::string = "base_link"`): tf frame id of the robot base
  footprint coordinate frame
- odom_frame (`std::string = "odom"`): tf frame id of the robot odometry
  coordinate frame
- map_frame (`std::string = "map"`): tf frame id of the map coordinate frame
- throttle_scans (`int = 1`): process 1/throttle_scans of the laser scans - set
  to higher number to skip more scans (>=1)
- map_update_interval (`float = 5.0`): map update period (s)
- maxUrange (`float = 80.0`): maximum usable range of the laser (m)
- sigma (`float = 0.05`): sigma used by the greedy endpoint matching
- kernelSize (`int = 1`): kernel in which to look for a correspondence
- lstep (`float = 0.05`): optimization step in translation
- astep (`float = 0.05`): optimization step in rotation
- iterations (`int = 5`): number of iterations of the scan matcher (>=1)
- lsigma (`float = 0.075`): sigma of a beam user for likelihood computation
- ogain (`float = 3.0`): gain used while evaluating the likelihood, for
  smoothing the resampling effects
- lskip (`int = 0`): number of beams to skip in each scan (0 = all rays)
- minimumScore (`float = 0.0`): minimum score for considering the outcome of the
  scan matching is good (0 <= score <~ 600+; 50 advisable when unstable pose)
- srr (`float = 0.1`): odometry translation error as a function of translation
  (m/m)
- srt (`float = 0.2`): odometry translation error as a function of rotation
  (m/th)
- str (`float = 0.1`): odometry rotation error as a function of translation
  (th/m)
- stt (`float = 0.2`): odometry rotation error as a function of rotation
  (th/th)
- linearUpdate (`float = 1.0`): process a scan each time the robot translates
  this far (m)
- angularUpdate (`float = 0.5`): process a scan each time the robot rotates this
  far (rad)
- temporalUpdate (`float = -1.0`): process a scan if the last one is older than
  the update time in seconds (<0 = turn time based updates off)
- resampleThreshold (`float = 0.5`): Neff-based resampling threshold
- particles (`int = 30`): number of particles in the filter
- xmin (`float = -100.0`): initial map size (m)
- ymin (`float = -100.0`): initial map size (m)
- xmax (`float = 100.0`): initial map size (m)
- ymax (`float = 100.0`): initial map size (m)
- delta (`float = 0.05`): map resolution (m)
- llsamplerange (`float = 0.01`): translational sampling range for the
  likelihood (m)
- llsamplestep (`float = 0.01`): translational sampling step for the likelihood
  (m)
- lasamplerange (`float = 0.005`): angular sampling range for the likelihood
  (rad)
- lasamplestep (`float = 0.005`): angular sampling step for the likelihood (rad)
- transform_publish_period (`float = 0.05`): transform publication period (s)
- occ_thresh (`float = 0.25`): threshold of the gmapping's occupancy values;
  cells with greater occupancy considered occupied
- maxRange (`float`): maximum range of the sensor

### Subscribes

- scan
  ([LaserScan.msg](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))
- tf (N/A)
  - base_frame > laser
  - odom > base_frame

### Publishes

- entropy
  ([Float64.msg](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- map
  ([OccupancyGrid.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html))
- map_metadata
  ([MapMetaData.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html))

### Services

See more in https://wiki.ros.org/gmapping#Services.

### Actions

None.

## Usage

### Installation

```sh
# Upgrade the installed packages on the system
sudo apt update
sudo apt dist-upgrade

# Install ROS package
sudo apt install ros-noetic-gmapping
```

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf run_gmapping.launch
```
