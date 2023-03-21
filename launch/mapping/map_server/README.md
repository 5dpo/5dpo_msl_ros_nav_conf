# [map_server](https://wiki.ros.org/map_server)

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [bullet](https://wiki.ros.org/bullet)
- [nav_msgs](https://wiki.ros.org/nav_msgs)
- [roscpp](https://wiki.ros.org/roscpp)
- [sdl](https://index.ros.org/d/sdl/)
- [sdl-image](https://index.ros.org/d/sdl-image/)
- [tf2](https://wiki.ros.org/tf2)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [roslib](https://wiki.ros.org/roslib) (_test_)
- [rospy](https://wiki.ros.org/rospy) (_test_)
- [rosunit](https://wiki.ros.org/rosunit) (_test_)
- [rostest](https://wiki.ros.org/rostest) (_test_)

### Parameters

- frame_id (`string = "map"`): frame to set in the header of the published map

### Subscribes

None.

### Publishes

- map
  ([`OccupancyGrid.msg`](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
  (_latched topic_)
- map_metadata
  ([`MapMetaData.msg`](https://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html))

### Services

- static_map
  ([`GetMap.srv`](https://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))

### Actions

None.

## Usage

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf run_map_server.launch
```

### Save map

```sh
# Save the map to the current directory where the launch file is executed
roslaunch sdpo_msl_ros_nav_conf run_map_saver.launch
```

**Usage** (map_saver)

```sh
rosrun map_server map_saver [--occ <threshold_occupied>] [--free <threshold_free>] [-f <mapname>] map:=/<topicname>
```
