# 5dpo_msl_ros_nav_conf

**Version 2.3.0**

This repository implements the launch files required for the 5DPO Navigation
Stack on the 5DPO MSL three-wheeled omnidirectional robot. The system
implemented is based on the INESC TEC Robotics Navigation Stack that it allows
you to have different configurations implemented and selecting just one based on
your environment variables.

**With this version, it is possible to do:**

- ROS package creation (`CMakeLists.txt`, `package.xml`)
- `basic` configuration
- `slam0` configuration ([SLAM Toolbox](https://wiki.ros.org/slam_toolbox))
- `slam1` configuration ([Hector SLAM](https://wiki.ros.org/hector_mapping))
- `slam2` configuration ([GMapping](https://wiki.ros.org/gmapping))
- `feup0` configuration (INESC TEC Robotics Navigation Stack)

**The next version will add these features:**

- `sim0` configuration (Gazebo-based Simulation)

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [5dpo_msl_firmware](https://github.com/5dpo/5dpo_msl_firmware)
- [gmapping](https://wiki.ros.org/gmapping)
- [hector_mapping](https://wiki.ros.org/hector_mapping)
- [map_server](https://wiki.ros.org/map_server)
- [rviz](https://wiki.ros.org/rviz)
- [sdpo_driver_omnijoy](https://github.com/5dpo/5dpo_driver_omnijoy)
- [sdpo_msl_ros_driver](https://github.com/5dpo/5dpo_msl_ros_driver)
- [sdpo_ros_odom](https://github.com/5dpo/5dpo_ros_odom)
- [slam_toolbox](https://wiki.ros.org/slam_toolbox)
- [tf](https://wiki.ros.org/tf)
- [urg_node](https://wiki.ros.org/urg_node)
- INESC TEC Robotics Navigation Stack
  - [graph_server](https://gitlab.inesctec.pt/jarvis/graph_planning_handlers_stack/-/tree/main/graph_server)
  - [localization_perfect_match](https://gitlab.inesctec.pt/jarvis/localization_perfect_match_stack)
  - [navigation_handler2](https://gitlab.inesctec.pt/jarvis/graph_planning_handlers_stack/-/tree/main/navigation_handler2)
  - [parametric_trajectories_control](https://gitlab.inesctec.pt/jarvis/parametric_trajectories_stack/-/tree/main/parametric_trajectories_control)
  - [parametric_trajectories_editor](https://gitlab.inesctec.pt/jarvis/parametric_trajectories_stack/-/tree/main/parametric_trajectories_editor)
  - [path_planner](https://gitlab.inesctec.pt/jarvis/path_planner)
  - [symbolic_pose_tracker](https://gitlab.inesctec.pt/jarvis/teastar_global_decision/-/tree/main/symbolic_pose_tracker)

## Usage

### Configurations

**Usage**

```sh
# Robot id
export ROBOT_ID=<id>                # (default: unnamed_robot)
# Configuration
export ROBOT_CONF=<configuration>   # (default: basic)
```

**`basic`** (joystick, odometry, rviz)

**`slam0`** (SLAM Toolbox, joystick)

**`slam1`** (HectorSLAM, joystick)

**`slam2`** (GMapping, joystick)

_How to play ROSbag data and execute a certain SLAM algorithm?_

```sh
export ROBOT_CONF=<SLAM configuration>
roslaunch sdpo_msl_ros_nav_conf run_rosbag_play.launch filenames:=$(pwd)/log_slam_2023-03-28-15-05-07_0.bag
```
  
**`feup0`** (graph, parametric trajectories, perfect match localization)


### Compilation

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_msl_ros_nav_conf.git

# Build
cd ..
catkin build
```

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf wake_up_almighty_msl.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
