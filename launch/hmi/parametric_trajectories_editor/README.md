# parametric_trajectories_editor

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](http://wiki.ros.org/noetic)

### Dependencies

- TBC

### Parameters

- HolomonicMode (`bool`): enable if the robot is omnidirectional
- GlobalFrameId (`string`): tf frame id of the global map
- BaseFrameId (`string`): tf frame id of the robot base footprint coordinate
  frame
- tf_prefix (`string`)
- ZOffSet (`double`):
- default_speed (`double`): default speed when creating the parametric
  trajectories
- default_paramf_and_paramb (`double`)
- IdsOffSet (`int`)
- arrow_length (`double`): visualization parameter of the arrow length
- arrow_width (`double`): visualization parameter of the arrow width
- pub_tf_period (`double`)
- come_here_service_name (``): name of the service for the goto functionality of
  the controller (interaction through rviz)

### Subscribes

- TBC
([`.msg`]())

### Publishes

- TBC
([`.msg`]())

### Services

- TBC
([`.srv`]())

### Actions

- TBC

## Usage

### Launch

```sh
roslaunch sdpo_msl_ros_nav_conf run_parametric_trajectories_editor.launch
```

### Save trajectories

```sh
# Save the trajectories to the current directory where the launch file is
# executed
roslaunch sdpo_msl_ros_nav_conf run_save_trajectory.launch ns:=nn0
```

### Load trajectories

```sh
# Load the trajectories from the current directory where the launch file is
# executed
roslaunch sdpo_msl_ros_nav_conf run_load_trajectory.launch
```
