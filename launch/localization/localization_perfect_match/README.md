# [localization_perfect_match](https://gitlab.inesctec.pt/jarvis/localization_perfect_match_stack)

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure)
- [eigen3](https://eigen.tuxfamily.org/dox/)
- [geometry_msgs](https://wiki.ros.org/geometry_msgs)
- [laser_geometry](https://wiki.ros.org/laser_geometry)
- [libopencv-dev](https://index.ros.org/d/libopencv-dev/)
- [libqt4](https://index.ros.org/d/libqt4-dev/)
- [message_filters](https://wiki.ros.org/message_filters)
- [roscpp](https://wiki.ros.org/roscpp)
- [sensor_msgs](https://wiki.ros.org/sensor_msgs)
- [tf](https://wiki.ros.org/tf)
- [itrci_nav](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/tree/main/itrci_nav)
  _(runtime)_
- INESC TEC Robotics Navigation Stack
    - qt_ros_interface

### Parameters

- GlobalFrameId (`string = "map"`): tf frame id of the global map
- OdomFrameId (`string = odom`): tf frame id of the robot odom coordinate frame
- BaseFrameId (`string = base_footprint`): tf frame id of the robot base
  footprint coordinate frame
- write_pose_result_to_tf (`bool = true`)
- invert_tf (`bool = false`)
- transform_tolerance (`double = 0.0`)
- MapFilePatch (`string`, only loaded if parameter defined): location in the
  local file system of the map (`directory/filename`)
- MapResolution (`double`, only loaded if MapFilePatch defined): resolution of
  the map (m/px)
- initial_position_sd (`double = 1.0`): initial standard deviation for the
  initial robot position estimation
- initial_orientation_sd (`double = 45.0`): initial standard deviation for the
  initial robot orientation estimation
- initial_pose_x (`double`, only loaded if all 3 parameters of initial pose
  defined): initial position of the robot in the x-axis (m)
- initial_pose_y (`double`, only loaded if all 3 parameters of initial pose
  defined): initial position of the robot in the y-axis (m)
- initial_pose_ori (`double`, only loaded if all 3 parameters of initial pose
  defined): initial orientation of the robot (deg)
- get_initial_pose_service_name (`string = ""`)
- LaserMeasuresNum (`int = 0`)

**Dynamic parameters (dynamic_reconfigure):**

- Optimizer:
    - Lc (`double = 1.0`)
    - NumIterationMax (`int = 100`): maximum number of iterations in the optimizer
    - OptimizationStopTranslation (`double = 0.0001`): stop condition of the
      optimizer relative to the translation component (m)
    - OptimizationStopRotation (`double = 0.01`): stop condition of the
      optimizer relative to the orientation component (rad)
    - DeltaUpdateInitPosition (`double = 0.1`)
    - DeltaUpdateInitOrientation (`double = 0.1`)
    - SensorsKErrorXY (`double = 1`)
    - SensorsKErrorTheta (`double = 1e-3`)
    - MinPercentValideSensorData (`double = 30`): minimum percentage for valid
      sensor data (%)
- Kalman filter:
    - OperationMode (`enum = SensorsOdomFusion`): operation mode of the Kalman
      filter
      (`None | OdomOnly | SensorsOnly | SensorsOdomNoFusion | SensorsOdomFusion`)
    - Odometry propagation drift
        - DriftTranslationDeltaD (`double = 0.001`)
        - DriftTranslationDeltaRot (`double = 0.0003`)
        - DriftRotationDeltaRot (`double = 0.001`)
    - StateMinVarXY (`double = 1e-2`): minimum covariance set for the robot
      position state
    - StateMinVarTheta (`double = 1e-3`): minimum covariance set for the robot
      orientation state
- Outlier filter:
    - MaxErrordistFilterEnabled1 (`bool = false`)
    - LaserFilterMaxDist1 (`double = 0.5`)
    - MaxErrordistFilterEnabled2 (`bool = false`)
    - LaserFilterMaxDist2 (`double = 0.05`)
    - WeightenMeasureDistEnable (`bool = false`)
    - FinalRefinementEnable (`bool = false`)

### Subscribes

- base_scan
  ([`LaserScan.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))
- base_scan_point_cloud
  ([`PointCloud.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud.html))
- base_scan_point_cloud2
  ([`PointCloud2.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- initialpose
  ([`PoseStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- map
  ([`OccupancyGrid.msg`](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html))

### Publishes

- ErrorMatch
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- LaserCorrectionX
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- LaserCorrectionY
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- LaserCorrectionTheta
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- LocalisationPoseResult
  ([`PoseStamped.msg`](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html))
- LocCorrectionX
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- LocCorrectionY
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- LocCorrectionTheta
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- PercentValidLaserMeasures
  ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
- pose_result_2d
  ([`Pose2DWithCovarianceStamped.msg`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/main/itrci_nav/msg/Pose2DWithCovarianceStamped.msg))
- pose_result_uncertainty
  ([`MarkerArray.msg`](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html))
- debug/
    - RPropIterationNumber
      ([`Int16.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int16.html))
    - TimeLoad
      ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
    - TimeCycle
      ([`Float64.msg`](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html))
    - LaserScan
      ([`PointCloud.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud.html))
    - LaserScanfiltered
      ([`PointCloud.msg`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud.html))

### Services

- loadMap
  ([`load_occupancy_grid.srv`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/main/itrci_nav/srv/load_occupancy_grid.srv))
- publishToTf
  ([`SetBool.srv`](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/SetBool.html))

### Actions

None.

## Usage

```sh
roslaunch sdpo_msl_ros_nav_conf run_localization_perfect_match.launch
```
