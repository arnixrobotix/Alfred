# hal_pose_manager package

## Overview

This package contains the node responsible for the communication with the controller:
- It computes odometry data from the motors' encoder count values and sends it to the controller. 

## Interfaces

### Topics

The odometry information is published in a [`Odometry`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg) message on `odometry`.

The motors' encoders values are received in a [`HalMotorControlEncoders`](../hal_motor_control_interfaces/msg/HalMotorControlEncoders.msg) message on `motorsEncoderCountValue`.

## Dependencies

### Internal

- [`hal_motor_control_interfaces`](../hal_motor_control_interfaces/README.md)
- [`common_utils`](../../utils/common_utils/README.md)

### External
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp`
- `rclcpp_lifecycle`
