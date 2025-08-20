# hal_motor_control package

## Overview

This package contains the motors' (`GA25-371`) driver. It consists of a [library](./include/hal_motor.hpp) defining a `motor` class and some associated member functions and a node (`hal_motorControl_node`) performing the configuration of the motors at startup, sending the command PWM signals to them and publishing the processed encoder count values at a 100Hz rate.

## Interfaces

### Topics

The motors' encoders values are published in a [`HalMotorControlEncoders`](../hal_motor_control_interfaces/msg/HalMotorControlEncoders.msg) message on `motorsEncoderCountValue`.

Velocity commands are received in a [`HalMotorControlCommand`](../hal_motor_control_interfaces/msg/HalMotorControlCommand.msg) message on `wheelsCmd`.

Raw encoder count values are received in a [`HalPigpioEncoderCount`](../hal_pigpio_interfaces/msg/HalPigpioEncoderCount.msg) message on `hal_pigpioEncoderCount`.

## Dependencies

### Internal

- [`hal_pigpio_interfaces`](../hal_pigpio_interfaces/README.md)
- [`hal_motor_control_interfaces`](../hal_motor_control_interfaces/README.md)
- [`common_utils`](../../utils/common_utils/README.md)

### External

- `ament_cmake`
- `ament_lint_auto`
- `ament_lint_common`
- `ament_cmake_gtest`
- `rclcpp`
- `rclcpp_lifecycle`
