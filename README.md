## What is Alfred?
Alfred is a self balanced two-wheels robot based around a Raspberry Pi. It will be able to move autonomously or be remotely controlled.

## Aim of the project
This project's main goal is for me to learn (or consolidate) and apply new skills:
- ROS/ROS 2, OpenCV and Google Test
- C++, Rust and UML

## Hardware
Alfred is composed of the following components:
- 2 motors [GA25-371](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/JGA25-371.pdf) (with quadrature encoders)
- [MDD3A drive](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/mdd3a.pdf)
- [MPU6050](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/MPU-6000-Datasheet1.pdf) accelerometer + gyroscope
- [HC-SR04](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/HCSR04.pdf) ultrasonic sensor
- [Raspberry Pi Camera V2](https://www.raspberrypi.com/products/camera-module-v2/)
- [Raspberry Pi 3B+](https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/)
- [TXB0104](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/txb0104.pdf) level shifter
- UBEC with a 5V output
- 4 18650 batteries

The remote used to control Alfred is a [STM32F3 Discovery board](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/STM32F3_Disc_manual.pdf) to which a [HC05](https://gitlab.com/arnixrobotix/alfred/-/blob/ros2/datasheets/HC05_DS.pdf) bluetooth module is connected.

## Software
The software powering Alfred is using [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) running on [Ubuntu server 22.04 LTS](https://ubuntu.com/download/raspberry-pi) for Raspberry Pi.
The Pi's code is using some librairies:
- [pigpio](http://abyz.me.uk/rpi/pigpio/index.html): a library used to control the GPIOs of the Rapsberry Pi

I am planning to use [Hubris](https://oxidecomputer.github.io/hubris/) (a small OS written in `Rust`) to run on the remote.

## Tooling
`Cargo` is used to compile the remote's code (written in `Rust`).
