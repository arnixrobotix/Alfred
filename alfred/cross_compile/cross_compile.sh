#!/bin/bash

docker run --platform linux/arm64/v8 -v ..:/ros2_ws/ -v ./build_target.sh:/usr/bin/build_target.sh -v /usr/bin/qemu-aarch64-static:/usr/bin/qemu-aarch64-static ros:humble-ros-base-jammy@sha256:6faa295af1dddacafa9e4f381c9500ca2a7ee215503056f5eab2f450a5f267e1 /usr/bin/build_target.sh