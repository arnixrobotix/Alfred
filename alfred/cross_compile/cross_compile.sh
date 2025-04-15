#!/bin/bash

. /opt/ros/humble/setup.bash

sudo apt-get update

cd /ros2_ws

rosdep install -ry \
    --rosdistro humble \
    --from-paths src \
    --ignore-src

colcon build --symlink-install \
             --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             --no-warn-unused-cli