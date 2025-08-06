#!/bin/bash

. /opt/ros/humble/setup.bash

sudo apt-get update

cd /ros2_ws

rm -fr install/target

rosdep install -ry \
    --rosdistro humble \
    --from-paths src \
    --ignore-src

colcon --log-base log/target \
       build --build-base build/target \
             --install-base install/target \
             --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             --no-warn-unused-cli

chown -R 1000:1000 build/target install/target log/target