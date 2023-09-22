# Description

This directory contains the `hal` component related to the proximity sensor (`HC-SR04`).

This node has dependencies on `hal_pigpio`. It sends a pulse of 20us every 100ms to the proximity sensor. The proximity sensor then responds with an edge which has a length proportional to the distance to the closest obstacle in front of the sensor (59us corresponds to 1 cm). The node also publishes every 100ms the distance it computed.