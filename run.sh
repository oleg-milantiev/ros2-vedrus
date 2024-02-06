#!/bin/sh

docker run -it --rm --privileged \
    -v /dev/ttyUSB0:/dev/ttyUSB0 \
    -v /root/docker-ros/vedrus:/opt/ros/iron/src/vedrus \
    -v /root/docker-ros/vedrus_interfaces:/opt/ros/iron/src/vedrus_interfaces \
    ros2-src:vedrus bash
