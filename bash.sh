#!/bin/sh

docker run -it --rm --privileged \
    -v /dev/ttyUSB0:/dev/ttyUSB0 \
    -v /dev/ttyUSB1:/dev/ttyUSB1 \
    -v /dev/ttyUSB2:/dev/ttyUSB2 \
    -v /root/ros2-vedrus/launch:/opt/ros/iron/launch \
    -v /root/ros2-vedrus/vedrus:/opt/ros/iron/src/vedrus \
    -v /root/ros2-vedrus/vedrus_interfaces:/opt/ros/iron/src/vedrus_interfaces \
    ros2-src:vedrus bash
