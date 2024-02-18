#!/bin/sh

docker run -it --rm --privileged \
    -v /dev:/dev \
    -v /root/ros2-vedrus/launch:/opt/ros/iron/launch \
    -v /root/ros2-vedrus/vedrus:/opt/ros/iron/src/vedrus \
    -v /root/ros2-vedrus/vedrus_interfaces:/opt/ros/iron/src/vedrus_interfaces \
    --device-cgroup-rule "c 81:* rmw" \
    --device-cgroup-rule "c 189:* rmw" \
    ros2-apt bash
