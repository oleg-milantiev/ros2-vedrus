#!/bin/sh

docker run -it --rm --privileged \
    -v /dev:/dev \
    -v /sys:/sys \
    -v /root/ros2-vedrus/launch:/opt/ros/iron/launch \
    -v /root/ros2-vedrus/diagnostics:/opt/ros/iron/src/diagnostics \
    -v /root/ros2-vedrus/vision_opencv:/opt/ros/iron/src/vision_opencv \
    -v /root/ros2-vedrus/realsense-ros:/opt/ros/iron/src/realsense-ros \
    -v /root/ros2-vedrus/xacro:/opt/ros/iron/src/xacro \
    -v /root/ros2-vedrus/vedrus:/opt/ros/iron/src/vedrus \
    -v /root/ros2-vedrus/vedrus_interfaces:/opt/ros/iron/src/vedrus_interfaces \
    ros2-src:realsense bash
