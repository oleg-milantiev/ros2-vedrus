#!/bin/bash

cd /opt/ros/iron/
source /opt/ros/iron/install/local_setup.bash
source /opt/ros/iron/setup.bash
ros2 launch /opt/ros/iron/launch/yolo.py
ros2 launch /opt/ros/iron/launch/ardu.py
