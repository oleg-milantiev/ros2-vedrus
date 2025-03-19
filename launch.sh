#!/bin/bash

cd /opt/ros/iron/
source /opt/ros/iron/install/local_setup.bash
source /opt/ros/iron/setup.bash

#ros2 launch /opt/ros/iron/launch/r2.py
ros2 launch /ros2-vedrus/launch/short.py
