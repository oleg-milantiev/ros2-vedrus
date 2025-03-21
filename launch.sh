#!/bin/bash

cd /opt/ros/jazzy/
source /opt/ros/jazzy/install/local_setup.bash
source /opt/ros/jazzy/setup.bash

#ros2 launch /ros2-vedrus/launch/r2.py
ros2 launch /ros2-vedrus/launch/short.py
