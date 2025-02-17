#!/bin/bash

# add 192.168.2.77 into c:\program files\vcxsrv\x0.hosts
# start vcxsrv at windows
#export DISPLAY=192.168.2.70:0

#or use vnc

cd /opt/ros/iron
source ./setup.bash

ros2 run rviz2 rviz2

