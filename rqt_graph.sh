#!/bin/bash

# add 192.168.2.77 into c:\program files\vcxsrv\x0.hosts
# start vcxsrv at windows
export DISPLAY=192.168.2.70:0

cd /opt/ros/iron
source ./setup.bash

ros2 run rqt_graph rqt_graph

