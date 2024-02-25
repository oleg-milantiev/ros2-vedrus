#!/bin/bash

cd /opt/ros/iron/
source /opt/ros/iron/setup.bash
source /opt/ros/iron/install/local_setup.bash

#rs-enumerate-devices #<- поддерживаемые режимы (в т.ч. fps)

ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -p enable_color:=false -p enable_infra1:=false -p enable_infra2:=false \
  -p enable_gyro:=true -p enable_accel:=true -p enable_depth:=true \
  -p depth_module.profile:=640x480x6 \
  -r __node:=D455
