#!/bin/sh

# ROS2 on Ubuntu via pkg
#https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html
apt-get update && apt-get upgrade -y
apt-get install -y software-properties-common
add-apt-repository universe

apt-get update && apt-get upgrade -y
apt-get install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt-get install -y ros-dev-tools
apt-get install -y ros-iron-desktop

apt-get install -y ros-iron-realsense2-camera
#https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation
#ros2 run realsense2_camera realsense2_camera_node
#rs-enumerate-devices # поддерживаемые режимы (в т.ч. fps)
#ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p enable_infra1:=false -p enable_infra2:=false -p enable_gyro:=true -p enable_accel:=true -p enable_depth:=true -p depth_module.profile:=640x480x6

apt-get install -y ros-iron-usb-cam
#https://github.com/ros-drivers/usb_cam/tree/ros2

#ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video6 -p image_width:=640 -p framerate:=6.0
#https://github.com/ros-drivers/usb_cam/blob/ros2/config/params_1.yaml

apt-get install -y python3-pip python3-serial
python3 -m pip install setuptools==58.2.0
python3 -m pip install getch # keyboard controller
python3 -m pip install simple-pid
python3 -m pip install aiohttp[speedups]

ln -s `pwd`/launch.sh /opt/ros/iron/launch.sh
ln -s `pwd`/launch /opt/ros/iron/launch
ln -s `pwd`/influx /opt/ros/iron/src/influx
ln -s `pwd`/vedrus /opt/ros/iron/src/vedrus
ln -s `pwd`/vedrus_interfaces /opt/ros/iron/src/vedrus_interfaces
ln -s `pwd`/yolov8_rknn /opt/ros/iron/src/yolov8_rknn
ln -s `pwd`/yolov8_interfaces /opt/ros/iron/src/yolov8_interfaces
ln -s `pwd`/ros2_mpu9250_driver /opt/ros/iron/src/ros2_mpu9250_driver
ln -s `pwd`/build.sh /opt/ros/iron/build.sh
