FROM ubuntu

RUN apt-get update && apt-get -y upgrade && \
    apt-get install -y software-properties-common && \
    add-apt-repository -y universe

RUN apt-get update && \
    apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Moscow

RUN apt-get update && \
    apt-get install -y \
      python3-pip ros-iron-ros2-control ros-iron-ros2launch \
      ros-iron-realsense2-camera \
      ros-dev-tools

RUN mkdir -p /opt/ros/iron/src && \
    cd /opt/ros/iron && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN python3 -m pip install setuptools==58.2.0

