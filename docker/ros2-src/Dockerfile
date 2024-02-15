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
      python3-flake8-docstrings \
      python3-pip \
      python3-pytest-cov \
      python3-flake8-blind-except \
      python3-flake8-builtins \
      python3-flake8-class-newline \
      python3-flake8-comprehensions \
      python3-flake8-deprecated \
      python3-flake8-import-order \
      python3-flake8-quotes \
      python3-pytest-repeat \
      python3-pytest-rerunfailures \
      ros-dev-tools

RUN mkdir -p /opt/ros/iron/src && \
    cd /opt/ros/iron && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src

# apt-utils?
RUN cd /opt/ros/iron && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN python3 -m pip install setuptools==58.2.0

RUN cd /opt/ros/iron && \
    colcon build --symlink-install
