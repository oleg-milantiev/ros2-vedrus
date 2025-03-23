# Используем базовый образ с CUDA
FROM nvidia/cuda:12.6.3-runtime-ubuntu24.04

LABEL maintainer="Oleg Milantiev oleg@milantiev.com"
LABEL description="NVIDIA CUDA Toolkit 12.6.3 runtime on Ubuntu 24.04 with ROS2 Jazzy"

ENV DEBIAN_FRONTEND=noninteractive

# Устанавливаем зависимости
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    lsb-release \
    gnupg2 \
    wget \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

RUN apt-get install -y software-properties-common && add-apt-repository universe && apt update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt-get upgrade -y


RUN apt-get install -y ros-jazzy-ros-base python3-pip ros-jazzy-usb-cam ros-jazzy-realsense2-camera

# Настраиваем окружение ROS 2
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/jazzy/install/local_setup.bash" >> /root/.bashrc

# Устанавливаем дополнительные инструменты
RUN apt-get install -y \
    build-essential \
    python3-colcon-common-extensions python3-opencv v4l-utils

RUN python3 -m pip --no-cache-dir install --break-system-packages torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
RUN python3 -m pip --no-cache-dir install --break-system-packages ultralytics

# Задаем рабочую директорию
WORKDIR /opt/ros/jazzy

# Открываем терминал по умолчанию
CMD ["/bin/bash"]
