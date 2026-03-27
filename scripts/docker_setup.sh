#!/bin/bash

# basic deps
sudo apt-get update && sudo apt-get install -y \
    curl \
    git \
    mesa-common-dev \
    python3-dev \
    python3-pip \
    python-is-python3 \
    build-essential \
    cmake \
    libglfw3-dev \
    locales

# ros-related deps
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
sudo apt-get update -y && \
sudo apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    ros-humble-rosidl-generator-cpp \
    ros-humble-rosidl-default-generators \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz-visual-tools
source /opt/ros/humble/setup.bash

# python deps
pip install -U \
    colcon-common-extensions \
    "ruamel.yaml"

echo -e "\033[1;32mSystem dependencies installed successfully!\033[0m"


# check whether /etc/sysctl.d/60-cyclonedds.conf is a directory - if it is, delete it
if [ -d /etc/sysctl.d/60-cyclonedds.conf ]; then
    sudo rm -rf /etc/sysctl.d/60-cyclonedds.conf
fi

# apply performance optimizations
if ! grep -q "net.core.rmem_max=8388608" /etc/sysctl.d/60-cyclonedds.conf; then
    echo 'net.core.rmem_max=8388608' | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
fi

if ! grep -q "net.core.rmem_default=8388608" /etc/sysctl.d/60-cyclonedds.conf; then
    echo 'net.core.rmem_default=8388608' | sudo tee -a /etc/sysctl.d/60-cyclonedds.conf
fi

echo -e "\033[1;32mCyclone DDS performance optimizations enabled permanently!\033[0m"

# check whether /opt/ros/humble/setup.bash exists; if so, source
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "\033[1;32mROS 2 sourced successfully!\033[0m"
else
    echo -e "\033[1;33mROS 2 not sourced because /opt/ros/humble/setup.bash does not exist!\033[0m"
fi
