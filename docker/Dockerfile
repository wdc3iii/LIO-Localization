# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# username, uid, gid
ARG USER=user
ARG UID=1000
ARG GID=1000
ENV USER=$USER
ENV UID=$UID
ENV GID=$GID

# set timezone
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# basic dependencies from docker_setup.sh (up until sudo and below)
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    cmake \
    clang-tools-12 \
    nano \
    vim \
    git \
    libeigen3-dev \
    x11-apps \
    locales \
    iputils-ping \
    sudo && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US.UTF-8

# create non-root user with sudo privileges for certain commands
RUN groupadd --gid $GID $USER && \
    useradd --uid $UID --gid $GID -m $USER -d /home/${USER} --shell /usr/bin/bash && \
    echo "${USER}:password" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "%${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# switch to new user and workdir
USER ${UID}

# Install ROS 2 and other dependencies
COPY --chown=${UID}:${GID} scripts/docker_setup.sh /tmp/docker_setup.sh
RUN bash /tmp/docker_setup.sh && rm /tmp/docker_setup.sh

# Ensure pip-installed binaries (colcon, etc.) are on PATH
ENV PATH="/home/${USER}/.local/bin:${PATH}"

WORKDIR /home/${USER}

# Lidar odometry
# Install PCL and other dependencies for the lidar
RUN sudo apt-get update && sudo apt-get install -y \
    libpcl-dev \
    pcl-tools \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions && \
    sudo rm -rf /var/lib/apt/lists/*

# Build Livox SDK2
WORKDIR /home/${USER}
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    sudo make install

# Build Livox ROS2 driver
RUN mkdir -p /home/${USER}/livox_driver_ws/src && \
    cd /home/${USER}/livox_driver_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cd livox_ros_driver2 && \
    source /opt/ros/humble/setup.sh && \
    ./build.sh humble

# Auto-source ROS 2 and Livox driver on shell startup
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USER}/.bashrc && \
    echo 'source /home/'"${USER}"'/livox_driver_ws/install/setup.bash' >> /home/${USER}/.bashrc
