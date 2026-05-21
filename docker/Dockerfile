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

# GLIM is desktop-only mapping tooling. Set WITH_GLIM=true at build time to
# pull in libgtsam-no-tbb-dev, libiridescence-dev, gtsam_points, and a glim_ws
# install. Skipped by default so onboard images stay slim. WITH_GLIM_CUDA=true
# additionally pulls in the nvidia-cuda-toolkit and builds gtsam_points with
# CUDA support (only meaningful on a host with NVIDIA drivers).
ARG WITH_GLIM=false
ARG WITH_GLIM_CUDA=false
ENV WITH_GLIM=$WITH_GLIM
ENV WITH_GLIM_CUDA=$WITH_GLIM_CUDA

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
    libboost-all-dev \
    libyaml-cpp-dev \
    x11-apps \
    locales \
    iputils-ping \
    tmux \
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
    sudo make install && \
    sudo ldconfig

# Build Livox ROS2 driver
RUN mkdir -p /home/${USER}/livox_driver_ws/src && \
    cd /home/${USER}/livox_driver_ws/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    cd livox_ros_driver2 && \
    source /opt/ros/humble/setup.sh && \
    ./build.sh humble

RUN mkdir -p /home/${USER}/ws_hesai/src && \
    cd /home/${USER}/ws_hesai/src && \
    git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git && \
    cd /home/${USER}/ws_hesai && \
    source /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Optional: GLIM (desktop only). Toggled by WITH_GLIM=true at build time.
# Pulls in GTSAM, Iridescence, builds gtsam_points, then builds glim + glim_ros2.
# WITH_GLIM_CUDA=true additionally installs CUDA 12.1 from NVIDIA's official
# apt repo (not the distro `nvidia-cuda-toolkit`, which ships CUDA 11.5 + drags
# in 3 conflicting nvidia-driver userspace packages) and builds gtsam_points
# with BUILD_WITH_CUDA=ON. Driver itself comes from the host via the NVIDIA
# Container Toolkit at run time. CUDA_ARCH 89 targets RTX 40-series (Ada);
# override CUDA_ARCH at build time for other GPUs (e.g. 86 for RTX 30-series,
# 75 for RTX 20-series, "75;86;89" for a fat binary).
ARG CUDA_ARCH=89
RUN if [ "${WITH_GLIM}" = "true" ] || [ "${WITH_GLIM_CUDA}" = "true" ]; then \
        set -e ; \
        echo "Installing GLIM build dependencies..." && \
        sudo apt-get update && \
        sudo apt-get install -y --no-install-recommends \
            software-properties-common wget gnupg ca-certificates \
            libfmt-dev libspdlog-dev libopencv-dev \
            libmetis-dev libomp-dev \
            libpng-dev libjpeg-dev libglm-dev libglfw3-dev \
            ros-humble-cv-bridge ros-humble-image-transport-plugins \
            ros-humble-tf2-ros && \
        sudo add-apt-repository -y ppa:koide3/iridescence && \
        sudo add-apt-repository -y ppa:koide3/gtsam && \
        sudo apt-get update && \
        sudo apt-get install -y --no-install-recommends \
            libiridescence-dev libgtsam-no-tbb-dev && \
        if [ "${WITH_GLIM_CUDA}" = "true" ]; then \
            echo "Installing CUDA 12.6 toolkit from NVIDIA repo..." && \
            cd /tmp && \
            wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
            sudo dpkg -i cuda-keyring_1.1-1_all.deb && \
            rm cuda-keyring_1.1-1_all.deb && \
            sudo apt-get update && \
            sudo apt-get install -y --no-install-recommends cuda-toolkit-12-6 ; \
        fi && \
        sudo rm -rf /var/lib/apt/lists/* && \
        echo "Building gtsam_points (CUDA=${WITH_GLIM_CUDA}, arch=${CUDA_ARCH})..." && \
        if [ "${WITH_GLIM_CUDA}" = "true" ]; then \
            export PATH=/usr/local/cuda-12.6/bin:$PATH ; \
            export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH ; \
            export CMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} ; \
        fi && \
        cd /home/${USER} && \
        git clone https://github.com/koide3/gtsam_points && \
        cd gtsam_points && mkdir build && cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_WITH_CUDA=${WITH_GLIM_CUDA} \
            -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} && \
        make -j$(nproc) && sudo make install && sudo ldconfig && \
        echo "Building glim_ws..." && \
        mkdir -p /home/${USER}/glim_ws/src && \
        cd /home/${USER}/glim_ws/src && \
        git clone https://github.com/koide3/glim.git && \
        git clone https://github.com/koide3/glim_ros2.git && \
        cd /home/${USER}/glim_ws && \
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_WITH_CUDA=${WITH_GLIM_CUDA} \
            -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCH} \
            -DBUILD_WITH_VIEWER=ON ; \
    else \
        echo "Skipping GLIM install (WITH_GLIM=${WITH_GLIM})" ; \
    fi

# Auto-source ROS 2 and Livox driver on shell startup
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USER}/.bashrc && \
    echo 'source /home/'"${USER}"'/livox_driver_ws/install/setup.bash' >> /home/${USER}/.bashrc && \
    echo 'source /home/'"${USER}"'/ws_hesai/install/setup.bash' >> /home/${USER}/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/home/'"${USER}"'/livox_driver_ws/install/livox_ros_driver2/lib:$LD_LIBRARY_PATH' >> /home/${USER}/.bashrc && \
    if [ "${WITH_GLIM}" = "true" ] || [ "${WITH_GLIM_CUDA}" = "true" ]; then \
        echo 'source /home/'"${USER}"'/glim_ws/install/setup.bash' >> /home/${USER}/.bashrc ; \
    fi && \
    if [ "${WITH_GLIM_CUDA}" = "true" ]; then \
        echo 'export PATH=/usr/local/cuda-12.6/bin:$PATH' >> /home/${USER}/.bashrc && \
        echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH' >> /home/${USER}/.bashrc ; \
    fi
