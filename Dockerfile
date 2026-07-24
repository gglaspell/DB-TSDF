# DB-TSDF image — built on osrf/ros:humble-desktop (ROS 2 Humble on Ubuntu 22.04)
#
# Uses the official OSRF ROS image, which already ships ROS 2 Humble Desktop,
# its apt repo, and a UTF-8 locale. This file only adds DB-TSDF's extra
# dependencies, clones the repo, and builds it with colcon.

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --- Extra build tools not guaranteed in the base image --------------------
RUN apt update && apt install -y --no-install-recommends \
    sudo \
    git \
    cmake \
    build-essential \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# --- DB-TSDF build/runtime dependencies not included in humble-desktop -----
# ros-humble-desktop itself already provides rclcpp, geometry-msgs,
# sensor-msgs, std-srvs, etc. This list only adds what's missing.
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-message-filters \
    libeigen3-dev \
    libboost-all-dev \
    libomp-dev \
    libpcl-dev \
    libvtk9-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python ROS tools not preinstalled in the base image
RUN pip3 install -U \
    colcon-common-extensions \
    rosdep \
    vcstool

# Initialize rosdep (system-wide, requires root — must run before USER switch)
RUN rosdep init || true

# --- Non-root user matching the host UID/GID -------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    echo "${USERNAME}:${USERNAME}" | chpasswd && \
    adduser ${USERNAME} sudo && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
RUN mkdir -p src

# rosdep update caches its index under $HOME/.ros/rosdep — run it as the
# user that will later call `rosdep install`.
RUN rosdep update || true

# --- Clone and build DB-TSDF ------------------------------------------------
RUN git clone -b atak https://github.com/gglaspell/DB-TSDF.git src/db_tsdf && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
        rosdep install --from-paths src --ignore-src -r -y && \
        colcon build"

# Auto-source workspace setup once it has been built
RUN echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

CMD ["bash"]
