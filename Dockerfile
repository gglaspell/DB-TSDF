# Build the supplied checkout. Refresh ROS_IMAGE deliberately after testing.
# Digest resolved from Docker Hub on 2026-09-05 CDT.
ARG ROS_IMAGE=osrf/ros:humble-desktop@sha256:b624d8bcea33796d32e0dbd85326722188485759bbd6b538e4785750a5c88a7b
FROM ${ROS_IMAGE}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3-colcon-common-extensions python3-venv \
    python3-pip python3-yaml wget zip libeigen3-dev libpcl-dev libvtk9-dev \
    libomp-dev libssl-dev nlohmann-json3-dev \
    ros-${ROS_DISTRO}-pcl-conversions ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-robot-localization ros-${ROS_DISTRO}-rosbag2 \
    && rm -rf /var/lib/apt/lists/*
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd -g ${USER_GID} ${USERNAME} && \
    useradd -m -u ${USER_UID} -g ${USER_GID} ${USERNAME}
WORKDIR /home/${USERNAME}/ros2_ws
COPY --chown=${USER_UID}:${USER_GID} . src/db_tsdf/
RUN chown ${USER_UID}:${USER_GID} .
USER ${USERNAME}
ARG BUILD_JOBS=2
ENV CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS}
ENV DB_TSDF_WORKSPACE=/home/${USERNAME}/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --executor sequential --cmake-args \
      -DCMAKE_BUILD_TYPE=Release -DDB_TSDF_NATIVE=OFF && \
    ctest --test-dir build/db_tsdf --output-on-failure && \
    dpkg-query -W > install/system-packages.txt
RUN echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source \"${DB_TSDF_WORKSPACE}/install/setup.bash\" && exec \"$@\"", "--"]
CMD ["bash"]
