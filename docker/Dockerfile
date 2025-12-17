FROM ros:humble-ros-base AS base

SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Add user
ENV USER=ubuntu
RUN groupadd -g 1000 ${USER} && \
    useradd -m -u 1000 -g ${USER} ${USER} && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "${USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USER} && \
    chmod 0440 /etc/sudoers.d/${USER}
	
# Install tools and tmuxinator
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ruby \
    ruby-dev \
    build-essential \
    tmux \
    nano && \
    rm -rf /var/lib/apt/lists/*
RUN gem install tmuxinator

# Install rviz2 and rqt_image_view
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

COPY launch_files/. /home/${USER}/launch_files

# Copying ros2 packages    
RUN mkdir -p /home/${USER}/ros2_ws/src && cd /home/${USER}/ros2_ws/src
COPY resources/ros_packages/. /home/${USER}/ros2_ws/src
WORKDIR /home/${USER}/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Install dependencies
RUN --mount=type=bind,source=./docker/scripts/install_deps.sh,target=/root/scripts/install_deps.sh \
    bash /root/scripts/install_deps.sh


# BUILD OPENCV    
FROM base AS build-opencv

ARG OPENCV_VERSION=4.10.0
RUN --mount=type=bind,source=./docker/scripts/build_opencv.sh,target=/home/${USER}/scripts/build_opencv.sh \
    bash /home/${USER}/scripts/build_opencv.sh ${OPENCV_VERSION}


# BUILD PRE-DEV
FROM base AS pre-dev
COPY --from=build-opencv /OpenCV/install /usr/local
RUN echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf && ldconfig


# BUILD ROS 2 DEPENDENCIES
FROM pre-dev AS dep-ros2
ARG MICRO_XRCE_DDS_AGENT_VERSION=v2.4.2
ARG PX4_MSGS_VERSION=release/1.16
ARG PX4_ROS2_INTERFACE_LIB_VERSION=release/1.16
ARG PX4_ROS_COM_VERSION=release/1.16
RUN --mount=type=bind,source=./docker/scripts/build_ros_deps.sh,target=/root/scripts/build_ros_deps.sh \
    bash /root/scripts/build_ros_deps.sh \
    ${MICRO_XRCE_DDS_AGENT_VERSION} \
    ${PX4_MSGS_VERSION} \
    ${PX4_ROS2_INTERFACE_LIB_VERSION} \
    ${PX4_ROS_COM_VERSION}


# BUILD PX4
FROM pre-dev AS build-px4
ARG PX4_VERSION=v1.16.0
RUN --mount=type=bind,source=./docker/scripts/build_px4.sh,target=/home/${USER}/scripts/build_px4.sh \
    bash /home/${USER}/scripts/build_px4.sh ${PX4_VERSION}


# BUILD DEV
FROM pre-dev AS dev
# install built packages
COPY --from=dep-ros2 --chown=${USER}:${USER} /root/px4_ros_ws/install /home/${USER}/px4_ros_ws/install

# Switch to user
USER ${USER}
WORKDIR /home/${USER}

# retrieve PX4-gazebo-models
COPY --from=build-px4 --chown=${USER}:${USER} /home/${USER}/PX4-Autopilot/Tools/simulation/gz /home/${USER}/PX4-gazebo-models

WORKDIR /home/${USER}/
COPY --from=build-px4 --chown=${USER}:${USER} /home/${USER}/px4_sitl /home/${USER}/px4_sitl
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc
    
# Entrypoint
COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

