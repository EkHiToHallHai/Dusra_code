FROM osrf/ros:jazzy-desktop-full

# Update all packages
RUN apt-get update \
    && apt-get upgrade -y \
    && rosdep update \
    && apt-get install -y \
    python3-colcon-clean \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
## Install Gazebo Harmonic and a few other useful dependencies
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    # ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-twist-stamper \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
## Very important for default render engine of Gazebo Harmonic
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*


ARG USERNAME
ARG USER_UID
ARG USER_GID
RUN existing_user=$(getent passwd $USER_UID | cut -d: -f1) && \
    if [ -z "$existing_user" ]; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    else \
        echo "User with UID $USER_UID exists: $existing_user" && \
        usermod -l $USERNAME -d /home/$USERNAME -m $existing_user; \
    fi && \
    mkdir -p /home/$USERNAME/.config && \
    chown $USER_UID:$USER_GID /home/$USERNAME
# ? Note: sudo is already installed in the image
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash\n \
export XDG_RUNTIME_DIR=/tmp/runtime-root\n" >> /home/$USERNAME/.bashrc
# RUN chown -R $USER_UID:$USER_GID /home/autodrive_devkit /home/autodrive_devkit.sh
USER $USER_UID:$USER_GID


ENV WS_DIR=/home/${USERNAME}/ros_ws
RUN mkdir -p $WS_DIR/src/packages/.vscode
WORKDIR $WS_DIR
RUN rosdep update
RUN ln -s $WS_DIR/src/packages/.vscode $WS_DIR/.vscode
#TODO: add git hooks for various things (especially automatically building submission docker image and pushing it)
#TODO: set up C++ and Python Debugging using VS Code ROS Extension
