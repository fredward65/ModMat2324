FROM osrf/ros:noetic-desktop-full

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install Universal Robots Packages
RUN sudo apt update && sudo apt upgrade -y \
    && sudo apt-get install -y ros-$ROS_DISTRO-universal-robots

# Install Python dependencies
RUN sudo apt update \
    && sudo apt install pip -y \
    && pip install sympy dill

# Create ROS workspace
ARG ROS_WS=ModMath2324
SHELL ["/bin/bash", "-c"]
RUN mkdir -p /home/${USERNAME}/${ROS_WS}/src \
    && cd /home/${USERNAME}/${ROS_WS} \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && catkin_make\
    && . devel/setup.bash \
    && echo "source ~/${ROS_WS}/devel/setup.bash" >> ~/.bashrc

# Smaller VNC resolution
ENV VNC_RESOLUTION="800x600x16"