# Use ROS Melodic as the base image
FROM ros:melodic

# User configuration arguments
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Fix for potential read-only file system issues
RUN mkdir -p /var/cache/apt/archives && chmod -R 777 /var/cache/apt/archives

# Create user and group, add sudo support, and install graphical interface packages
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo x11-apps x11-xserver-utils \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install Python 2.7 and necessary packages for GUI and ROS tools
RUN apt-get update && apt-get install -y \
    python-pip \
    git \
    ros-melodic-rviz \
    ros-melodic-rqt* \
    ros-melodic-plotjuggler \
    gazebo9 \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    ros-melodic-map-server \
    ros-melodic-catkin \
    curl \
    dbus-x11 libx11-xcb1 # Necessary for GUI applications

# Install catkin_pkg and other Python 2 dependencies for ROS
RUN pip install -U pip setuptools catkin_pkg rospkg

# Set the default shell
ENV SHELL /bin/bash

# Change the ROS workspace directory
WORKDIR /mini/ws/

# Set environment variable for workspace
ENV ROS_WORKSPACE /mini/ws

# Copy the necessary ROS project directories
COPY ./src/scout_ros $ROS_WORKSPACE/src/scout_ros
COPY ./src/ugv_sdk $ROS_WORKSPACE/src/ugv_sdk

# Expose the noVNC port
EXPOSE 8080

# Set bash as the default shell
SHELL ["/bin/bash", "-c"]

# Change ownership of the /mini/ directory to the created user
RUN chown -R "$USER_UID:$USER_GID" /mini/

# Switch to the created user
USER $USERNAME

# Install ROS dependencies for the scout_ros project
RUN . "/opt/ros/melodic/setup.bash" && \
    rosdep update --rosdistro "melodic" && \
    rosdep install -i --from-path src --rosdistro "melodic" -y \
    --skip-keys="catkin roscpp std_msgs"

ENTRYPOINT ["/bin/bash"]
CMD []