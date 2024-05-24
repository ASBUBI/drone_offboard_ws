FROM osrf/ros:foxy-desktop
 
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
 
RUN sudo apt install -y \
	ca-certificates \
	gnupg \
	lsb-core \
	sudo \
	wget
 
# Rosdep update
RUN rosdep update
 
# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
 
WORKDIR /home/$USERNAME/drones_ws
 
# clone and build px4
RUN git clone --depth 1 --branch release/1.14 https://github.com/PX4/PX4-Autopilot.git --recursive
 
RUN ./PX4-Autopilot/Tools/setup/ubuntu.sh
 
# install Micro-XRCE-DDS-Agent
RUN git clone -b ros2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/micro-xrce-dds-agent \
&& cd /tmp/micro-xrce-dds-agent \
&& mkdir build && cd build \
&& cmake .. \
&& make -j $(nproc) \
&& sudo make install
 
RUN sudo ldconfig /usr/local/lib \
&& sudo rm -rf /tmp/*