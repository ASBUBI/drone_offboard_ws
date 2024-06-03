# Source ROS2 distro
source /opt/ros/humble/setup.sh

# Check whether the Vicon SDK is already installed (prev. run script)
if [ -d "$HOME/drone_offboard_ws/src/vicon_stream/vicon_libs" ]
then
    echo "Vicon SDK already installed!"
else
    # Download sources from specific git repo (only necessary)
    echo "Cloning repo with ViconSDK libs..."
    git clone https://github.com/ASBUBI/Vicon_CPPClient.git $HOME/drone_offboard_ws/src/vicon_stream/Vicon_CPPClient
    mkdir $HOME/drone_offboard_ws/src/vicon_stream/vicon_libs
    echo "Copying files into \"vicon_libs\" folder..."
    mv $HOME/drone_offboard_ws/src/vicon_stream/Vicon_CPPClient/CppClient/lib/Linux/* $HOME/drone_offboard_ws/src/vicon_stream/vicon_libs
    # sudo chmod 0755 ./vicon_libs/*.*
    # sudo ldconfig
fi

# Cleanup
echo "Cleaning..."
rm -rf $HOME/drone_offboard_ws/src/vicon_stream/Vicon_CPPClient

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/drone_offboard_ws/src/vicon_stream/vicon_libs

# px4_msgs package "release/1.14" to match firmware 1.14.2 of the autopilot
echo "Importing \"px4_msgs\" package\n"
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git $HOME/drone_offboard_ws/src/px4_msgs

echo "Building \"px4_msgs\" package REQUIRED to build others..."
colcon build --packages-select px4_msgs

source $HOME/drone_offboard_ws/install/local_setup.sh

echo "Building drone offboard specific packages... --- vicon_stream --- v6c_offboard_control ---"
colcon build --packages-select vicon_stream v6c_offboard_control

echo "Sourcing environment.."
source $HOME/drone_offboard_ws/install/local_setup.sh

# uXRCE-DDS version "ros2"
# echo "Importing \"MicroDDS-uXRCE\" package...\n"
# git clone -b foxy https://github.com/eProsima/Micro-XRCE-DDS-Agent.git $HOME/drone_offboard_ws/src/Micro-XRCE-DDS-Agent
# echo "Building \"MicroDDS-uXRCE\" package...\n"
# colcon build --packages-select microxrcedds_agent