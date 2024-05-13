export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./src/vicon_stream/vicon_libs

echo "Importing \"px4_msgs\" package\n"
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs

echo "Building \"px4_msgs\" package REQUIRED to build others..."
colcon build --packages-select px4_msgs

source install/local_setup.sh

echo "Building drone offboard specific packages..."
colcon build --packages-select vicon_stream v6c_offboard_control

echo "Sourcing environment.."

source install/local_setup.sh