# Check whether the Vicon SDK is already installed (prev. run script)
if [ -d ".src/vicon_stream/vicon_libs" ]
then
    echo "Vicon SDK already installed!"
    exit 1
fi

# Download sources from specific git repo (only necessary)
echo "Cloning repo with ViconSDK libs..."
git clone https://github.com/ASBUBI/Vicon_CPPClient.git ./src/vicon_stream/
mkdir .src/vicon_stream/vicon_libs
echo "Copying files into \"vicon_libs\" folder..."
mv ./src/vicon_stream/Vicon_CPPClient/CppClient/lib/Linux/* ./src/vicon_stream/vicon_libs
# sudo chmod 0755 ./vicon_libs/*.*
# sudo ldconfig

# Cleanup
echo "Cleaning..."
rm -rf ./src/vicon_stream/Vicon_CPPClient

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