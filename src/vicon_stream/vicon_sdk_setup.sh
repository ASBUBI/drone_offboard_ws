# Check whether the Vicon SDK is already installed (prev. run script)
if [ -d "./vicon_libs" ]
then
    echo "Vicon SDK already installed!"
    exit 1
fi

# Download sources from specific git repo (only necessary)
echo "Cloning repo with ViconSDK libs..."
git clone https://github.com/ASBUBI/Vicon_CPPClient.git
mkdir ./vicon_libs
echo "Copying files into \"vicon_libs\" folder..."
mv ./Vicon_CPPClient/CppClient/lib/Linux/* ./vicon_libs
# sudo chmod 0755 ./vicon_libs/*.*
# sudo ldconfig

# Cleanup
echo "Cleaning..."
rm -rf Vicon_CPPClient