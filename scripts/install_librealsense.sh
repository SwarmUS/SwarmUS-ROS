#! /bin/sh

echo "Donwloading librealsense dependencies"
export REALSENSE_SOURCE_DIR=$HOME/catkin_ws/src/SwarmUS-ROS/contrib/librealsense
sudo apt-get install guvcview git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
mkdir $REALSENSE_SOURCE_DIR/build
cd $REALSENSE_SOURCE_DIR/build

# build
echo "Building librealsense from source"
export REALSENSE_INSTALL_PREFIX=/opt/realsense
sudo mkdir -p $REALSENSE_INSTALL_PREFIX; 
sudo chown $USER:$USER -R $REALSENSE_INSTALL_PREFIX # not relay needed -> you could run _make_ followed by _sudo make install_
cmake ../ -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=false -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true -DCMAKE_INSTALL_PREFIX=$REALSENSE_INSTALL_PREFIX
sudo make uninstall && make clean && make -j4 && sudo make install

# extend the ld search path
sudo sh -c "echo $REALSENSE_INSTALL_PREFIX/lib > /etc/ld.so.conf.d/realsense.conf"
sudo ldconfig

# udev rules
echo "Setting the udev rules"
cd ~/projects/librealsense/
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules && sudo udevadm control --reload-rules && udevadm trigger

# make cmake config avaliable
echo "export realsense2_DIR=/opt/realsense/lib/cmake/realsense2" >> ~/.bashrc

# reboot
echo "Reboot to finish the installation"