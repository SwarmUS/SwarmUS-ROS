echo "Donwloading librealsense dependencies"
export REALSENSE_SOURCE_DIR=$HOME/librealsense
sudo apt-get install guvcview git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
git clone https://github.com/IntelRealSense/librealsense.git $REALSENSE_SOURCE_DIR

# udev rules
echo "Setting udev rules"
sudo cp $REALSENSE_SOURCE_DIR/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

