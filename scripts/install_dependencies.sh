#Get the ubuntu codename for the release
codename=$( lsb_release --codename | cut -f2)

# Install package to set server keys
sudo apt-get update && sudo apt-get install software-properties-common

# Register server keys to install some dependencies
# Intel RealSense SDK
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $codename main" -u

# Installing dependencies
sudo apt-get update && apt-get install -y \
    wget \
    build-essential \
    apt-utils \
    ca-certificates \
    curl \
    make \
    python3 \
    python3-pip \
    unzip \
    wget \
    zip \
    protobuf-compiler \
    python3-protobuf \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
    clang-tidy \
    clang-format \
    doxygen \

# Install cmake from pip for the latest release
pip3 install cmake

# Install Buzz. The /tmp/buzz path can be changed.
git clone https://github.com/MISTLab/Buzz.git /tmp/buzz && \
    cd /tmp/buzz && \
    mkdir build && \
    cd build && \
    cmake ../src && \
    make && \
    sudo make install && \
    sudo ldconfig

# Install HiveMindBridge
cd contrib/HiveMindBridge && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    sudo make install


