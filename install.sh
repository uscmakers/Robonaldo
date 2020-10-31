#!/bin/bash

SCRIPT_DIR="$(dirname "$0")"
SCRIPT_DIR="$(realpath $SCRIPT_DIR)"
echo "script_dir: $SCRIPT_DIR"
cd $SCRIPT_DIR # Make sure working directory is directory of this script.

# Install Arduino-CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
echo "export PATH=\"$SCRIPT_DIR/bin:\$PATH\"" >> ~/.bashrc


# Add ROS key so packages can be installed.
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
apt update

apt -y install ros-melodic-desktop-full

# Add ROS to ~/.bashrc to enable commands.
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional recommended ROS tools
apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Initialize rosdep which is necessary for some tools
if [ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
sudo -u $SUDO_USER rosdep update

# Install Robonaldo dependencies
apt -y install libsdl2-dev ros-melodic-rosserial-arduino ros-melodic-rosserial