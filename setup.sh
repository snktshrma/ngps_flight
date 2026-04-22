#!/bin/bash
sudo -v

set -e

# Require USER to be set (needed by ArduPilot prereqs script for usermod)
if [[ -z "$USER" ]]; then
    echo "ERROR: USER environment variable is not set."
    echo "Run setup as: USER=dev bash setup.sh"
    exit 1
fi

# Validate expected workspace layout
if [[ ! -d ~/ngps_ws/src/ngps_flight ]]; then
    echo "ERROR: ~/ngps_ws/src/ngps_flight not found."
    echo "Clone the repo there or create a symlink:"
    echo "  ln -s /path/to/ngps_flight ~/ngps_ws/src/ngps_flight"
    exit 1
fi

echo "Setting up ngps_ws :)"
cd ~/ngps_ws

echo "Importing ros2 repos..."
vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

echo "Importing gazebo repos..."
vcs import --recursive --input ~/ngps_ws/src/ngps_flight/ros2_gz.repos src

echo "Installing ArduPilot prerequisites..."
cd ~/ngps_ws/src/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.bashrc
cd ~/ngps_ws

echo "Installing MicroXRCEDDSGen..."
sudo apt install -y default-jre
cd ~
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts" >> ~/.bashrc
export PATH=$PATH:$HOME/Micro-XRCE-DDS-Gen/scripts
cd ~/ngps_ws

echo "Verifying microxrceddsgen..."
microxrceddsgen -help

echo "Adding Gazebo rosdep source..."
sudo wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
    -O /etc/ros/rosdep/sources.list.d/00-gazebo.list

echo "export GZ_VERSION=harmonic" >> ~/.bashrc
export GZ_VERSION=harmonic

echo "Installing dependencies..."
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

echo "Building workspace..."
source ~/.bashrc
colcon build --packages-up-to ardupilot_gz_bringup

echo "source ~/ngps_ws/install/setup.bash" >> ~/.bashrc

echo "Done! Run: source ~/.bashrc"