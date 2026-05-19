#!/bin/bash
sudo -v

set -e

echo "Setting up ngps_ws :)"
cd ~/ngps_ws

echo "Importing ros2 repos..."
vcs import --recursive --input ~/ngps_ws/src/ngps_flight/ros2.repos src

echo "Importing gazebo repos..."
vcs import --recursive --input ~/ngps_ws/src/ngps_flight/ros2_gz.repos src

echo "Installing pymavlink..."
pip install --user pymavlink --index-url https://pypi.org/simple/
pip install --user PyYAML mavproxy --index-url https://pypi.org/simple/
pip3 install --user --upgrade geocoder dronecan junitparser wsproto tabulate pygame intelhex --index-url https://pypi.org/simple/

echo "Installing ArduPilot prerequisites..."
cd ~/ngps_ws/src/ardupilot
Tools/environment_install/install-prereqs-jp6.sh -y
source ~/.bashrc
cd ~/ngps_ws

echo "Installing MicroXRCEDDSGen..."
sudo apt install -y default-jre
cd ~
if [ ! -d "Micro-XRCE-DDS-Gen" ]; then
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
else
    echo "Micro-XRCE-DDS-Gen directory already exists. Skipping clone."
fi
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
MAKEFLAGS="-j2" colcon build --packages-up-to ardupilot_gz_bringup --executor sequential

echo "source ~/ngps_ws/install/setup.bash" >> ~/.bashrc

sudo usermod -aG video $USER
sudo usermod -aG render $USER || true

echo "Done! Exit the Distrobox, enter again and then Run: source ~/.bashrc"
