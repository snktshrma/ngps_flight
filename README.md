# NGPS: Next-Generation Positioning System

<div align="center">

[![WIP](https://img.shields.io/badge/status-WIP-yellow)](https://github.com/snktshrma/ngps_flight)
[![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Compatible-orange)](https://ardupilot.org/)
<br />
[![Python](https://img.shields.io/badge/Python-3.10+-blue?logo=python&logoColor=white)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-blue?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![PyTorch](https://img.shields.io/badge/PyTorch-1.9+-orange?logo=pytorch&logoColor=white)](https://pytorch.org/)
<br />
[![OpenCV](https://img.shields.io/badge/OpenCV-4.5+-green?logo=opencv&logoColor=white)](https://opencv.org/)
[![LightGlue](https://img.shields.io/badge/LightGlue-SuperPoint-yellow)](https://github.com/cvg/LightGlue)

</div>

Next-Generation Positioning System (NGPS) for high-altitude drone navigation without GPS.

## Description

This codebase implements a visual geo-localization system for drones that matches down-facing camera images against satellite reference images using deep learning-based feature matching. The system provides absolute position estimates to correct drift in Visual-Inertial Odometry (VIO) systems.

### Main Components

- **ap_ngps_ros2**: ROS2 node that performs visual geo-localization by matching camera images to satellite reference images using LightGlue/SuperPoint deep learning features. Runs at 1-2 Hz.

- **ap_ukf**: Unscented Kalman Filter that fuses multiple sensor inputs:
  - NGPS absolute position (1-2 Hz)
  - VIO relative pose (10-20 Hz)
  - IMU data (high frequency)

  Outputs fused odometry at 10-20 Hz for flight control.

- **ap_vips**: Visual-Inertial Odometry system that provides high-frequency relative pose estimates optimized for high-altitude flight.

### How It Works

1. NGPS module matches real-time camera frames to a georeferenced satellite reference image
2. Provides absolute position estimates at low frequency (1-2 Hz)
3. UKF fuses NGPS absolute positions with high-frequency VIO estimates
4. Fused output sent to ArduPilot's EKF for final state estimation

## Demo

[Algorithm Demonstration Video](https://youtu.be/iHE6cFCccTA)

## Related Articles & Blogs

- [GSoC 2024: High Altitude Non-GPS Navigation](https://discuss.ardupilot.org/t/gsoc-2024-wrapping-up-high-altitude-non-gps-navigation/122905/1) - Initial GSoC project summary
- [Transformer & Optimization Based High Altitude GPS-Denied Fusion](https://discuss.ardupilot.org/t/transformer-optimization-based-high-altitude-gps-denied-fusion/134181/1) - Updated implementation and architecture details

## Related Projects

- [ap_nongps](https://github.com/snktshrma/ap_nongps) - Earlier prototype implementation with SIFT-based feature matching and optical flow methods

## TODO

- [ ] Add an intereactive initial guess interface
- [x] Add a fallback VO pipeline
- [ ] Add global optimisation for fusion
- [ ] Update AP to accept position and odometry as separate sources to be fused internally
- [ ] Add support for multiple reference images
- [ ] Optimize feature matching for faster performance
- [x] Add calibration tools and documentation
- [ ] Improve error handling and recovery
- [ ] Add more unit tests
- [x] Document configuration parameters
- [x] Add example launch files for different scenarios
- [ ] Performance profiling and optimization
- [ ] Convert lightglue model to jetson friendly compute capable
- [ ] Support for different camera models
- [ ] GTSAM and SFM support

## Installation

**Step 1: Install Docker Engine**

> ### For WSL2
> Setup WSL2 for ubuntu using these instructions: [https://learn.microsoft.com/en-us/windows/wsl/install#upgrade-version-from-wsl-1-to-wsl-2](https://learn.microsoft.com/en-us/windows/wsl/install#upgrade-version-from-wsl-1-to-wsl-2) and confirm WSL version is `2`.
>
> All steps mentioned in README remains same. Just while setting up docker, before you run `sudo systemctl status docker` to check if docker is running, test:
> ```bash
> $ ps -p 1 -o comm=
> # should show systemd enabled. Alternattively confirm:
> ```
>
> ```bash
> $ cat /etc/wsl.conf
> # It must be:
> [boot]
> systemd=true
> ```
> If not, update contents of file to enable `systemd` on WSL.
> 
> - Exit WSL2
> - wsl --shutdown
> - wsl --update
> - Run WSL again: `wsl --distribution Ubuntu` and confirm if it's using `systemd` this time and continue with docker instructions.

- Follow the official installation guide: [Install Docker Engine](https://docs.docker.com/engine/install/).
- Apply the Linux post-installation configuration as non-root user: [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/).

If you are running WSL2 on Windows:

- Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/) (not Docker Engine inside WSL).
- In Docker Desktop, select **Settings, Resources, WSL Integration** and enable your distro.
- Verify Docker works by running, `docker run hello-world`

**Step 2: NVIDIA Container Toolkit (optional, WSL2 users should skip this step)**

> If NVIDIA GPU is present.
- Install the toolkit: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
- Configure Docker to use the NVIDIA runtime and restart the daemon:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

**Step 3: Clone the repository**

```bash
mkdir -p ~/ngps_ws/src
cd ~/ngps_ws/src
git clone https://github.com/snktshrma/ngps_flight.git
```

**Step 4: Docker image**

Pull a prebuilt dev image:

```bash
docker pull snktshrma/ngps-vps-dev:latest
```

Or build locally:

```bash
cd ~/ngps_ws/src/ngps_flight
docker build -f Dockerfile.dev -t vps-dev:latest .
```

Use `snktshrma/ngps-vps-dev:latest` in the commands below, or `vps-dev:latest` if built locally.

> **Docker only:** Skip **Steps 5–6**.
>
> With NVIDIA GPU (requires **Step 2**):
>
> ```bash
> docker run -it \
>   --name vps-dev \
>   --network host \
>   --privileged \
>   --ipc host \
>   --gpus all \
>   -v ~/ngps_ws:/home/dev/ngps_ws \
>   -e DISPLAY=$DISPLAY \
>   -v /tmp/.X11-unix:/tmp/.X11-unix \
>   snktshrma/ngps-vps-dev:latest
> ```
>
> Without NVIDIA GPU:
>
> ```bash
> docker run -it \
>   --name vps-dev \
>   --network host \
>   --privileged \
>   --ipc host \
>   -v ~/ngps_ws:/home/dev/ngps_ws \
>   -e DISPLAY=$DISPLAY \
>   -v /tmp/.X11-unix:/tmp/.X11-unix \
>   snktshrma/ngps-vps-dev:latest
> ```
>
> To start again:
>
> ```bash
> docker start vps-dev
> docker exec -it vps-dev /bin/bash
> ```

**Step 5: Install Distrobox**

For Ubuntu:

```bash
sudo apt install distrobox

export DBX_CONTAINER_MANAGER=docker
echo "export DBX_CONTAINER_MANAGER=docker" >> ~/.bashrc
```

**Step 6: Create the Distrobox**

With NVIDIA GPU (requires **Step 2**):

```bash
distrobox create \
  --name vps-dev \
  --image snktshrma/ngps-vps-dev:latest \
  --additional-flags "--privileged --ipc=host --gpus all"
```

Without GPU:

```bash
distrobox create \
  --name vps-dev \
  --image snktshrma/ngps-vps-dev:latest \
  --additional-flags "--privileged --ipc=host"
```

**Step 7: Enter and initialize the workspace**

```bash
distrobox enter vps-dev
```

Inside the container:

```bash
bash ~/ngps_ws/src/ngps_flight/setup.sh
# When done,
source ~/.bashrc
```

> **Distrobox:** The host user home directory is mounted; workspace paths such as `~/ngps_ws` match the host, while binaries and libraries resolve from the container image.

**Step 8: Verify the simulation stack**

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

- Expected result: ArduPilot DDS and Gazebo Harmonic start with the runway simulation.

#### Removing or reconnecting

**Distrobox:**

```bash
distrobox stop vps-dev
distrobox rm vps-dev
```
**Docker**:

```bash
docker stop vps-dev
docker rm vps-dev
```

---

### Package-level setup

See individual package READMEs:

- [ap_ngps_ros2/README.md](ap_ngps_ros2/README.md)
- [ap_ukf/README.md](ap_ukf/README.md)
- [ap_vips/README.md](ap_vips/README.md)

## Setting up environment variables
```bash
export MAPBOX_API_KEY=''
```
## Running the package

### Run SITL with DDS:
#### Terminal 1- Run micro ros agent:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

#### Terminal 2- Run SITL with DDS:
```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter --enable-DDS -DG --location OSRF0
```


Then in another terminal, **after GPS is detected in sitl**, run:

```bash
python3 ./Tools/autotest/sat_cam_emulator.py --port 14550 --airfield-radius-m 1500 --airfield-zoom 20  --http-mjpeg-port 8090 --no-hud --ros --no-display --ros-compressed --ros-size 640x360 --pose-source sim
```

> If want to record bag file:
> 
> ```bash
> ros2 bag record -o <location> /camera/image_raw/compressed   /ap/imu/experimental/data   /ap/clock   /ap/tf_static /ap/navsat /ap/gps_global_origin/filtered /ap/geopose/filtered /ap/time /ap/tf /ap/pose/filtered
> ```
> 
> To replay:
> 
> ```bash
> ros2 bag play <bag location>
> ```

#### Now to run our VPS (with bag for debugging or with SITL (with non-GPS EKF params for realtime test),

Set the .tif file location in `[config file here](ngps_flight/ap_ngps_ros2/config/ngps_config.yaml)`

```bash
ros2 launch ap_ngps_ros2 ngps_localization.launch.py
```

## For changing location
To change location and get new .tif for that location, please follow steps in the gazebo_terrain_generator fork specifically for this: [https://github.com/snktshrma/gazebo_terrain_generator/tree/dev/geotiff](https://github.com/snktshrma/gazebo_terrain_generator/tree/dev/geotiff)

After changing, please change the location for sitl launch as well.

> ## NOTE
> For now the steps to generate a .tif are very manual but addition to sat_camemulator.py already sets a base to automaticallyt manage and autogenerate .TIF using MAPBOX. So in next updates, I'll add that feature as well and that will also help wiith setting initial guess.

## Documentation

- [Changelog](CHANGELOG.md) - Project history and version timeline
- [Camera-IMU Calibration](CAMERA_IMU_CALIBRATION.md) - [Google Docs](https://docs.google.com/document/d/13JY4MAfdqjsa-Oa39xT4HW6WFFT-KQloymyGb0LvuGo/edit?tab=t.0#heading=h.43j03vqklwxn)
- [Non-GPS Navigation Setup](NON_GPS_NAVIGATION.md) - [Google Docs](https://docs.google.com/document/d/1Opsji8ZT2YeRjR8lPMAb49Ai44oMDmwoFYnSJPwfXz0/edit?tab=t.0#heading=h.d9zawrpqff1s)

## Special mentions

Ofcourse, ~~ChatGPT~~ Gemma4 (running locally) :)

## Safety & Ethical Considerations

**IMPORTANT DISCLAIMER:** This software is provided for research and educational applications only. The developers and contributors of this project doesn't promote and are not responsible for any misuse.

## License

See LICENSE file.
