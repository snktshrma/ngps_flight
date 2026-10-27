# Installation

Target platform: NVIDIA Jetson Orin with **JetPack 7.2** (L4T r39.x, Ubuntu 24.04, CUDA 13.2).
The container stack is ROS 2 Jazzy + TensorRT 10.16.2 + PyTorch cu130 (`Dockerfile.jp7.dev`).

**Step 1: Install Docker Engine**

- Follow the official installation guide: [Install Docker Engine](https://docs.docker.com/engine/install/).
- Apply the Linux post-installation configuration as non-root user: [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/).

**Step 2: NVIDIA Container Toolkit**

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

The default branch (`main`) is the one to use - the repo also carries a stale `master`.

**Step 4: Build the Docker image** (30–60 min on device)

```bash
cd ~/ngps_ws/src/ngps_flight
docker build -f Dockerfile.jp7.dev -t ngps-vps-dev-arm:jp72-cu132-jazzy-v2 .
```

> If the build fails on apt/curl steps with DNS or timeout errors, add `--network=host`.
> Docker's default bridge cannot use a `127.0.0.53` systemd-resolved stub and falls back
> to public DNS, which some networks block.

> **Docker only** (skip Steps 5–6). Distrobox is recommended - it handles the home mount and
> display/audio passthrough - but plain Docker works and needs no `--init-hooks`, because the
> image's user is already in the GPU groups.
>
> ```bash
> docker run -it \
>   --name vps-jp7 \
>   --network host \
>   --ipc host \
>   --runtime nvidia \
>   -u $(id -u) \
>   -v "$HOME:$HOME" -w "$HOME/ngps_ws" \
>   -e DISPLAY="$DISPLAY" -e XAUTHORITY="$XAUTHORITY" \
>   -v /tmp/.X11-unix:/tmp/.X11-unix \
>   --device /dev/ttyACM0 \
>   ngps-vps-dev-arm:jp72-cu132-jazzy-v2
> ```
>
> Mount the home directory at the **same path** as on the host (`$HOME:$HOME`). The workspace
> build and `ap_ngps_ros2/config/ngps_config.yaml` store absolute paths, so remapping the
> mount point (e.g. to `/home/dev/ngps_ws`) breaks engine and reference-image lookups.
>
> Drop `--device` if no flight controller is attached. GUI apps may also need `xhost +local:`
> on the host.
>
> To start again:
>
> ```bash
> docker start vps-jp7
> docker exec -it -u $(id -u) -w "$HOME/ngps_ws" vps-jp7 /bin/bash
> ```

**Step 5: Install Distrobox**

For Ubuntu:

```bash
sudo apt install distrobox

mkdir -p ~/.config/distrobox
printf 'container_manager="docker"\n' > ~/.config/distrobox/distrobox.conf
```

> Distrobox autodetects podman first and only falls back to docker, so if podman is
> installed it will pick the wrong one. The config file pins the choice for every
> invocation - including the non-interactive `scripts/run_*.sh` launchers, which an
> `export DBX_CONTAINER_MANAGER=docker` in `~/.bashrc` would not reach (bash skips
> `~/.bashrc` in non-interactive shells, and zsh never reads it at all).

**Step 6: Create the Distrobox**

```bash
distrobox create \
  --name vps-jp7 \
  --image ngps-vps-dev-arm:jp72-cu132-jazzy-v2 \
  --additional-flags "--runtime nvidia --ipc=host" \
  --init-hooks "usermod -aG video,debug,render,dialout $USER 2>/dev/null || true"
```

> The `--init-hooks` line is required for GPU access. Distrobox replaces the image's user
> with one matching the host, dropping its group memberships, and the Jetson GPU device
> nodes are group-restricted - `/dev/nvgpu/igpu0/{ctxsw,dbg}` to `debug` (GID 982) and the
> rest to `video` (44) / `render` (993). The image defines these groups at the host GIDs, so
> the hook only needs to add the user to them. Without it CUDA fails with
> `cuInit: operation not supported`.

> **Distrobox:** The host user home directory is mounted; workspace paths such as `~/ngps_ws` match the host, while binaries and libraries resolve from the container image.

**Step 7: Import sources and build the workspace**

```bash
distrobox enter vps-jp7
```

Everything below runs **inside the container** - it provides `vcs`, `colcon` and the ROS
toolchain, so nothing extra is needed on the host.

```bash
# Verify the image:
python3 -c 'import torch, cv2, lightglue, tensorrt; print("ok", torch.cuda.is_available())'

cd ~/ngps_ws
vcs import --recursive --input src/ngps_flight/ros2.jazzy.repos src

source /opt/ros/jazzy/setup.bash
colcon build --packages-select micro_ros_msgs micro_ros_agent ardupilot_msgs ap_ngps_ros2
```

**Step 8: Build SITL with DDS** (inside the container):

```bash
cd ~/ngps_ws/src/ardupilot
./waf configure --board sitl --enable-DDS && ./waf copter
```

**Step 9: Verify** (from the host):

```bash
~/ngps_ws/src/ngps_flight/scripts/run_sitl_stack.sh
```

`/ap/*` topics appear once MAVProxy connects to SITL:

```bash
~/ngps_ws/src/ngps_flight/scripts/_distrobox_ros.sh ros2 topic list
```

> Keep `ROS_DOMAIN_ID` unset (or match it to the `DDS_DOMAIN_ID` parameter, default 0).

#### Removing or reconnecting

**Distrobox:**

```bash
distrobox stop vps-jp7
distrobox rm vps-jp7
```
**Docker**:

```bash
docker stop vps-jp7
docker rm vps-jp7
```

---

### Package-level setup

See individual package READMEs:

- [ap_ngps_ros2/README.md](ap_ngps_ros2/README.md)
- [ap_ukf/README.md](ap_ukf/README.md)
- [ap_vo/README.md](ap_vo/README.md)
- [ap_vo2/README.md](ap_vo2/README.md)
