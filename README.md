# slam_ws — G1 depth-only navigation

Autonomous navigation for a Unitree G1 using a depth camera, IMU and joint
encoders — no LiDAR.

**For the reasoning behind any decision here, read [`INSTRUCTIONS.md`](INSTRUCTIONS.md).**
This file is just how to run things.

## Environment

| | |
|---|---|
| Ubuntu | 24.04 |
| ROS 2 | Jazzy |
| Isaac Sim | 5.1.0 (not currently used — see "Transition" below) |
| Navigation2 | jazzy branch |
| RTAB-Map, RealSense SDK | vendored via `slam_ws.repos` |
| Robot | `192.168.123.164` over `eno1` (`192.168.123.1/24`), key-based SSH |
| Head camera | RealSense **D435i**, USB 3.2 |

## Clone and build

```bash
mkdir -p ~/workspaces/slam_ws/src
cd ~/workspaces/slam_ws
vcs import src < slam_ws.repos

source /opt/ros/jazzy/setup.bash
./setup.sh          # asserts ROS_DISTRO, vcs import, rosdep, colcon build
```

This makes the workspace much easier to recreate months later, or on another
machine. Rebuilding a single package:

```bash
colcon build --packages-select g1_perception --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Sourcing

Only source the workspace you are actively working in.

```bash
source ~/workspaces/slam_ws/install/setup.bash      # this project
source ~/Documents/manipulation/moveit2demo/ros2_ws/install/setup.bash   # MoveIt
source ~/g1_ws/install/setup.bash                   # g1_ws
```

## Running on the real robot

Three terminals. Order matters only in that the ROS side warns (and recovers)
if the others are not up yet.

```bash
# 1. this PC — joints + both IMUs, over the Unitree DDS
~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1

# 2. robot — camera, 848x480 @ 30 Hz
ssh unitree@192.168.123.164
conda activate teleimager
cd ~/image_server
./run_slam.sh

# 3. this PC — ROS 2
source ~/workspaces/slam_ws/install/setup.bash
ros2 launch g1_bringup sensors.launch.py            # add rviz:=true or cloud:=true
```

Verify everything in one command:

```bash
ros2 run g1_bringup check_sensors                   # 36 checks, exits non-zero on failure
```

Expected: `/imu/data` 177 Hz, `/joint_states` 88 Hz, camera 30 Hz, `/tf` 88 Hz,
single TF tree rooted at `base_footprint`.

### SSH tunnel (not normally needed)

```bash
ssh -L 5556:127.0.0.1:5556 unitree@192.168.123.164
```

Only for reaching the camera stream when the direct route is unavailable — off
the `192.168.123.x` link, or if the server is ever bound to loopback. It is not
the normal path: `image_server` binds `tcp://*:5556` and everything here talks
straight to `192.168.123.164:5556`. Prefer the direct route, because the tunnel
puts a ~19 MB/s stream through SSH encryption on both ends, and Jetson CPU is
the resource that limits the frame rate. If you do use it, point the bridge at
the tunnel:

```bash
ros2 launch g1_bringup sensors.launch.py camera_address:=127.0.0.1
```

## Monitoring the camera outside ROS

```bash
cd ~/workspaces/unitree
source camera_viewer/bin/activate
python live_view.py        # one colour feed, one depth feed
```

Read-only ZMQ subscriber; it cannot disturb the ROS stack.
Keys: `q` quit, `s` save PNG + raw-millimetre `.npy`, `c` cycle colormap.

## Model only, no robot

```bash
ros2 launch g1_description description.launch.py use_sim:=true
```

---

## Transition: what changed and why

The workspace began as an Isaac Sim / teleop setup and has moved to the real
robot. Four changes are worth knowing about, because the old commands still
exist and will mislead you.

**1. `run_slam.sh` on the robot, not `run.sh`.**
`run.sh` starts `image_server.py`, which serves teleop: it stitches a depth
*colormap* panel beside the colour image and streams 1280×720. That ran at
**7.2 Hz** — profiling showed 126 ms/frame of Jetson CPU in `rs.align` plus the
depth filters and the colormap render, against 0.3 ms of network. `run_slam.sh`
starts `slam_image_server.py`: 848×480, no colormap panel, no hole-filling
filter, **29.8 Hz**. Both files exist; `image_server.py` is untouched so teleop
still works.

**2. `g1_state_server`, not `g1_sensor_reader`.**
`g1_sensor_reader` only prints. `g1_state_server` publishes the same
`rt/lowstate` + `rt/secondary_imu` data over ZMQ/TCP for the ROS bridge. It must
be a separate process because `unitree_sdk2` and ROS 2 each bundle their own
CycloneDDS and corrupt the heap in one address space.

**3. `g1_bringup sensors.launch.py`, not `g1_description description.launch.py`.**
The description launch only publishes the model. The bringup launch adds the
sensor bridges and sets `robot_state_publisher publish_frequency:=200` — without
that, TF republishes at its 20 Hz default regardless of how fast joints arrive.

**4. Isaac Sim is not in the loop right now.**
`G1_nav.usd` still sits at the repo root and the description launch still has
`use_sim`, but all current work is against the physical robot. Anything
defaulting to `use_sim_time:=true` will hang on TF lookups here, because
nothing publishes `/clock`.

Also removed: `g1_bringup.launch.py` (targeted ROS 2 Humble and included
`livox_ros_driver2` / `open3d_loc`, neither of which exists in this workspace)
and the static `base_link → pelvis` transform that placed `base_link` at the old
MID360 LiDAR position on the head.
