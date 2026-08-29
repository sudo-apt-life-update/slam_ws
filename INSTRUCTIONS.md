# G1 Depth-Only Navigation — Design Notes

The reasoning behind this workspace: what we are building, the order we are
building it in, and **why** each decision was made. Deliberately light on code —
read this to understand the system, then read the source for detail.

Keep this file updated whenever a new decision is made.

---

## 1. The goal

Navigate a Unitree G1 autonomously using **only**:

- a depth camera (head-mounted RealSense D435i)
- an IMU (the G1's pelvis IMU)
- joint encoders

**No LiDAR.** The end product is a layered map: a 3D point cloud plus a 2D
occupancy grid, with more layers if needed.

## 2. The order of work, and why

Each stage depends on the one before it being *provably* correct. Getting this
order wrong is how people spend weeks debugging a SLAM stack whose real problem
is a 2 cm extrinsic error.

| # | Stage | Status | Why it comes here |
|---|-------|--------|-------------------|
| 0 | Find out what the sensors actually publish | done | You cannot design against a stream you have not inspected. This stage overturned two of our assumptions. |
| 1 | Get joints + IMU into ROS correctly | done | Fusion needs these before it needs the camera. |
| 2 | Get colour + aligned depth + CameraInfo into ROS | done | RTAB-Map cannot use images without CameraInfo. |
| 3 | Fix the TF tree | done | Every later stage expresses its result in a frame. A wrong frame silently corrupts everything downstream. |
| 4 | Automated sensor/TF check | done | So a regression is caught in 8 seconds, not after a bad map. |
| 5 | Geometric verification | done | Found a 16° camera extrinsic error that 36 passing tests had missed (§9). |
| 6 | Odometry, `odom -> base_link` | done | **Not** the EKF originally planned — see below. |
| 7 | `/scan` from depth | done | AMCL needs a LaserScan, and getting one from a tilted camera is not the obvious conversion. |
| 8 | Localisation, `map -> odom` | **switched to RTAB-Map, tag-anchored, working** | AMCL held a tight lock once seeded but could not hold position through a long feature-poor corridor (measured: σ_x grew past 1.7 m over 6 m of travel — a real geometric-observability limit, not a config bug). Switched to RTAB-Map's own localisation mode plus two AprilTags as periodic hard re-anchors (§11b). |
| 8b | Build a usable map | done | Four slam_toolbox walks produced zero loop closures. RTAB-Map closed it on walk 5, and reprocessing raised that to 635, then to 123 after removing the tag's mapping-time influence (§11). |
| 9 | Nav2: costmaps, planner, controller | **wired; rotates correctly, does not walk reliably** | `g1_navigation/config/nav2_params.yaml` + `g1_bringup/launch/nav2_navigation.launch.py`. `nav2_navfn_planner` with A*, as planned. Fixed a real rotation-speed mismatch (RPP commanding 3.6x the velocity_smoother's cap); confirmed a genuine ~180° turn afterward. But sustained forward progress still fails — 14 "Failed to make progress" events, ~0.32 m net movement over ~300 s. Root cause not yet found; needs live telemetry, not log archaeology (§11c). |
| 10 | The layered map (3D cloud + 2D grid) | pending | Comes largely free with RTAB-Map: the 2D grid is `/map`, and the 3D cloud exports from the same database with `rtabmap-export --cloud`. |

**Two plans changed as evidence arrived, and both changes made the work
smaller.** Worth recording, because the original plan is the thing a reader
would otherwise assume:

- **Stage 6 was going to be an EKF** fusing the IMU with leg odometry written
  from scratch. It turned out the robot already publishes its own leg+IMU state
  estimate on `rt/odommodestate`, found by enumerating DDS discovery rather
  than guessing topic names. Nothing needed writing.
- **Stage 7 was going to be RTAB-Map building a map.** Instead there is a prior
  map from an E57 survey scan, so the job is localising against a known map,
  not building one. RTAB-Map may still return for the 3D layer.

## 3. Shape of the system

```
   ROBOT (192.168.123.164, Jetson)          THIS PC (192.168.123.1, eno1)
   ┌───────────────────────────┐            ┌────────────────────────────┐
   │ slam_image_server.py      │  ZMQ :5556 │ g1_camera_bridge  ──► ROS  │
   │   D435i, depth aligned    │───────────►│   colour + depth + Info    │
   └───────────────────────────┘            │                            │
                                            │ g1_state_bridge   ──► ROS  │
   ┌───────────────────────────┐  ZMQ :5557 │   joints + both IMUs       │
   │ G1 control DDS            │◄───────────│ g1_state_server (C++, DDS) │
   │   rt/lowstate ~1050 Hz    │            │                            │
   │   rt/secondary_imu        │  ZMQ :5560 │ odom_bridge       ──► ROS  │
   │   rt/odommodestate ~500Hz │◄───────────│   /odom + odom->base_foot  │
   │                           │  ZMQ :5558 │ loco_bridge       ◄── ROS  │
   │   loco RPC (sport)        │◄───────────│   /cmd_vel  [g1_loco_server]│
   └───────────────────────────┘            │ robot_state_publisher ► TF │
                                            └────────────────────────────┘
```

### Port allocation — and why they must not collide

Every bridge is a ZMQ socket on localhost, and a second process binding an
already-taken port aborts. Two of these were assigned by different pieces of
work, so the list is worth keeping in one place:

| Port | Bound by | Carries | Direction |
|---|---|---|---|
| **5556** | `slam_image_server.py` *(on the robot)* | colour JPEG + aligned depth + intrinsics | robot → PC |
| **5557** | `g1_state_server` *(in `~/workspaces/unitree`)* | 29 joints + both IMUs, 472-byte packet | PC-local |
| **5558** | `g1_loco_server` | velocity commands and their replies | PC-local |
| **5560** | `g1_loco_server` | `rt/odommodestate` as a 68-byte odometry packet | PC-local |

Notes that matter:

- **5556 is the only one that crosses the network.** The rest are `127.0.0.1`,
  because both servers run on this PC and only the DDS side talks to the robot.
- **5557 is shared with `waypoint_follow_ws`.** Its `state_bridge.py` reads the
  same port from the same `g1_state_server` binary. That binary lives outside
  both ROS workspaces, in `~/workspaces/unitree/unitree_sdk2/build/bin/`, so
  neither workspace is truly standalone and both must agree on 5557.
- **5559 is deliberately free** — it is the documented fallback if 5558 is
  taken (`g1_loco_server --bind`, then `port:=5559` on the ROS side).
- A collision does not fail gracefully by default. Both `g1_state_server` and
  `g1_loco_server` now catch the bind error and print which port and how to
  clear it; without that you get a raw `zmq::error_t` and a core dump. Check
  with `ss -ltnp | grep 55`.

### Two workspaces, and what was copied

`waypoint_follow_ws` solved odometry and velocity control first. Rather than
overlay the two workspaces at runtime, its locomotion pieces were **copied**
into `slam_ws` on 2026-08-19 so each workspace runs independently:

| Copied | Into |
|---|---|
| `g1_loco_server` (C++) + `g1_odom_probe` | `src/g1_nav/g1_loco_server` |
| `odom_bridge`, `loco_bridge`, `loco_cli`, `loco_protocol`, `single_instance`, `fake_loco_server` | `src/g1_nav/g1_locomotion` |

`waypoint_follow_ws` is the original and is left untouched. Every copied file
carries a provenance header naming its source and date. **The copies will
drift** — a fix in one does not reach the other, exactly as the two copies of
`g1_29dof.urdf` already risk. Check both when changing anything in them.

Not copied: `path_follower`, `path`, `check_path` and `config/paths` — that is
the waypoint-following application, and Nav2 replaces it.

`fake_loco_server` was worth taking: it stands in for the robot, so the whole
`/cmd_vel` path can be exercised with the robot switched off.

### Why two processes instead of one ROS node

`unitree_sdk2` and ROS 2 each bundle their own CycloneDDS. Loaded into one
process they clash and corrupt the heap. So anything touching Unitree DDS lives
in a separate binary (`g1_state_server`) and talks to ROS over a local socket.

This is not optional; it is the reason the bridge pattern exists at all.

**Related trap:** even as a separate binary, the loader will pair the SDK's
`libddscxx` with *ROS's* `libddsc` if ROS is sourced, because `LD_LIBRARY_PATH`
beats `RPATH`. That aborts with `corrupted size vs. prev_size`. We pin the SDK's
own library with `DT_RPATH` (`-Wl,--disable-new-dtags`). Check any SDK binary
with `ldd <binary> | grep ddsc` — both must resolve under `unitree_sdk2/thirdparty`.

### Why TCP (via ZeroMQ)

Chosen over UDP for framing, ordering and automatic reconnect. The one hazard is
that TCP queues stale data under load and injects latency into a state stream, so
both subscribers set `CONFLATE=1` and `RCVHWM=1`: always read the newest sample,
drop the rest. For state and camera data, fresh beats complete.

## 4. Frames — the decisions

### Root is now `base_footprint`, not `pelvis`

Originally the tree was rooted at `pelvis` with no world anchor at all, so
nothing could express "where the robot is". We added:

```
base_footprint ──0.785 m──► base_link ──identity──► pelvis ──► (the robot)
```

- **`base_link` is coincident with `pelvis`.** The pelvis is the floating base
  that *both* the IMU and the joint encoders are referenced to. Making them
  identical means there is no extrinsic to get wrong. The alternative
  (`base_link` at the torso) would shorten the camera chain but add an
  extrinsic between the IMU and the navigation frame — a bad trade.
- **`base_footprint` is the ground projection**, which the 2D occupancy layer
  needs.

**We deleted a legacy static transform** that placed `base_link` at the
head-mounted MID360 LiDAR position. That was correct for the previous FAST-LIO
stack, but here it would put every sensor ~0.46 m too high — and now that the
URDF defines `base_link` itself, it would also be a second publisher for the
same edge.

### Where 0.785 m comes from

Measured, not assumed:

| Source | Value |
|---|---|
| Standing robot, live TF `pelvis -> ankle_roll_link` | −0.74998 m |
| Foot contact spheres (`z=-0.03`, radius `0.005`) | a further 0.035 m |
| **Total** | **0.785 m** |
| URDF zero-pose prediction, as a cross-check | −0.75219 → 0.787 m |

The two agree to ~2 mm, which tells us something useful: **the URDF's zero pose
is the standing pose.** An earlier reading of −0.587 m was simply the robot
crouched in damping mode — not a model error.

> **Known limitation.** This offset is *fixed*. A walking humanoid's pelvis
> height varies with stance. It is good enough to validate TF and seed the 2D
> grid, but stage 6 should replace it with a node that projects `base_link` onto
> the ground using the lower foot's current z.

### The camera optical chain

The URDF had `d435_link` and an identity-mapped `camera_frame`, with **no
optical-frame rotation anywhere**. Camera drivers publish images in *optical*
convention (z forward, x right, y down), while robot frames use REP-103 (x
forward, y left, z up). Without the rotation, every point cloud comes out
rotated 90°. We added:

```
d435_link ≡ camera_link → camera_color_frame  (y +0.015) → camera_color_optical_frame  (rpy −π/2, 0, −π/2)
                        → camera_depth_frame              → camera_depth_optical_frame
```

**`d435_link` *is* the RealSense `camera_link`** — it is not an approximation.
`d435_joint`'s y-offset is `0.01753`, and `realsense2_description` places
`camera_link` at `d435_cam_depth_py = 0.0175` from the mount. Live TF confirmed
`+0.0174`. So the chain hangs directly off `d435_link`; stacking the
`bottom_screw_frame` offset on top would double-count it.

Offsets and the rotation are taken from
`realsense2_description/urdf/_d435.urdf.xacro` rather than invented.

Colour and aligned depth **both** use `camera_color_optical_frame`, because the
robot aligns depth to colour and reports the colour module's intrinsics.

### The camera mount angle is calibrated, not taken from the URDF

Unitree's stock URDF has `d435_joint rpy = 0 0.8308 0` — a 47.6° downward tilt.
**That number does not describe this robot.** The camera was re-angled
mechanically on 2026-08-08, and the stock value was already wrong before that.

Measured value: **19.16° ± 0.12** (12 samples). `d435_joint rpy y = 0.3344213`.

**How it is measured** (`measure_camera_pitch.py`) — two independent sensors
must agree, and the method needs no prior knowledge of the mount:

1. Fit the floor plane in the depth image, RANSAC on the lower image rows, so a
   cluttered room does not fool it. That yields *world-up* in the camera's
   optical frame. With the standard optical rotation, a camera pitched θ below
   horizontal sees up at `[0, −cos θ, −sin θ]`, so `θ = atan2(−u_z, −u_y)`.
2. The IMU gives the robot's own lean. Subtract it, and what remains is the
   camera angle relative to the torso, which is exactly what `d435_joint` encodes.

Cross-checked by geometry: the optical axis should meet the floor at
`height / sin θ` = 2.81 m, against an observed median depth of 2.66 m.

> **Re-measure after any mechanical change to the mount.** The stock URDF value
> is not a fallback — it never matched.

Roll measured **+1.04°** but is left at 0. A 1° camera rotation is
indistinguishable from a 1° floor slope on gym matting. Revisit if maps show a
consistent lean.

#### Why the new angle is better

| | Old mount | New mount (current) |
|---|---|---|
| Pitch below torso | ~63.9° | **19.16°** |
| Depth range observed | 1.00–1.68 m | **0.94–8.29 m** |
| View span (43.1° V FOV) | 42°–85° below horizontal | **2.4° above horizon → 40.7° below** |

The old mount saw almost nothing but floor 1.5 m ahead. The new one sees walls,
obstacles and the horizon, which is what an occupancy map needs.

**The remaining trade:** the nearest visible floor is now ~1.47 m in front of
the robot, so the camera cannot see the ground at its own feet. Fine for
navigation and mapping; a gap if vision-based footstep planning is ever wanted.

#### The mount angle is a MECHANICAL SETTING, not a fixed property

The camera sits on a bracket that can be re-angled by hand. **What the robot
can see is therefore a design choice we control**, not a constraint to work
around, and it has already been changed once (63.9° → 19.16°, 2026-08-08).

Treat it as a tuning parameter with a real trade-off:

| Aim it lower | Aim it higher |
|---|---|
| sees the ground closer to the feet | sees further, and more wall |
| better for footstep planning and near-obstacle detection | better for localisation, which needs walls |
| loses range and wall coverage | blind closer in — currently blind inside 1.47 m |

If localisation turns out to be weak (see §9 on the narrow FOV), **raising the
camera angle is a legitimate fix** and may beat any amount of parameter
tuning: more wall in frame directly improves AMCL's observability. Equally, if
the robot starts tripping on things it cannot see, lower it.

> **Any mechanical adjustment invalidates the URDF.** `d435_joint` is
> calibrated, not nominal, so after touching the bracket you must re-measure
> and update it:
>
> ```
> ros2 launch g1_bringup sensors.launch.py
> python3 <scratchpad>/measure_camera_pitch.py 12
> ```
>
> Then write the reported value into `d435_joint rpy` and rebuild
> `g1_description`. Skipping this silently tilts every point cloud and every
> map built from it.

## 5. Camera pixel format — why 848×480

The stream ran at **7.2 Hz**. We profiled instead of guessing, and the guess
would have been wrong:

| Resolution | capture | encode | send | Rate | H FOV |
|---|---|---|---|---|---|
| 1280×720 | 126 ms | 12 ms | 0.3 ms | 7.2 Hz | 70.2° |
| 640×480 | 44 ms | 4 ms | 0.1 ms | 30.1 Hz | 55.6° |
| **848×480** | ~30 ms | 4 ms | 0.1 ms | **29.8 Hz** | **69.8°** |

Three conclusions:

1. **`send` is 0.3 ms — the network was never the bottleneck.** 1.97 MB crossed
   the link in 22 ms, ~716 Mbit/s of headroom. An early plan to PNG-compress
   depth was **abandoned**: it would have added encode work to the Jetson CPU,
   the one resource actually saturated, to save bandwidth we had in abundance.
2. **`capture` (`rs.align` + depth filters) dominates and is pixel-bound.**
   Cutting pixels is the whole fix.
3. **848×480 beats 640×480 outright.** Both hit ~30 Hz, but the D435 *crops
   horizontally* to make a 4:3 frame rather than gaining vertical — so 640×480
   throws away 21% of the horizontal field for nothing. (An earlier assumption
   that 4:3 would gain vertical FOV was wrong; measured V FOV is 43.1° either way.)

30 Hz is the camera's own ceiling at `fps=30`, so the loop is now
camera-limited rather than CPU-limited.

Resulting intrinsics: `fx=607.3 fy=607.3 cx=433.6 cy=244.9`, `depth_scale=0.001`
(depth is uint16 millimetres, the ROS convention).

### Depth filters: `light`, not `full`

The robot applies `spatial → temporal → hole_filling`. We dropped
**`hole_filling`**:

- It was why depth looked 99.9% dense. Without it, still ~97% — so almost all of
  that density was real and only ~2% was invented.
- It **fabricates depth in occluded regions**. In a map, fabricated depth becomes
  fabricated occupancy, which is worse than a hole.

`spatial` and `temporal` are kept: `temporal` in particular suppresses per-frame
depth flicker, which matters on a walking robot.

## 6. Other decisions worth knowing

**Reliable QoS on the image topics, not the conventional `SENSOR_DATA`.**
Best-effort lost **8% of depth frames** while the tiny `CameraInfo` arrived
100%. A 50-deep subscriber queue did not help, so it was not backpressure — a
1.84 MB image is split into many DDS fragments and losing any one drops the
whole sample. Reliable took it to 100%. It is also strictly more compatible: a
reliable publisher can feed best-effort subscribers, not the reverse.

**One timestamp per frame.** Colour, depth and both CameraInfos share a single
stamp taken from the server's capture time. The old bridge called
`get_clock().now()` separately per message, which destroys sync. Verified:
300/300 exact matches.

**`robot_state_publisher` needs `publish_frequency:=200`.** It does *not* simply
follow `/joint_states` — it republishes TF at `publish_frequency`, default
**20 Hz**. That default, not a slow joint stream, is why the original frame dumps
recorded 20.5 Hz. Now 88 Hz.

**Unitree quaternions are `(w,x,y,z)`; ROS wants `(x,y,z,w)`.** Reordered
explicitly and normalised. Getting this wrong yields a plausible-looking but
wrong orientation — the single most common G1 integration bug.

**IMU covariances are populated, not zero.** `robot_localization` reads them;
all-zero means "unknown". Current values (`1e-3` orientation, `1e-4` gyro,
`1e-2` accel) are **placeholders** and should be replaced with real values
measured from a stationary log before stage 6.

**The robot's own files are never modified.** `image_server.py` and `run.sh` are
untouched so teleop keeps working; the SLAM variant lives beside them as
`slam_image_server.py` + `run_slam.sh`.

### The two clocks must be synchronised, and `systemd-timesyncd` is not enough

The camera is stamped by the **robot's** clock (`time.time()` in the image
server); the IMU and joints are stamped by **this PC's** (`CLOCK_REALTIME` in
`g1_state_server`). If those clocks disagree, the two streams are misaligned by
the difference — and nothing in the data reveals it.

Measured before the fix: **the robot was 112 ms ahead of the PC.** At a 1 m/s
walk that is ~11 cm of misregistration straight into the EKF and the map, and it
would surface as unexplained drift rather than as a clock problem.

Both machines reported `System clock synchronized: yes`. That was true and
useless: each ran `systemd-timesyncd` against a *different* internet server
(`ntp.ubuntu.com` at +66 ms, `1.pool.ntp.org` at +31 ms). timesyncd is minimal
SNTP — it corrects slowly and leaves tens of milliseconds on the table. Two
independently drifting SNTP clients is exactly how you end up 112 ms apart while
both look healthy.

**The fix**: `chrony` on both, with the robot slaved to the PC over the direct
link rather than to the internet.

- PC `/etc/chrony/chrony.conf`: `allow 192.168.123.0/24` plus
  `local stratum 10`. The `local` line is what keeps the PC serving time when
  there is no internet — without it an unsynced PC refuses to serve and the
  robot silently drifts.
- Robot: `server 192.168.123.1 iburst prefer minpoll 2 maxpoll 4`. The
  short poll interval converges in under a minute instead of ~10.

Result: **19 µs**, a ~50x improvement, and the two will now stay together
because they share one reference.

Verify with `chronyc tracking` on the robot (`Reference ID` should decode to
192.168.123.1), or just watch the `frame age` row in `check_sensors` — it must
be a small **positive** number, around 20-30 ms of genuine transit time. A
negative age means a frame was stamped in the future, which is only possible if
the clocks disagree.

## 7. Bringing it up

```bash
# 1. this PC — DDS side (joints + IMU)
~/workspaces/unitree/unitree_sdk2/build/bin/g1_state_server eno1

# 2. robot — camera
ssh unitree@192.168.123.164 'cd ~/image_server && ./run_slam.sh'

# 3. this PC — ROS
ros2 launch g1_bringup sensors.launch.py
ros2 run g1_bringup check_sensors
```

## 8. What "correct" currently means

`check_sensors` passes 36/36:

| Signal | Rate | Frame |
|---|---|---|
| `/imu/data` | 177 Hz | `imu_in_pelvis` |
| `/imu_torso/data` | 177 Hz | `imu_in_torso` |
| `/joint_states` | 88 Hz | 29 joints, pos + vel + effort |
| `/camera/color/image_raw` (+ `camera_info`) | 30 Hz | `camera_color_optical_frame` |
| `/camera/aligned_depth_to_color/image_raw` (+ `camera_info`) | 30 Hz | `camera_color_optical_frame` |
| `/tf` | 88 Hz | single tree rooted at `base_footprint` |

Sanity: `|quaternion| = 1.000000`, `|accel| = 9.77 m/s²`, colour/depth stamp
delta `0.000 ms`, depth 96.7% valid over 1.00–1.67 m.

## 9. Geometric verification

Three of the four checks originally listed as "RViz only" turned out to be
automatable, and doing so **found a real bug that 36 passing tests had missed**
— the camera extrinsic was 16° out. Never treat a green test suite as proof
that geometry is right.

| Check | Status | Evidence |
|---|---|---|
| Optical frame orientation | **verified** | cloud lies flat and below the robot; view direction 19.2° below horizontal, image-up has +z |
| Extrinsic / mount angle | **verified** | camera-vs-IMU agreement, §4; floor lands at **+0.010 m** above ground |
| Gravity alignment | **verified** | residual floor tilt equals the robot's own lean, see below |
| IMU axis convention | **verified** | see below; both axes slope ≈ +1, inversion ruled out |

### IMU axes — verified, and what it took

`check_imu_axes` compares world-up as seen by the accelerometer against
world-up from the camera's floor plane, both in the pelvis frame, with the
robot held still at several roll angles.

Result from 380 samples (`~/.ros/g1_checks/imu_roll_20260810_113833.json`):

| axis | span | OLS slope | TLS slope | 95% interval | verdict |
|---|---|---|---|---|---|
| pitch | 0.074 | +0.849 | +0.903 | [+0.816, +0.881] | OK |
| roll | 0.051 | +0.867 | **+0.963** | [+0.824, +0.910] | OK |

Inversion needs a slope near −1 and is excluded by roughly 80 standard errors.
The best-fit rotation carrying camera-up onto IMU-up is **4.16°**; an inverted
axis would need ~180°.

**Three traps this walked into, all worth remembering.**

1. **Do not use the fused quaternion as the reference.** Its two plausible
   readings (world-from-body vs body-from-world) differ by a *sign flip on the
   axes under test*, so a wrong reading is indistinguishable from an inverted
   axis. Near upright the two candidates scored 0.93° and 0.61° against
   gravity — a coin toss, and it chose wrong. The accelerometer at rest has no
   such ambiguity. Once genuinely tilted the candidates separated by 15.5° and
   the convention resolved cleanly: **`row`**, i.e. world-up is the third
   *row* of the rotation matrix.
2. **Ordinary regression under-reads the slope.** The camera's floor normal is
   noisy, and noise in the *input* variable biases the fitted slope toward
   zero. Measured camera noise σ=0.0032 against a 0.051 span predicts an
   attenuation factor of 0.856 — which is exactly the gap between OLS 0.867
   and TLS 0.963. Use orthogonal regression, and judge on the confidence
   interval rather than a fixed span threshold. The first analysis reported
   "NOT ROLLED ENOUGH" on data that answered the question perfectly well.
3. **Do not do this while walking.** The robot is tethered by ethernet;
   walking produced 11 link drops in 90 s, several renegotiating at 100 Mbps —
   below the 152 Mbit/s the camera needs. The run measured a disconnected
   cable. Stationary also keeps the accelerometer valid.

The robot's body is stiff and only rolled ~2.9°. That was still ample: 380
samples pin the slope to ±0.02.

**Left over: a fixed 3.4° camera-to-IMU misalignment** (≈ +3.1° pitch, +1.3°
roll; the component about z is invisible to an up-vector comparison). The roll
part matches the +1.04° camera roll deliberately left at 0 in §4. This is not
an axis error and does not block the EKF, which takes attitude from the IMU —
but it will tilt the map by a few degrees, and the lab floor may itself not be
level. Revisit if maps come out leaning.

### The floor still looks tilted, and that is correct

After calibration the floor sits at +0.010 m but is still tilted **7.73°** in
`base_footprint`. The IMU independently reports the pelvis leaning **7.46°**
forward. Those match — the tilt is the robot's *real posture*, not a
calibration error.

TF cannot know about it: `base_footprint → base_link` is a fixed vertical
translation, and there is no `odom` frame applying the IMU attitude. **Stage 6
fixes this**, and until then any cloud expressed in `base_footprint` inherits
the robot's unmodelled lean. Do not chase it as a camera bug.

## 10. The prior map, and what it needs to display

Navigation runs against a **prior map** built from an E57 survey scan of the
building, not a map the robot made. It lives in
`g1_navigation/maps/` as `map.pgm` + `map.yaml`.

Validated 2026-08-19 by loading it with `nav2_map_server`:

```
293 x 342 px @ 0.05 m/px      origin (-11.45, -19.70)
covers x -11.45 .. 3.20 m,  y -19.70 .. -2.60 m       (14.7 x 17.1 m)
34.1% free   27.1% occupied   38.8% unknown
```

**`map.png` in the same folder is NOT a map** — it is 1370x710 RGBA with a blue
background and white points, i.e. a screenshot of a point-cloud viewer. Nav2
would load it without complaint and produce nonsense. `map.yaml` correctly
points at the `.pgm`; leave it that way.

### Three conditions for seeing a map in RViz2

1. **`map_server` is a lifecycle node.** It publishes nothing until it is both
   *configured* and *activated*, and it reports no error while idle — you just
   get a silent topic. Use `nav2_lifecycle_manager` (as the launch file does),
   or transition it by hand with `ros2 lifecycle set /map_server configure`
   then `activate`.
2. **QoS has to match.** The publisher is `TRANSIENT_LOCAL` / `RELIABLE`.
   RViz's Map display defaults to Volatile, which silently shows an empty grid.
   Set **Durability = Transient Local**.
3. **Fixed Frame = `map`.**

Until AMCL is running there is no `map -> odom` transform, so the map draws but
the robot is not placed on it. That is expected, not a fault.

## 11. Viewing the stream outside ROS

`~/workspaces/unitree/live_view.py` shows one colour feed and one depth feed.

The depth window is rendered **on the PC** from the `depth_raw` array already in
every packet — nothing is recomputed from stereo. That render used to happen on
the robot and cost it ~15 ms/frame, which is part of what capped the stream at
7 Hz. Moving it to the PC costs the robot nothing.

It auto-detects the format: against `image_server.py` it takes the left half of
the stitched frame; against `slam_image_server.py` the frame is already colour
only. (The old version split every frame in half and labelled the pieces
"Left Camera"/"Right Camera" — misleading, since it was showing colour beside a
picture of depth, never a stereo pair.)

Keys: `q` quit, `s` save a colour/depth PNG pair plus the raw mm as `.npy`,
`c` cycle colormap. The depth window prints the **centre-pixel distance**, which
is what you want when checking geometry against a tape measure.

### Turning depth into a LaserScan

AMCL wants a `LaserScan`. Getting one out of a downward-pitched depth camera
took two nodes, and the reason is worth keeping.

**Why not `depthimage_to_laserscan`.** It flattens a horizontal band of *image
rows*. That is right for a level camera and wrong for this one: pitched 19.16°
down, the row through the image centre points at the floor ~3.6 m ahead. The
"scan" would be a near-constant ring of floor readings, and AMCL would fail in
a way that looks like a tuning problem. `pointcloud_to_laserscan` filters by
height in a chosen *frame* instead, so what enters the scan depends on where a
point is in the world, not where it landed on the sensor.

**Why `base_stabilized` had to be invented.** A height filter needs a frame
whose z points at the sky, and the robot's tree has none.
`odom -> base_footprint` comes from the robot's own estimator and carries its
**full attitude**, so despite the name it tilts with the body; everything below
it is rigidly attached. With the G1's measured 7.5° stance lean, a wall point
4 m out sits `4·sin(7.5°) = 0.52 m` off its true height — a 0.3–1.5 m band
would swing by half a metre, admitting floor one moment and losing walls the
next.

`g1_locomotion/base_stabilizer.py` publishes `odom -> base_stabilized`: same
x/y, z on the ground plane, **yaw only**. Ranges then measure horizontal
distance and z measures true height. It reports the largest tilt it has
discarded, so the size of the problem stays visible.

> Levelling `base_footprint` itself would have been wrong. The URDF joins it to
> `base_link` with a *fixed* joint, so levelling one levels the other — and
> `base_link` is the robot body, which genuinely does tilt.

**The band: 0.30–1.50 m**, set by two independent constraints.

| Constraint | Range |
|---|---|
| The **map** (`make_dense_occupancy_v2.py`) calls a cell occupied from points | 0.12–2.20 m, with −0.04–0.10 m treated as floor |
| The **camera** can observe, at 4 m | floor up to ~1.43 m |

0.30 leaves 0.20 m of margin above the map's floor band for odometry drift and
uneven ground; 1.50 is past what the camera sees, which costs nothing and
leaves headroom if the bracket is raised. Anything the map holds between
0.12–0.30 m or above 1.5 m simply is not reported — AMCL tolerates *missing*
returns far better than spurious ones, which is the right way round.

Angles are ±35°, matching the real 69.8° FOV. Claiming more would fill the scan
with fabricated no-returns, and AMCL reads a max-range reading as positive
evidence of empty space; `use_inf: true` avoids that too.

Verified 2026-08-19 with a synthetic cloud containing floor at 0.05 m, a wall
at 0.80 m and ceiling at 2.50 m: 140 rays over ±35°, and only the wall
survived.

**Note the QoS.** `/scan` is published best-effort. A reliable subscriber
receives *nothing* from it and reports only a warning.

### AMCL against the prior map

`g1_navigation/config/amcl.yaml`. The tuning follows from one asymmetry:

| | |
|---|---|
| **Worse than usual** | AMCL assumes a ~270° LiDAR; we have a 70° wedge. One scan constrains only what is in front, so along a corridor the position *along* it is weakly observed, and global localisation from scratch does not converge. **Expect to give an initial pose.** |
| **Better than usual** | The motion model is unusually good. Odometry is the robot's own leg+IMU estimator, tape-measure validated. AMCL normally fights bad wheel odometry; here the scan only corrects slow drift. |

So the config trusts odometry and treats the scan as a gentle correction:

- **`base_frame_id: base_stabilized`**, not `base_footprint`. The scan lives in
  the level frame; pointing AMCL at the tilted one would interpret every range
  against a tilted axis.
- **`OmniMotionModel`**, not differential. The G1 side-steps under command, and
  drifts sideways when told to walk straight — 0.52 m of rightward drift over
  1.37 m forward in the validation run. A differential model has no term for
  that. `alpha5` (sideways) is set highest for the same reason.
- **`alpha1..4: 0.15`**, below stock. Inflating odometry noise spreads
  particles that 70° of view then cannot re-tighten.
- **800–3000 particles**, above the 500/2000 default: less information per scan
  means hypotheses must survive longer before one wins.
- **`likelihood_field`**, not `beam`. The beam model reasons about what each ray
  passes through, which needs a wide sweep and is far more sensitive to
  unmodelled obstacles — and this map is a survey scan of a room whose
  furniture has since moved.
- **Recovery enabled but slow** (`0.001` / `0.1`). A 70° view legitimately
  scores badly at times, such as when facing open space; aggressive random
  injection would scatter a good estimate. These react to a sustained collapse,
  not a momentary one.

Verified 2026-08-19 with a synthetic wall arc standing in for the camera:
publishing an initial pose at map `(-2.0, -8.0)` produced `map -> odom` and
`map -> base_stabilized` at exactly that pose. The `map -> odom` link Nav2
needs now exists.

**AMCL publishes no transform until it has processed a scan.** With the camera
off you get an active node, no error, and no `map` frame — which looks like a
failure and is not.

### The ROS 2 CLI hangs when the robot is unplugged

If `ros2 node list`, `ros2 topic` and `ros2 lifecycle` all block and time out,
check the link first: **CycloneDDS stalls trying to use `eno1` when that
interface is DOWN**, which is exactly the state whenever the robot is off.

```
ip -brief addr show eno1        # DOWN, and /sys/class/net/eno1/carrier == 0
```

Pin discovery to loopback for offline work and the CLI responds instantly:

```xml
<CycloneDDS><Domain id="any"><General>
  <Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces>
</General></Domain></CycloneDDS>
```

```
export CYCLONEDDS_URI=file:///path/to/that.xml
```

It looks like a broken ROS install or a hung node, and it is neither.

### Mapping: slam_toolbox was abandoned for RTAB-Map

`g1_navigation/config/rtabmap.yaml`, launched by
`g1_bringup/launch/rtabmap_mapping.launch.py`. `slam_toolbox.yaml` and
`mapping.launch.py` are kept for reference but are no longer the path.

**What went wrong.** AMCL could not localise against the surveyed map, so we
built our own with slam_toolbox. Four walks, and **loop closure never fired
once**:

| Walk | Extent | Occupied | Unknown | Outcome |
|---|---|---|---|---|
| 1 | 11.7 × 14.5 m | 3.1% | — | starburst |
| 2 | 26.2 × 24.3 m | 1.7% | 82.1% | AMCL failed — 62% of scan endpoints landed in unknown space |
| 3 (perimeter) | 28.1 × 21.8 m | 2.4% | 75.3% | open horseshoe, ends never joined |
| 4 (+ inner area) | 28.1 × 22.5 m | 3.2% | 67.1% | coverage improved, still no closure |

Coverage improved on every walk, which is the important detail: **the walking
was never the problem.** The matcher was.

**Why it could not have worked.** Karto, inside slam_toolbox, closes loops by
*geometric* scan correlation. Every assumption it makes about the scan is
violated here:

| | Karto expects | D435i on the G1 |
|---|---|---|
| FOV | ~270° | **70°** |
| Nearest return | ~0.1 m | **1.5 m** (bracket pitched 21.3° down) |
| Jitter at range | ~10 mm | **~350 mm** in open space |
| Rate | 40 Hz | **15 Hz** (USB limit) |

In a 15 × 15 m hall, a 70° wedge of sparse, distant, ±2-cell-noisy returns
gives the solver almost nothing to correlate. Drift then outran
`loop_search_maximum_distance` (3.0 m), so slam_toolbox stopped even
*searching* for a closure — the failure was silent, because it kept publishing
`map -> odom` throughout and the stack looked healthy.

**Why RTAB-Map is different.** It recognises a place by *appearance* — a
bag-of-words index over RGB features — not by the shape of a range wedge.
Geometry is used only to compute the transform once a place has already been
recognised, which a 70° FOV is perfectly adequate for. It also produces both
layers stage 10 asks for from one session: a 2D grid for Nav2, and an RGB-D
database that exports to a 3D cloud.

The settings that carry the most weight, all argued in the config file itself:

- **`Grid/RayTracing: true`** — marks cells *between* camera and hit as free.
  This is the direct fix for 67.1% unknown, which is what actually starved AMCL:
  it reads unknown space as no information at all, not as an obstacle.
- **`Reg/Force3DoF: true`** — zeroes z, roll and pitch on every graph link.
  Three of six degrees of freedom then cannot accumulate error at all, which
  matters more on a humanoid that bobs and leans every step than on a wheeled
  robot. (There is no companion `Optimizer/Slam2D` in 0.22.1; that name is from
  older docs and is **silently dropped**, with no warning in the log.)
- **`RGBD/MaxLoopClosureDistance: 0.0`** — deliberately *no* distance gate. The
  whole problem is that odometry has drifted far from truth by the time the
  robot returns; gating by distance rejects precisely the closures worth most.
  This is the direct counterpart of the slam_toolbox setting that killed walk 3.
- **`RGBD/NeighborLinkRefining: true`** — refines each odometry link visually as
  it is added, correcting leg-odometry drift inside the graph.
- **`frame_id: base_stabilized`** — unchanged reasoning from the scan pipeline.
  The grid is built by height-thresholding in this frame, so if it tilts, the
  ground/obstacle split tilts and the floor registers as an obstacle mid-stride.

**The walking advice changed with the method.** For scan matching it was "walk
closed circuits". For appearance matching, *returning to a spot facing the same
direction* matters more than the circuit — with a 70° FOV, walking back down a
corridor the other way shows the camera an entirely different scene. Point it
at texture; a blank wall at 2 m yields almost no features, and bare walls are
where this method is weakest exactly as open space is where scan matching is.

#### Walk 5 — the first map that closed

2026-08-22, the first walk under RTAB-Map, and the first closed map in five
attempts.

| | walk 4 (slam_toolbox) | walk 5 live | walk 5 reprocessed |
|---|---|---|---|
| loop closures | **0** | 4 | **635** (93 visual, 542 proximity) |
| occupied | 3.2% | 19.4% | 15.8% |
| free | — | 22.5% | 27.2% |
| unknown | 67.1% | 58.1% | 56.9% |
| extent | 28.1 × 22.5 m | 32.5 × 26.8 m | 28.7 × 23.6 m |

Two things in that table are worth reading carefully.

**The reprocessed run found 635 closures against the live run's 4.** Reprocessing
replays the recorded data with no real-time budget, so proximity detection gets
to run against an already-partly-optimised graph instead of against raw drifting
odometry. The lesson is not that the live settings are wrong — it is that
**`rtabmap-reprocess` is part of the workflow, not a debugging afterthought.**
Walk, then reprocess, then use the reprocessed database.

**The extent SHRANK, and that is the good direction.** A map that covers more
ground is not automatically better: the extra 3.8 × 3.2 m in the live map was
far-field noise inflating the bounding box, not discovered structure. Losing it
means the walls sit where the walls are.

Artifacts: `~/g1_maps/walk5_original.db` (as walked, preserved),
`walk5_tuned.db` (reprocessed), `rtab1.{pgm,yaml}` (live grid),
`rtab1_tuned.{pgm,yaml}` (reprocessed grid).

#### Two tuning faults walk 5 exposed

**`Grid/RangeMax` was too generous at 5.0 m.** The live map came out 19.4%
occupied with black fans of spurious obstacle spraying outward along the walk
path. Two effects compound at range and both scale with it: the ~350 mm depth
jitter measured in open space, and the fact that a small camera-pitch error
lifts distant *floor* points above the 0.15 m ground threshold, so the floor
itself starts registering as an obstacle. 3.5 m cuts the worst of both. This
governs the occupancy grid only — the cloud still carries 6 m, and loop closure
still uses features at any range.

**The landmark covariance was too tight, and cost us the tag entirely.** All
four AprilTag constraints were rejected during the live walk:

```
Loop closure 3222->-10 rejected!
  abs error=27.300287 deg, stddev=0.100000  ->  error ratio 4.76 (limit 3.0)
```

Node `-10` is the tag. At `landmark_sigma_angular: 0.10` rad (5.7°), a
17–27° orientation disagreement reads as a 3–5σ contradiction, and
`RGBD/OptimizeMaxError` vetoes the *whole* constraint — position included. So
the tag added to fix loop closure contributed nothing to the map that closed.

Worth noting how the diagnosis went, because it constrains what may be changed:
this gate rejected five constraints across the whole walk and **every one was
the landmark**; all 79 visual rejections came from the separate
`Vis/MinInliers` gate. That is what makes it safe to override
`RGBD/OptimizeMaxError` when reprocessing walk 5 — the bad covariance is baked
into the recorded landmarks and `rtabmap-reprocess` replays them as stored, so
the source fix cannot reach old data:

```
rtabmap-reprocess --Grid/RangeMax 3.5 --RGBD/OptimizeMaxError 5.0 in.db out.db
```

The config keeps `RGBD/OptimizeMaxError: 3.0`. Loosening a guard permanently to
compensate for a covariance that has since been fixed at source would be the
wrong repair.

**Still unresolved:** whether that 17–27° is poor PnP orientation or genuine
accumulated yaw drift. If it is real drift, the tag was right and rejecting it
threw away a correction we needed. Loosening the angular sigma helps either way,
but nothing so far establishes that the tag's orientation is untrustworthy.

### The AprilTag is now a real graph constraint

Earlier note, now superseded: the tag could only *measure* drift, because
slam_toolbox has no interface for injecting a landmark.

**RTAB-Map does.** It subscribes to `rtabmap_msgs/LandmarkDetection` on
`landmark_detection`, and with `Optimizer/LandmarksIgnored` at its default
`false`, GTSAM treats each sighting as a genuine pose-graph constraint. Two
sightings of the same tag on two visits tie those nodes rigidly together —
which is the loop closure that never fired in four walks.

`apriltag_localizer.py` publishes it. Details that matter:

- The pose goes out **in the camera optical frame**, stamped at *capture* time.
  RTAB-Map does the `base_stabilized -> camera_color_optical_frame` lookup
  itself and then corrects for odometry motion since. Nothing is pre-transformed.
- **Landmark IDs must be > 0.** RTAB-Map rejects `id <= 0` inside the conversion,
  per detection, at runtime. The node now refuses to start instead.
- The landmark is published **only after every rejection test** — size, range,
  and the depth cross-check. A landmark is a *hard* constraint: a wrong one does
  not degrade the map gracefully, it folds the graph around a lie. The depth
  cross-check exists to catch flipped PnP solutions, which look entirely
  confident.
- **Angular sigma is fixed and large (0.10 rad, ~5.7°)** while linear sigma grows
  with range. Orientation is the weak half of a planar PnP solution — the same
  near-degeneracy that produces the flips. This tells GTSAM to lean on *where*
  the tag is and largely disregard which way it is facing. Suspect this first if
  the map develops a twist around the tag.
- Covariance is populated explicitly. Leaving it zero is **not** neutral:
  RTAB-Map reads `covariance[0] <= 0` as "unset" and substitutes
  `landmark_linear_variance` (0.001, ~3 cm sigma) regardless of actual range.

One cosmetic collision: `apriltag_localizer` has published a `PoseStamped` on
`/tag_detections` since before RTAB-Map was involved, and RTAB-Map subscribes to
that same name expecting `apriltag_msgs/AprilTagDetectionArray`. Same name,
different type, so ROS logs an alarming *"incompatible QoS. No messages will be
sent to it"* that means nothing. The launch file remaps RTAB-Map's unused
subscription out of the way — warnings that are always present are warnings
nobody reads.

### 11b. AMCL abandoned for live localisation; RTAB-Map plus two tags instead

2026-08-24/25. AMCL was verified holding a genuine lock (§11a's tag-seeded
test: σ_x ≈ 5 cm, position stable to sub-cm over 8+ s). It then walked a
6 m stretch of a long, straight, feature-poor corridor and σ_x grew past
1.7 m — far worse than a similar walk on the *pre-fix* distorted map (54 cm).
Direct camera evidence of the corridor (colour feed: blank walls, a floor
line, distant doors) confirmed why: with a 70° FOV, a long corridor gives the
likelihood field almost no differential signal on position *along* it. No
AMCL parameter tunes this away — it is a real observability limit, not a
bug. This is the same limitation §11a's config comments already flagged in
theory; this is the first time it was measured.

**Switched to RTAB-Map's own localisation mode**
(`g1_bringup/launch/rtabmap_localization.launch.py`): `Mem/IncrementalMemory:
false` (never modify the map), `Mem/InitWMWithAllNodes: true` (load the whole
prior map, not just its last session), `publish_tf: true` (RTAB-Map now owns
`map -> odom`). Appearance-based place recognition doesn't share AMCL's
corridor weakness — a blank wall defeats geometry and vision alike, but a
door, sign, or piece of equipment (all present in this corridor's own colour
feed) gives vision a fix geometry never could.

**A bug that made RTAB-Map look broken for most of a day.** Walk 5 was
reprocessed with `--Optimizer/LandmarksIgnored true` (§11's fix for the
tag-rotation bug). RTAB-Map **persists that setting into the database** and
reloads it on every future launch — silently discarding every tag correction
in *this* session too, with no warning. Diagnosed by noticing
`apriltag_localizer` was correctly publishing `/landmark_detection` the whole
time while zero corrections ever showed up in the log. Fixed by explicitly
overriding `Optimizer/LandmarksIgnored: false` in the localisation launch
file's inline parameters, which take priority over whatever the database
says. Safe here specifically because `Mem/IncrementalMemory` is false — a
landmark can only correct the *current pose estimate*, never touch the frozen
graph.

**Cold-start appearance matching failed before the tags were anchoring
correctly** — 755 candidate matches attempted over an extensive walk, every
single one rejected with **exactly** 0/20 inliers despite 40-54 raw feature
matches. First suspected as perceptual aliasing (repetitive-looking building).
**That theory turned out to be wrong.** With `Optimizer/LandmarksIgnored`
fixed and both tags anchoring the pose (§ below), a repeat walk through the
same corridor found **8 genuine confirmed matches** (both loop closures and
proximity detections) — real visual content clearly *is* matchable here.
The likely actual cause: RTAB-Map's geometric verification (PnP/RANSAC) uses
the *current pose estimate* as a prior to search for correspondences
(`Vis/CorGuessWinSize`). With the pose badly drifted (no tag anchoring yet),
that prior pointed RANSAC's search window at the wrong part of the image
entirely — real matches were being missed by a bad prior, not rejected for
lacking one. Anchoring the pose with tags didn't just add corrections; it
gave every subsequent visual match a search window that could actually find
the truth.

**Practical conclusion, from everything measured today: both methods hold a
tight lock once seeded, and neither can be trusted to find that lock from
scratch or after a long unobserved stretch, for different underlying
reasons** (AMCL: geometric observability; RTAB-Map: possible perceptual
aliasing). The fix is the same either way — don't ask either algorithm to
carry the whole distance alone. Seed once, then re-anchor periodically with
something unambiguous.

#### Two AprilTags, not one

Tag 10 (map 3.38, 1.88 — see §11) covers the start/tag area. Added **tag 20**
partway down the corridor that broke AMCL, id chosen to stay clear of 10 and
leave room for more. Measured live, the same way as tag 10 originally: with a
reasonably-anchored pose nearby, read `apriltag_localizer`'s own "first seen
at map (x, y)" log line. Confirmed stable — detection climbed from 12 to a
steady 75 frames/5 s with no drift warnings, landing at **(2.91, 12.01)**,
matched within 1 cm by the live landmark-publishing instance's own first
reading.

Both tags run as separate `apriltag_localizer` instances, both publishing to
the same `/landmark_detection` topic — safe, because `LandmarkDetection`
carries its own `id` field and RTAB-Map keys on it; confirmed 2 publishers
active with no collision.

**A real PnP-flip caught live, worth knowing the signature of.** Standing
close to tag 10 nearly head-on, every single reading showed the *identical*
disagreement: PnP said 4.63 m, depth said ~1.8 m, hundreds of times in a row,
never varying. That constancy is the tell — real noise varies frame to frame;
a flipped-but-locally-stable PnP solution does not. This is exactly the
"depth cross-check" `apriltag_localizer` was built to catch (§ earlier), and
it did: every one of those readings was correctly rejected rather than fed to
RTAB-Map as a confident wrong pose. Fixed by physically backing up / viewing
the tag at a slight angle instead of dead-on — a known degenerate case for
planar-target PnP, not a code bug.

**Process hygiene note, twice bitten today:** an ad-hoc diagnostic
`apriltag_localizer` instance launched via `ros2 run` leaves an orphaned
child process behind even after the parent `ros2 run` wrapper is killed —
`kill` the PID `ros2 node list` / `ps` actually shows running the node, not
the wrapper PID printed by the launching shell. Same lesson as the
nine-orphan `g1_state_bridge` incident (§7): always verify with `ps` after
killing, don't trust the PID you started it with.

### 11c. Nav2: costmaps, planner, controller

2026-08-25. `g1_navigation/config/nav2_params.yaml` + new
`g1_bringup/launch/nav2_navigation.launch.py`, built on RTAB-Map localisation
(§11b), not AMCL.

**`docking_server` blocked the entire stack from activating.**
`nav2_bringup`'s `navigation_launch.py` hardcodes its managed node list as a
Python literal (not a launch argument) and unconditionally includes
`docking_server`, which refuses to activate without at least one real
charging-dock plugin -- *"Charging dock plugins not given!"* -- and there is
no valid empty/no-op config for that requirement. This robot has no docking
hardware. One unconfigurable node aborted `lifecycle_manager_navigation`'s
entire bringup every time. Fixed by not including that file at all --
`nav2_navigation.launch.py` hand-rolls the same node set (controller_server,
planner_server, behavior_server, bt_navigator, waypoint_follower,
velocity_smoother, collision_monitor) minus docking_server, smoother_server
and route_server -- the latter two are harmless but genuinely unconfigured
here, and this project would rather not run a node on the faith that its
defaults are fine (see the collision_monitor lesson right below).

**`collision_monitor` needs its own config section** -- it also has no safe
default (`observation_sources` is a required parameter with no fallback) and
blocked bringup the same way docking_server did before it was configured.
Added a single "approach" polygon reading `/scan`, tied to the local
costmap's own published footprint.

**Controller and footprint are placeholders, not measurements.** RPP
(`nav2_regulated_pure_pursuit_controller`) was chosen over DWB for less
tuning to a first working state; it commands forward speed and yaw only,
leaving the G1's real strafing ability (§11a) unused -- revisit with
`nav2_mppi_controller` once basic navigation is proven, not before.
Footprint is a circular/rectangular placeholder from approximate shoulder
width, not a measured value.

**First real goal: stuck in a repeated abort loop, and it took real log
digging to find why.** Sent via RViz's `2D Goal Pose` tool (the
`Navigation 2` PANEL's "Start Nav Through Poses" button is a *different*
tool requiring its own multi-waypoint selection first -- clicking it with
nothing selected just aborts instantly, harmless but easy to mistake for a
real failure). The genuine attempt got stuck: yaw swung roughly 180° while
XY position barely moved, and `controller_server` logged **"Failed to make
progress" seven times over ~111 seconds, ~15-20 s apart** -- too regular to
be random.

Root cause: `nav2_regulated_pure_pursuit_controller`'s own default
`rotate_to_heading_angular_vel` is **1.8 rad/s**, but `velocity_smoother`'s
`max_velocity` caps angular speed at **0.5 rad/s** below it -- a mismatch
introduced when the smoother's conservative placeholder limits (§ nav2_params
comments) were picked without checking them against RPP's own defaults. RPP
commands 1.8, the smoother silently throttles every command to 0.5 -- a >3x
slowdown RPP has no way to know about. `SimpleProgressChecker` only counts
*linear* movement (`required_movement_radius`), not rotation, so a large
heading correction that RPP expects to finish in a few seconds was actually
taking three times as long at the real, throttled rate -- long enough to blow
through the 15 s progress window on almost every large turn. Fixed by setting
`rotate_to_heading_angular_vel: 0.5` to match the smoother's real cap
(matching commanded to achievable, not raising the achievable to match an
untested commanded value), and raising `movement_time_allowance` 15 -> 20 s
for margin on the translation phase that follows a long rotation.

**Re-tested. The fix was real but not sufficient.** Rotation itself genuinely
improved -- the robot completed a real ~180 deg turn this time, confirmed
live via TF (not just commanded). But **"Failed to make progress" fired 14
times over the retest, more than the original 7**, and net displacement over
the full ~300 s attempt was only ~0.32 m -- barely above the 0.3 m
`required_movement_radius` threshold itself. Rotation works; sustained
forward walking after rotating does not. Stopped by closing the gate again
(`ros2 service call /g1_loco_bridge/disable std_srvs/srv/Trigger {}`), robot
confirmed physically stationary before disconnecting.

**A real mistake worth recording, not just the robot's.** Mid-retest, a
long-running Python TF watcher script appeared to show yaw frozen at ~8 deg
for 90+ seconds while the robot was, per the operator, visibly turning
around. This was reported to the user as a possible localisation/reality
disconnect and the gate was closed as a precaution. **It was not a real
disconnect** -- a fresh, direct `tf2_echo` query taken immediately after
showed the correct current pose (map yaw -170.5 deg, matching the physical
turn) matching the watcher's own later lines once caught up. The actual
error: reporting a snapshot read ~200 s earlier as current state, not a
staleness bug in the pipeline itself. Lesson: when a long-running background
watcher's *last known* value looks alarming, re-query live before treating it
as ground truth -- the delay between "I last checked" and "now" can itself be
the whole explanation. Closing the gate on that mistaken belief was still the
right call given the information available at the time; the mistake was in
the diagnosis, not the caution.

**Next session should NOT keep guessing from post-hoc log greps** -- that
approach diagnosed the rotation-speed mismatch correctly but has now missed
whatever is actually blocking forward progress.

**Built**: `ros2 run g1_bringup nav2_watch` (`g1_bringup/g1_bringup/nav2_watch.py`).
Samples pose (TF), the full command chain side by side
(`/cmd_vel_nav` -> `/cmd_vel_smoothed` -> `/cmd_vel`, so a drop anywhere in
that chain is visible instead of inferred), local costmap cost at the
robot's own cell plus distance to the nearest obstacle-grade cell (reads
`/local_costmap/costmap_raw`, NOT the plain `/local_costmap/costmap`
OccupancyGrid -- that one is rescaled 0-100 for RViz and silently breaks a
raw 253/254 obstacle threshold), global vs. RPP's-own-copy plan length, and
`/navigate_to_pose` action feedback (distance remaining, recovery count) --
all at the same instant, every ~0.5 s, with the age of each value printed
alongside it so a stale read is visible rather than silently indistinguishable
from a fresh one (the mistake made earlier in this same session, see above).
Start it BEFORE sending the goal so its very first samples cover goal
acceptance. Unit-tested against a synthetic costmap; not yet run against a
real stuck attempt.

Candidate hypotheses not yet checked: RPP's regulated-speed scaling
being throttled near-zero by costmap proximity/curvature
(`regulated_linear_scaling_min_radius: 0.9`, `regulated_linear_scaling_min_speed:
0.08`) even on ostensibly clear paths; the planner repeatedly finding a path
that immediately fails some downstream check; or a genuine mismatch between
the planned path's frame/orientation and what the controller can actually
follow.

A secondary, less certain finding from the same stuck window: `collision_monitor`
logged a TF extrapolation error, `/scan`'s observation buffer went stale
(0.60 s old, should refresh every 0.5 s), the control loop's rate briefly
dropped from 10 Hz to 6.6 Hz, and `g1_loco_server` timed out replying to a
command -- all within about a 2-second window. That smells like a shared
resource hiccup (CPU spike, DDS discovery churn from the multiple
kill/relaunch cycles around that time) rather than four independent bugs, but
it only happened once during a ~2-minute test and has not been isolated or
confirmed as a recurring issue.

**Safety note for next time:** `nav2_navigation.launch.py`'s `start_enabled`
default is `true` -- unlike every other launch file in this project, which
starts with the gate closed. Nav2 cannot do anything useful with the gate
closed and there is no joystick backup once it's driving, but this is a real
departure from the "closed by default" pattern everywhere else -- worth
remembering before assuming any launch file here is safe-by-default.

## 12. Open items

- IMU covariances are placeholders (§6).
- Fixed 3.4° camera-to-IMU misalignment (§9); will tilt the map, not an axis error.
- `base_footprint` offset is fixed, and the robot's lean is not in TF at all
  (§4, §9); both are resolved by stage 6.
- Camera roll measured +1.04°, currently ignored (§4).
- The camera cannot see the ground at its own feet (§4).
- `g1_navigation` and `g1_interfaces` are still empty packages.
- The repo has no commits yet, so there is no diff to fall back on.
- The bad-pose-prior explanation for the 755/755 cold-start rejection rate is
  well supported (8 genuine matches once tags anchored the pose) but not
  proven with certainty — hasn't been isolated from other things that changed
  at the same time (the corridor recalibration, walking further).
- Only tag 10's map position was re-verified against the corrected
  (no-landmark) graph today. Tag 20's position was measured using a
  pose anchored by tag 10's live correction, which is sound, but neither has
  been cross-checked by approaching from a second, independent direction.
- AMCL's config (`g1_navigation/config/amcl.yaml`) is unmodified and still
  fully functional — nothing about the RTAB-Map switch broke it. Worth
  revisiting per-corridor if RTAB-Map's re-anchoring strategy has coverage
  gaps AMCL could fill outside the weak stretch specifically.
- **Nav2 has not completed a successful goal yet.** Rotation is confirmed
  working (real ~180° turn, TF-verified). Forward walking after rotating is
  not — 14 progress-checker failures, ~0.32 m net movement over ~300 s on the
  retest. This is now the priority open item for stage 9 (§11c) — needs a
  live watcher -- `ros2 run g1_bringup nav2_watch` (§11c) -- while a goal
  executes, not more log-file archaeology after the fact.
- Nav2's footprint, velocity limits, and controller choice (RPP) are all
  placeholders (§11c) — none are measured from the real robot. Safe defaults
  for a first test, not tuned values.
- The collision_monitor/scan-staleness/loco_server-timeout cluster (§11c) was
  seen once and not isolated. Worth watching for on the next real navigation
  attempt — if it recurs, it's systemic and needs its own investigation.

---

## Decision log

| Date | Decision | Reason |
|---|---|---|
| 2026-08-07 | Two-process DDS↔ROS bridge over ZMQ/TCP | `libddsc` symbol clash |
| 2026-08-07 | Pin SDK `libddsc` with `DT_RPATH` | ROS's `libddsc` was being paired with the SDK's `libddscxx` |
| 2026-08-07 | `base_link` = pelvis; `base_footprint` at 0.785 m | IMU and encoders both reference the pelvis; height measured live |
| 2026-08-07 | Deleted the legacy `base_link → pelvis` static TF | It placed `base_link` at the old MID360 LiDAR |
| 2026-08-08 | Colour is the **left** half of the stitched JPEG | Server does `hconcat([colour, depth_colormap])` |
| 2026-08-08 | Reliable QoS on image topics | Best-effort lost 8% of depth frames to fragmentation |
| 2026-08-08 | **Rejected** PNG depth compression | Profiling: `send` = 0.3 ms; the bottleneck is Jetson CPU |
| 2026-08-08 | 848×480 at 30 Hz | Same rate as 640×480 but keeps 69.8° H FOV; D435 crops horizontally for 4:3 |
| 2026-08-08 | Dropped `hole_filling` | Fabricates depth; density only falls 99.9% → 97% |
| 2026-08-08 | Added the camera optical frame chain | No optical rotation existed; clouds would be 90° wrong |
| 2026-08-08 | Camera mount re-angled mechanically, 63.9° → 19.16° | The old angle saw only floor 1.5 m ahead; the new one sees walls and the horizon |
| 2026-08-08 | `d435_joint` pitch calibrated to 0.3344213 rad, not Unitree's 0.8307767 | Camera-vs-IMU measurement, 19.16° ± 0.12; the stock value never matched this robot |
| 2026-08-08 | Camera roll left at 0 despite measuring +1.04° | Cannot distinguish a 1° camera rotation from a 1° floor slope on gym matting |
| 2026-08-08 | Depth colourisation moved to the PC (`live_view.py`) | It cost the robot ~15 ms/frame; the PC has the raw depth anyway |
| 2026-08-08 | chrony on both machines, robot slaved to the PC | `systemd-timesyncd` left the clocks 112 ms apart while both reported "synchronized"; camera and IMU are stamped by different clocks |
| 2026-08-08 | `check_sensors` fails on a *negative* frame age | The 112 ms skew passed the original check, which only tested staleness. A future-stamped frame is impossible and must fail loudly |
| 2026-08-10 | IMU axis check is stationary + accelerometer, not walking + quaternion | The quaternion's two readings differ by a sign flip on the axes under test, so it cannot distinguish a misread convention from an inverted axis; walking also yanked the ethernet tether |
| 2026-08-10 | Verdicts use orthogonal regression and a confidence interval | Noise in the camera (the input variable) biases ordinary slopes toward zero; a fixed span threshold rejected data that was perfectly conclusive |
| 2026-08-10 | Quaternion convention is `row` (world-up = third row of R) | Resolved with a 15.5° margin once genuinely tilted; the near-upright margin was 0.3° and picked wrong |
| 2026-08-10 | Check runs are archived as JSON in `~/.ros/g1_checks/` | Raw samples survive, so an improved analysis can be re-run on old data with `--reanalyse` instead of re-testing the robot |
| 2026-08-19 | Locomotion copied from `waypoint_follow_ws` rather than overlaying it | Each workspace runs independently; the price is two copies that will drift, accepted knowingly |
| 2026-08-19 | Odometry comes from the robot's own estimator (`rt/odommodestate`) | Found by enumerating DDS discovery, not guessing topic names; leg odometry + IMU fused on the robot, tape-measure validated, so no leg odometry needs writing |
| 2026-08-19 | Camera mount angle treated as a tunable mechanical setting | The bracket is hand-adjustable; aiming it higher is a legitimate fix if AMCL struggles, at the cost of near-field blindness. Requires re-calibrating `d435_joint` every time |
| 2026-08-19 | New code is written in Python | The C++ is confined to what must link `unitree_sdk2` |
| 2026-08-19 | `pointcloud_to_laserscan` with a height filter, not `depthimage_to_laserscan` | The camera is pitched 19.16° down, so an image-row band reads the floor, not walls |
| 2026-08-19 | Added a `base_stabilized` frame | The height filter needs a level frame; `base_footprint` carries the robot's full attitude and tilts 7.5° |
| 2026-08-19 | Scan band 0.30–1.50 m | Overlaps the map's 0.12–2.20 m obstacle band while staying inside what the camera can actually observe |
| 2026-08-19 | AMCL `base_frame_id` is `base_stabilized` | The scan is expressed there; pointing AMCL at the tilted `base_footprint` would read every range against a tilted axis |
| 2026-08-19 | AMCL uses `OmniMotionModel` with a raised `alpha5` | The G1 side-steps, and drifts sideways when walking straight (0.52 m over 1.37 m); a differential model cannot represent that |
| 2026-08-19 | Odometry noise below stock, particle count above | Narrow FOV cannot re-tighten a spread-out particle cloud, but the odometry is good enough to trust |
| 2026-08-22 | **Mapping switched from slam_toolbox to RTAB-Map** | Four walks, zero loop closures. Karto correlates range wedges; a 70° FOV with returns starting at 1.5 m and ~350 mm jitter gives it nothing to correlate. RTAB-Map closes on RGB appearance instead, and geometry is only needed after a place is already recognised |
| 2026-08-22 | `Grid/RayTracing: true` | Walk 4 was still 67.1% unknown and AMCL scored 62% of endpoints against unknown space, which it reads as no information. Ray tracing turns the volume actually looked through into known free space |
| 2026-08-22 | `Reg/Force3DoF: true`, and no `Optimizer/Slam2D` | Flat floor and a 2D deliverable, so z/roll/pitch drift is removed outright — worth more on a robot that bobs every step. `Optimizer/Slam2D` does not exist in 0.22.1 and is silently dropped |
| 2026-08-22 | No loop-closure distance gate (`RGBD/MaxLoopClosureDistance: 0.0`) | The equivalent slam_toolbox gate (`loop_search_maximum_distance: 3.0`) is why walk 3 never even searched. Drift is largest precisely where the closure is worth most |
| 2026-08-22 | AprilTag published as an RTAB-Map landmark, not just logged | RTAB-Map has a typed landmark interface that slam_toolbox lacked; GTSAM honours it as a hard constraint, so the tag now *corrects* the graph instead of only measuring how bent it is |
| 2026-08-22 | Landmark angular sigma large (0.10 rad), linear sigma range-dependent | Orientation is the weak half of a planar PnP solution — the same degeneracy that yields flipped solutions. Trust *where* the tag is, not which way it faces |
| 2026-08-22 | Landmark published only after the depth cross-check | A landmark is a hard constraint: a wrong one folds the graph around a lie rather than degrading gracefully. Good enough to report ≠ good enough to optimise against |
| 2026-08-22 | `Grid/RangeMax` 5.0 → 3.5 m | Walk 5 came out 19.4% occupied with noise fans along the walk path. Depth jitter and pitch-induced floor leak both scale with range; this bounds the grid only, not the cloud or loop closure |
| 2026-08-22 | `landmark_sigma_angular` 0.10 → 0.5 rad | At 0.10 the tag's 17–27° orientation disagreement read as 3–5σ and `RGBD/OptimizeMaxError` vetoed the whole constraint, position included. All four landmark constraints were discarded, so the tag contributed nothing to walk 5 |
| 2026-08-22 | `RGBD/OptimizeMaxError` stays 3.0; overridden only when reprocessing walk 5 | The gate rejected five constraints and every one was the landmark — all 79 visual rejections came from `Vis/MinInliers`. The fault was the covariance, now fixed at source; permanently loosening a guard to compensate would be the wrong repair |
| 2026-08-22 | **`rtabmap-reprocess` is part of the mapping workflow, not a debugging tool** | Reprocessing walk 5 found 635 loop closures against the live run's 4 — replaying without a real-time budget lets proximity detection run against an already-optimised graph. Walk, reprocess, then use the reprocessed database |
| 2026-08-22 | A map that covers more ground is not automatically better | Reprocessing shrank the extent 32.5×26.8 → 28.7×23.6 m. The lost area was far-field noise inflating the bounding box, not discovered structure |
| 2026-08-24 | Reprocessed walk 5 again with `Optimizer/LandmarksIgnored: true`, keeping `Grid/RangeMax: 3.5` | Live AMCL testing found a real ~27-40° rotation near the tag that a manual RViz re-seed could not shake. Suspected the loosened landmark tolerance had let genuine tag drift warp the graph. Closures dropped 635 → 123 (mostly proximity) but occupied/free/unknown stayed nearly identical, meaning most of the lost closures were redundant, not structural |
| 2026-08-24 | **Localisation switched from AMCL to RTAB-Map's own mode** | AMCL held a tight lock once tag-seeded (σ_x ≈ 5 cm) but σ_x grew past 1.7 m over a 6 m walk through a long feature-poor corridor — a real geometric-observability limit a 70° FOV cannot escape, not a tuning problem. RTAB-Map's appearance matching doesn't share that specific weakness |
| 2026-08-25 | `Optimizer/LandmarksIgnored` explicitly overridden to `false` in the localisation launch | RTAB-Map persists that setting *into the database* and reloads it on every future launch, silently discarding every tag correction with no warning — even though it was set `true` deliberately for a different reason (mapping-time graph safety) that no longer applies once `Mem/IncrementalMemory` is false |
| 2026-08-25 | Added a second AprilTag (id 20, map 2.91/12.01) partway down the weak corridor | Neither AMCL nor RTAB-Map can reliably relocalise from scratch in this building, but both hold a tight lock once seeded. Periodic re-anchoring beats asking either algorithm to carry the whole distance alone |
| 2026-08-25 | Perceptual-aliasing theory retired | With both tags anchoring, a repeat corridor walk found 8 genuine confirmed matches -- the earlier 755/755 rejection rate was much more likely a bad-pose-prior problem (RANSAC searching the wrong part of the image) than a genuine appearance-matching failure |
| 2026-08-25 | Nav2 hand-rolls its node list instead of including nav2_bringup's navigation_launch.py | That file's lifecycle_nodes list is hardcoded and unconditionally includes docking_server, which has no valid config for a robot with no docking hardware and aborted the entire bringup every time |
| 2026-08-25 | `rotate_to_heading_angular_vel: 0.5`, matching velocity_smoother's real wz cap | RPP's own default (1.8 rad/s) was 3.6x the smoother's actual limit (0.5). RPP commanded at the fast rate, the smoother silently throttled every command, and SimpleProgressChecker's 15 s window (which only counts linear movement) kept timing out mid-turn -- a real stuck-robot incident, not a config nicety |
