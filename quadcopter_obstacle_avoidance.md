# 4-Camera Obstacle-Avoidance Quadcopter — Simulate & Deploy

A 360°-ish obstacle-avoidance system for a **multirotor** (ArduPilot Iris) using
**four cameras** (front / back / left / right). It is the airborne sibling of the
ground-robot build in [`car_obstacle_avoidance.md`](car_obstacle_avoidance.md):
the **perception pipeline is identical** (YOLOv8n + monocular depth + L/C/R
zones), but the **output stage** drives a copter over MAVLink instead of a
diff-drive base.

Because a quadcopter is holonomic in the horizontal plane, each per-direction
`STOP` maps onto a flight axis:

| Direction in `STOP` | Aircraft action |
| ------------------- | --------------- |
| **FRONT** | block forward motion (don't pitch forward) |
| **BACK**  | block reverse motion (don't pitch back) |
| **LEFT**  | strafe **right** (roll right) + yaw nose **right** |
| **RIGHT** | strafe **left** (roll left) + yaw nose **left** |
| **LEFT + RIGHT** | "boxed in" — no auto strafe/yaw, manual + pitch-gate only |

Climb (throttle/`linear.z`) is **never gated** — up is always an escape.

This document covers:

1. [How it works](#1-how-it-works)
2. [Repository layout](#2-repository-layout)
3. [Run it in simulation (Gazebo + ArduPilot SITL)](#3-run-it-in-simulation-gazebo--ardupilot-sitl)
4. [Deploy on a real drone](#4-deploy-on-a-real-drone)
5. [Tuning & troubleshooting](#5-tuning--troubleshooting)
6. [Safety notes](#6-safety-notes)

---

## 1. How it works

### Perception pipeline (per frame, all 4 cameras batched)

```
                 ┌───────────────────────────┐
 4 camera   ───► │  YOLOv8n  (detection)     │ ─► bounding boxes (what)
 frames          │  Depth Anything V2 (depth)│ ─► relative depth (how close)
                 └─────────────┬─────────────┘
                               │  per camera
                 ┌─────────────▼─────────────┐
                 │ normalize depth 0..1       │
                 │ split L / C / R zones      │
                 │ count "close" pixels       │
                 │ -> CLEAR / CAUTION / STOP  │
                 └─────────────┬─────────────┘
                               │  4 × (zone statuses)
                 ┌─────────────▼─────────────┐
                 │ safety filter -> MAVLink   │ ─► GUIDED body-velocity setpoint
                 └───────────────────────────┘
```

- **YOLOv8n** (`yolov8n.pt`, Ultralytics) — lightweight detector, batched across
  the 4 frames in one inference call.
- **Depth Anything V2 Small** (`depth-anything/Depth-Anything-V2-Small-hf`) —
  monocular **relative** depth (larger value = closer). Not metric.
- **Zones**: each view → 3 vertical slices. A zone is `STOP` when ≥ `STOP_FRAC`
  (25 %) of its pixels are "close" (normalized depth > `CLOSE_THRESH` = 0.70),
  `CAUTION` at ≥ `WARN_FRAC` (10 %), else `CLEAR`. Tunable at the top of the script.

This is the same code path as the ground robot, so anything you tune for one
carries over to the other.

### Control / safety layer (the copter-specific part)

The driver (keyboard teleop, gamepad, or an autonomy node) publishes a raw
velocity intent as a ROS `Twist` on `/cmd_vel_teleop`:

| Twist field | Meaning | Maps to |
| ----------- | ------- | ------- |
| `linear.x`  | forward (+) / back (−) | pitch |
| `linear.y`  | left (+) / right (−) (ROS convention) | roll |
| `linear.z`  | up (+) / down (−) | climb |
| `angular.z` | yaw left/CCW (+) | yaw rate |

`drone_obstacle_avoidance.py` filters that intent with the per-zone `STOP` flags
(table at the top of this doc) and sends the result to the autopilot as a
**GUIDED body-frame velocity + yaw-rate setpoint** over MAVLink (`pymavlink`):

```
teleop_twist_keyboard ─► /cmd_vel_teleop ─► [avoidance node: safety filter]
                                                     │  body velocity + yaw rate
                                                     ▼
                                   MAVLink GUIDED setpoint ─► ArduCopter (SITL or real FC)
```

- The setpoint is `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_BODY_NED` with a
  type-mask that uses **velocity + yaw-rate only** (position, acceleration and
  yaw-angle ignored). The autopilot realizes forward velocity as pitch, lateral
  velocity as roll, and yaw-rate as yaw — so the avoidance literally acts on
  pitch / roll / yaw.
- Setpoints are streamed continuously at `SETPOINT_HZ` (10 Hz). **GUIDED reverts
  to Loiter if setpoints stop for more than a few seconds** — this is a built-in
  failsafe, not a bug.
- On startup the node connects, requests MAVLink data streams, sets **GUIDED**,
  **arms**, **takes off** to `TAKEOFF_ALT` (3 m), then streams setpoints. On exit
  it commands a zero-velocity hover.

> ⚠️ **Relative depth is not metric.** `STOP` means *something is the closest
> thing in this view*, not "obstacle at 1.2 m". Against a large flat wall that
> fills the whole frame, normalized depth loses contrast (min ≈ max), so the
> `STOP` standoff can be uncomfortably small. For a dependable distance gate,
> pair each direction with a metric sensor (ToF/ultrasonic/radar) — see
> [§4](#4-deploy-on-a-real-drone).

---

## 2. Repository layout

```
obstacle_dection_four_cameras/
├── yolov8n.pt                          YOLOv8n weights (shared with the car build)
├── car_obstacle_avoidance.md           ground-robot README
├── quadcopter_obstacle_avoidance.md    (this file)
└── sim_drone/                          Gazebo + ArduPilot SITL twin
    ├── drone_obstacle_avoidance.py     perception + safety + MAVLink (the node)
    ├── obstacle_world_iris.sdf         world: ground + 4 obstacles at flight altitude
    ├── iris_cams_bridge.yaml           Gazebo -> ROS 2 bridge (4 cams + clock)
    ├── models/iris_four_cam/           Iris + 4 fixed cameras (merges iris_with_gimbal)
    │   ├── model.sdf
    │   └── model.config
    ├── run_gazebo_iris.sh              T1: launch Gazebo (ArduPilot lock-step)
    ├── run_sitl.sh                     T2: ArduCopter SITL (headless, --no-mavproxy)
    ├── run_bridge_iris.sh              T3: ros_gz_bridge for the cameras
    ├── run_avoidance_iris.sh           T4: the perception/safety/MAVLink node
    └── run_teleop.sh                   T5 (optional): keyboard teleop
```

The avoidance node is the same on the bench and in the air — only `MAVLINK_URL`
changes. In sim it points at ArduCopter SITL (`tcp:127.0.0.1:5760`); on a real
drone it points at the flight controller's telemetry port.

---

## 3. Run it in simulation (Gazebo + ArduPilot SITL)

Validated stack: **Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim 8 (Harmonic, 8.10) +
ArduCopter SITL + `ardupilot_gazebo`**, with an NVIDIA GPU (CUDA) for perception.

### 3.1 Dependencies

```bash
# ROS 2 Humble + Gazebo Harmonic + bridge + teleop
sudo apt install ros-humble-desktop ros-humble-ros-gz \
                 ros-humble-teleop-twist-keyboard

# Python perception + control deps (into the env that runs the node)
pip install ultralytics transformers "pillow>=10" opencv-python pymavlink
# torch with CUDA matching your machine (from pytorch.org)
```

**ArduPilot SITL + the Gazebo plugin** (one-time; the scripts expect these paths):

```bash
# ArduPilot source (gives sim_vehicle.py + builds the ArduCopter SITL binary)
git clone --recursive https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y

# ardupilot_gazebo plugin (the .so that couples Gazebo <-> ArduPilot over JSON FDM)
mkdir -p ~/ardu_ws/src && cd ~/ardu_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ~/ardu_ws && colcon build   # or the cmake build in that repo's README
```

`run_gazebo_iris.sh` adds `~/ardu_ws` (plugin `.so` + Iris models) and this
repo's `models/` to `GZ_SIM_SYSTEM_PLUGIN_PATH` / `GZ_SIM_RESOURCE_PATH`, so no
manual env setup is needed beyond having those two trees built.

### 3.2 Launch (4 terminals, in order)

ArduPilot uses **lock-step**: Gazebo waits for SITL to connect on port 9002, so
start Gazebo first, then SITL.

```bash
cd sim_drone

# Terminal 1 — Gazebo with the Iris-4-camera obstacle world
./run_gazebo_iris.sh

# Terminal 2 — ArduCopter SITL (first run compiles the binary, ~1–2 min)
./run_sitl.sh

# Terminal 3 — bridge the 4 cameras (+ /clock) into ROS 2
./run_bridge_iris.sh

# Terminal 4 — perception + safety + MAVLink: connects, arms, takes off to 3 m,
#              streams GUIDED setpoints, opens the 2×2 grid window
./run_avoidance_iris.sh

# Terminal 5 (optional) — fly it with the keyboard (needs keyboard focus)
./run_teleop.sh
```

Teleop keys (`teleop_twist_keyboard`, remapped to `/cmd_vel_teleop`):

```
  i / ,        pitch forward / back   (linear.x)
  J / L (shift) strafe left / right   (linear.y, "Holonomic"/strafe mode)
  j / l        yaw left / right       (angular.z)
  t / b        climb up / down        (linear.z)
  k or space   hover (zero)
  q / z        increase / decrease speed
```

The grid window shows each camera's depth heatmap, the per-zone bars
(green/yellow/red), a `FLYING` / `MAVLINK` / `PERCEPTION-ONLY` mode tag with FPS,
and a red **`AVOID:`** banner when an override engages (`PITCH-STOP FWD`,
`PITCH-STOP REV`, `ROLL LEFT/RIGHT`, `BOXED IN`).

### 3.3 Why the SITL launch is headless (important)

`run_sitl.sh` runs `sim_vehicle.py ... --no-mavproxy --no-rebuild`, and the
avoidance node talks MAVLink **straight to ArduCopter SITL on
`tcp:127.0.0.1:5760`** (set in `run_avoidance_iris.sh` via `MAVLINK_URL`, and as
the default in the script). This was a deliberate change from the original
MAVProxy-based wiring:

- MAVProxy's interactive `--console` needs a DISPLAY/TTY; under a detached
  (`setsid … </dev/null`) launch it hits EOF and exits, taking SITL down with it.
- Talking directly to the FC means no GCS is requesting telemetry, so the node
  calls `request_data_stream_send(..., MAV_DATA_STREAM_ALL, 5, 1)` after the
  heartbeat — otherwise `GLOBAL_POSITION_INT` never arrives and the takeoff
  altitude check stalls. (This is also exactly what you do on real hardware.)

If you *want* MAVProxy/a GCS as well, run SITL the normal way and add a
`--out udp:127.0.0.1:14552`, then point `MAVLINK_URL` at that UDP port instead.

### 3.4 Detached / background launch

The processes are long-running; to keep them alive from a non-interactive shell,
launch each with `setsid` in its own session and run perception on `DISPLAY=:1`:

```bash
DISPLAY=:1 setsid bash ./run_gazebo_iris.sh > /tmp/gz.log 2>&1 </dev/null &
setsid bash ./run_sitl.sh                    > /tmp/sitl.log 2>&1 </dev/null &
setsid bash ./run_bridge_iris.sh             > /tmp/bridge.log 2>&1 </dev/null &
DISPLAY=:1 setsid bash ./run_avoidance_iris.sh > /tmp/avoid.log 2>&1 </dev/null &
```

A plain `nohup … &` can get reaped when the launching task exits; `setsid`
survives. After restarting Gazebo, also restart the bridge (it does not reliably
re-discover a new Gazebo instance).

Useful checks while detached:

```bash
# SITL listening + GPS/EKF healthy (request streams first, or you'll only see HEARTBEAT)
python3 - <<'PY'
from pymavlink import mavutil
m = mavutil.mavlink_connection("tcp:127.0.0.1:5760"); m.wait_heartbeat()
m.mav.request_data_stream_send(m.target_system, m.target_component,
                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)
print(m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5))
PY

# Camera topics publishing
ros2 topic hz /cam_front/image

# Drone pose (no odom is bridged; read it straight from Gazebo)
gz topic -e -n 1 -t /world/map/dynamic_pose/info | grep -A6 'name: "iris"'
```

> Note: only **one** client may hold SITL's TCP `5760` at a time — the avoidance
> node owns it while running. Inject driver intent via ROS `/cmd_vel_teleop`, not
> a second MAVLink connection.

### 3.5 Stop everything

Use **bracketed** patterns so `pkill -f` doesn't match (and kill) its own command
line:

```bash
pkill -9 -f "gz[ ]sim";        pkill -9 -f "sim_[v]ehicle"
pkill -9 -f "[a]rducopter";    pkill -9 -f "[p]arameter_bridge"
pkill -9 -f "drone_obstacle_[a]voidance"
```

---

## 4. Deploy on a real drone

A real quadcopter already has a **flight controller** (Pixhawk / Cube / similar
running **ArduPilot** or PX4) doing the stabilization, EKF and motor mixing. So
deployment is *not* "replace the diff-drive with motors" like the car — it's:

```
        ┌──────────────────────────────┐        ┌─────────────────────────┐
 4 cams │ Companion computer            │ MAVLink│ Flight controller       │ motors
 ──────►│  drone_obstacle_avoidance.py  │───────►│  ArduCopter (GUIDED)    │──────►
        │  (YOLO + depth + safety)      │ serial │  EKF / stabilize / mix  │
        └──────────────────────────────┘ or UDP └─────────────────────────┘
```

**The control path is already what the sim does** — the node speaks MAVLink to an
autopilot. You only change *where* `MAVLINK_URL` points. That makes the porting
effort mostly (a) wiring the companion computer to the FC, and (b) getting the
4-camera inference fast enough on the companion.

### 4.0 Bill of materials

| Part | Notes |
| ---- | ----- |
| Airframe | Quad/hex with payload margin for an SBC + 4 cameras (≈ 250–450 mm+) |
| Flight controller | Pixhawk 6C / Cube Orange / Matek H743 — ArduCopter or PX4 |
| Companion computer | **Jetson Orin Nano/NX (has CUDA → runs this torch pipeline unchanged)**, or RPi 5 + Hailo, RDK X5, ROCK 5C |
| 4 × cameras | USB UVC or CSI/GMSL; 640×480 @ 15 fps is plenty. Global-shutter preferred (rolling shutter + vibration = smear) |
| **Distance sensors (strongly recommended)** | 4 × ToF (VL53L1X) or a scanning/solid-state lidar for *metric* gating |
| FC↔companion link | UART (TELEM2) or USB; set the FC serial to MAVLink2 |
| GPS / position source | GPS+compass outdoors; optical-flow + rangefinder or a VIO/mocap source indoors (GUIDED needs a position estimate) |
| Power | Companion + cameras on a regulated 5 V BEC, separate from ESC power |

### 4.1 Wire the companion to the flight controller

1. On the FC, set the companion serial port to MAVLink2 (ArduPilot params, via a
   GCS like Mission Planner / QGroundControl):
   ```
   SERIAL2_PROTOCOL = 2      # MAVLink2 on TELEM2
   SERIAL2_BAUD     = 921    # 921600
   ```
2. Point the node at that link instead of SITL:
   ```bash
   # serial (Jetson example)
   export MAVLINK_URL="/dev/ttyTHS1,921600"
   # or UDP via a telemetry radio / mavlink-router
   export MAVLINK_URL="udpin:0.0.0.0:14552"
   python3 sim_drone/drone_obstacle_avoidance.py     # (no Gazebo/bridge on hardware)
   ```
   On hardware the cameras come from `/dev/video*` (USB) or the CSI stack — reuse
   the capture code from `obstacle_avoidance_four_cameras.py` (the car build's
   real-camera entry point) in place of the ROS image subscriptions.
3. **GUIDED preconditions**: ArduCopter only enters GUIDED with a healthy
   position estimate and keeps it only while setpoints stream (`> ~3 Hz`, which
   `SETPOINT_HZ = 10` satisfies). Stop streaming → it falls back to Loiter.

### 4.2 Re-think auto-arm / auto-takeoff for the field

The sim node **arms and takes off itself** for convenience. On a real aircraft
that is unsafe. The recommended field flow:

- The **pilot** arms and takes off manually in **Loiter/AltHold** via RC.
- Once stable, switch to **GUIDED** (RC switch) and let the companion stream the
  filtered velocity setpoints.
- Keep an **RC mode switch as the e-stop**: flipping back to Loiter/Stabilize
  instantly returns full manual control and ignores the companion.
- Disable the script's takeoff (`Pilot.takeoff`) for hardware, or guard it behind
  an explicit "I am on the bench with props off" flag.

### 4.3 Metric distance gating (recommended)

For each direction, OR a metric sensor reading into the `STOP` flag so the stop
distance is real, not relative:

```python
STOP_M = 0.8     # hard stop under 0.8 m (tune for mass + speed + braking)
front_stop = (tof_front.range_m < STOP_M) or front_camera_stop
```

Vision then mainly answers *what* it is and gives 360° coverage; the ToF/lidar
gives the trustworthy *how far*. This directly addresses the flat-wall standoff
limitation noted in §1.

### 4.4 Running the perception on the companion

Same model-export path as the car build (see
[`car_obstacle_avoidance.md` §4](car_obstacle_avoidance.md)) — the only
difference is the companion class:

| Companion | AI accel | Convert with | Runtime | Notes |
| --------- | -------- | ------------ | ------- | ----- |
| **Jetson Orin Nano/NX** | CUDA GPU | — (TensorRT optional) | PyTorch/TensorRT | **runs this exact torch+CUDA pipeline as-is**; easiest path |
| RPi 5 + Hailo-8/8L | 13–26 TOPS | Hailo DFC | HailoRT | YOLO on Hailo + ToF is the pragmatic build |
| D-Robotics RDK X5 | ~10 TOPS BPU | `hb_mapper` | hobot_dnn | turnkey vision stack |
| Radxa ROCK 5C | 6 TOPS NPU | rknn-toolkit2 | rknnlite2 | low cost |

Depth Anything across 4 cameras is heavy on edge NPUs — lower `DEPTH_INPUT`
(multiple of 14), process cameras round-robin, or drop vision depth and gate on
**YOLO + ToF**. On Jetson with CUDA you can keep the full pipeline.

### 4.5 Autostart on boot (companion, systemd)

```ini
# /etc/systemd/system/quad-avoidance.service
[Unit]
Description=4-camera quadcopter obstacle avoidance (companion)
After=multi-user.target

[Service]
Environment=MAVLINK_URL=/dev/ttyTHS1,921600
ExecStart=/usr/bin/python3 /home/USER/obstacle_dection_four_cameras/sim_drone/drone_obstacle_avoidance.py
Restart=on-failure
User=USER

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now quad-avoidance.service
journalctl -u quad-avoidance -f
```

---

## 5. Tuning & troubleshooting

**Perception thresholds** (top of `drone_obstacle_avoidance.py`):

| Knob | Default | Effect |
| ---- | ------- | ------ |
| `CLOSE_THRESH` | 0.70 | higher = only very near pixels count as close |
| `WARN_FRAC` | 0.10 | zone → CAUTION |
| `STOP_FRAC` | 0.25 | zone → STOP (raise if it stops too eagerly) |
| `N_ZONES` | 3 | bump to 5 for finer L/C/R resolution |
| `DEPTH_INPUT` | 308 | lower (multiple of 14) for more FPS on edge |

**Flight / control knobs**:

| Knob | Default | Effect |
| ---- | ------- | ------ |
| `TAKEOFF_ALT` | 3.0 m | takeoff altitude (env `TAKEOFF_ALT`) |
| `STRAFE_SPEED` | 1.0 m/s | sideways dodge speed when a side is blocked |
| `YAW_AWAY` | 0.4 rad/s | yaw rate injected to turn the nose away from a blocked side |
| `SETPOINT_HZ` | 10 Hz | setpoint stream rate (keep ≥ ~3 Hz or GUIDED drops to Loiter) |
| `MAVLINK_URL` | `tcp:127.0.0.1:5760` | SITL endpoint; on hardware a serial/UDP link |

| Symptom | Fix |
| ------- | --- |
| `SIM_VEHICLE: MAVProxy exited`, SITL dies on launch | use the headless `--no-mavproxy` path (§3.3); don't pass `--console` under a detached launch |
| Node logs only `HEARTBEAT`, takeoff never "reaches" altitude | data streams not requested — the node now calls `request_data_stream_send`; confirm `GLOBAL_POSITION_INT` is arriving |
| `PERCEPTION-ONLY` mode in the grid | no MAVLink heartbeat — SITL not up yet, or `MAVLINK_URL` wrong, or another client already holds `5760` |
| Won't arm in SITL | wait for GPS/EKF (`GPS_RAW_INT.fix_type ≥ 3`); the node retries arming ~20× while EKF settles |
| A camera tile shows `NO SIGNAL` | bridge not running, or restart it after restarting Gazebo (it won't re-discover a new gz instance) |
| Drone gets too close to a flat wall before stopping | relative depth saturates on flat fills — lower the approach speed and add **ToF gating** (§4.3) |
| Camera topics dead | check `run_bridge_iris.sh` and that camera `<topic>`s in `model.sdf` match `iris_cams_bridge.yaml` |
| GUIDED keeps dropping to Loiter | setpoints not streaming fast enough — verify the streamer thread is alive and `SETPOINT_HZ ≥ 3` |

---

## 6. Safety notes

- **Validate in SITL first** (§3), then bench-test with **props removed** and the
  motors disarmed/limited before any flight.
- **Keep an RC e-stop.** A mode switch back to Loiter/Stabilize must instantly
  override the companion. The vision filter gates *direction*; it does not replace
  a manual takeover or a hardware kill switch.
- **The filter is open-loop on speed** — it gates which way you may go, it does
  not brake for you. Tune approach speed, `STRAFE_SPEED` and (ideally) a metric
  `STOP_M` for your mass and braking.
- **Relative depth is not metric** and is worst against large flat surfaces and
  in glare/bright sky (which can read as "close"). Use ToF/lidar for the actual
  stop decision outdoors and near walls.
- **GUIDED needs a position estimate** (GPS outdoors; optical-flow+rangefinder or
  VIO indoors) and a streaming setpoint link — losing either drops it to a
  failsafe mode. Configure ArduCopter geofence, RTL and battery failsafes
  independently of this node.
- **Don't auto-arm/auto-takeoff on a real aircraft** — that convenience is for the
  simulator only (§4.2).
```
