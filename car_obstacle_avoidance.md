# 4-Camera Obstacle-Avoidance Car — Build, Simulate & Deploy

A DJI-style, 360°-ish obstacle-avoidance system for a small ground robot using
**four cameras** (front / back / left / right). Each camera runs object
detection + monocular depth, every view is split into left/center/right zones,
and the per-zone proximity drives a **closed-loop safety layer** that filters
the driver's command:

| Direction in `STOP` | Action |
| ------------------- | ------ |
| **FRONT** | block forward motion |
| **BACK**  | block reverse motion |
| **LEFT**  | auto-steer **right** (turn away) |
| **RIGHT** | auto-steer **left** (turn away) |
| **LEFT + RIGHT** | "boxed in" — no auto-turn, manual only |

This document covers the whole path:

1. [How it works](#1-how-it-works)
2. [Repository layout](#2-repository-layout)
3. [Run it in simulation (Gazebo)](#3-run-it-in-simulation-gazebo)
4. [Deploy on real hardware](#4-deploy-on-real-hardware)
   - [Radxa ROCK 5C](#radxa-rock-5c-rk3588s2)
   - [D-Robotics RDK X5](#d-robotics-rdk-x5)
   - [Raspberry Pi 5](#raspberry-pi-5)
5. [Tuning & troubleshooting](#5-tuning--troubleshooting)
6. [Safety notes](#6-safety-notes)

---

## 1. How it works

### Perception pipeline (per frame, all 4 cameras batched)

```
                 ┌──────────────────────────┐
 4 camera   ───► │  YOLOv8n  (detection)    │ ─► bounding boxes (what)
 frames          │  Depth Anything V2 (depth)│ ─► relative depth (how close)
                 └─────────────┬────────────┘
                               │  per camera
                 ┌─────────────▼────────────┐
                 │ normalize depth 0..1      │
                 │ split L / C / R zones     │
                 │ count "close" pixels      │
                 │ -> CLEAR / CAUTION / STOP │
                 └─────────────┬────────────┘
                               │  4 × (zone statuses)
                 ┌─────────────▼────────────┐
                 │ closed-loop safety filter │ ─► motor / cmd_vel
                 └──────────────────────────┘
```

- **YOLOv8n** (`yolov8n.pt`, Ultralytics) — lightweight detector, batched across
  the 4 frames in one inference call.
- **Depth Anything V2 Small** (`depth-anything/Depth-Anything-V2-Small-hf`) —
  monocular **relative** depth (larger value = closer). No metric meters; good
  enough for "this zone is closer than that one".
- **Zones**: each view → 3 vertical slices. A zone is `STOP` when ≥ `STOP_FRAC`
  (25 %) of its pixels are "close" (normalized depth > `CLOSE_THRESH` = 0.70),
  `CAUTION` at ≥ 10 %, else `CLEAR`. Tunable at the top of the script.

### Control / safety layer

The driver (keyboard teleop, gamepad, or an autonomy node) publishes a raw
velocity command. The safety filter **never blocks turning** so the operator can
always steer out, and reverse stays available when the front is blocked. On a
differential-drive base, "turn away" is the only meaningful side response (it
can't strafe).

> ⚠️ **Relative depth is not metric.** "STOP" means *something is the closest
> thing in this view*, not "obstacle at 1.2 m". For dependable real-world
> distance gating, pair each camera with a cheap **ToF / ultrasonic** sensor
> (see [§4](#4-deploy-on-real-hardware)).

---

## 2. Repository layout

```
obstacle_dection_four_cameras/
├── yolov8n.pt                          YOLOv8n weights
├── detect_four_cameras.py              YOLO-only, 4 USB cams -> 2x2 grid
├── obstacle_avoidance_four_cameras.py  YOLO + depth + zones (real USB cams)
├── obstacle_avoidance.md               design notes for the perception script
├── car_obstacle_avoidance.md           (this file)
└── sim/                                Gazebo simulation twin
    ├── obstacle_world.sdf              world + 4-wheel skid-steer cam-bot
    ├── ros_gz_bridge.yaml              Gazebo <-> ROS 2 topic bridge
    ├── obstacle_avoidance_sim.py       perception + closed-loop safety (ROS 2)
    ├── run_gazebo.sh                   T1: launch Gazebo
    ├── run_bridge.sh                   T2: launch ros_gz_bridge
    ├── run_avoidance.sh                T3: launch perception/safety node
    └── run_teleop.sh                   T4: keyboard teleop -> /cmd_vel_teleop
```

Two entry points:

- **`obstacle_avoidance_four_cameras.py`** — opens `/dev/video*` directly. This
  is the base for **real hardware** (it just needs a motor-output stage added).
- **`sim/obstacle_avoidance_sim.py`** — same pipeline, but frames come from
  Gazebo over ROS 2 and the command goes back to a simulated diff-drive. It also
  contains the finished **closed-loop safety filter** you'll port to hardware.

---

## 3. Run it in simulation (Gazebo)

Validated stack: **Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim 8 (Harmonic)** with
an NVIDIA GPU (CUDA) on the dev machine.

### 3.1 Dependencies

```bash
# ROS 2 Humble + Gazebo Harmonic + bridge
sudo apt install ros-humble-desktop ros-humble-ros-gz \
                 ros-humble-teleop-twist-keyboard

# Python perception deps (into the env that runs the node)
pip install ultralytics transformers "pillow>=10" opencv-python
# torch with CUDA matching your machine, e.g. from pytorch.org
```

### 3.2 Launch (4 terminals)

```bash
cd sim

# Terminal 1 — Gazebo with the obstacle world + 4-camera skid-steer bot
./run_gazebo.sh

# Terminal 2 — bridge the 4 cameras, /cmd_vel and /odom into ROS 2
./run_bridge.sh

# Terminal 3 — perception + closed-loop safety (YOLO + depth, opens the 2x2 grid)
./run_avoidance.sh

# Terminal 4 — drive it (HOLD keys; needs keyboard focus on this terminal)
./run_teleop.sh
```

Command flow:

```
teleop_twist_keyboard ──► /cmd_vel_teleop ──► [avoidance node: safety filter]
                                                        │
                                                        ▼
                                    /cmd_vel ──► ros_gz_bridge ──► Gazebo diff-drive
```

Drive toward the colored boxes: the grid window shows the depth heatmap, the
per-zone bars (green/yellow/red), and a red **`AVOID:`** banner when an override
engages (`STOP FWD`, `STOP REV`, `TURN LEFT/RIGHT`, `BOXED IN`).

### 3.3 Stop everything

```bash
pkill -9 -f "gz sim"; pkill -9 -f parameter_bridge
pkill -9 -f 'obstacle_avoidance_[s]im'
```

> Tip: when scripting restarts, match the node with a bracketed pattern like
> `obstacle_avoidance_[s]im` so `pkill -f` doesn't also match (and kill) your own
> shell command line.

---

## 4. Deploy on real hardware

The dev machine and simulation use **PyTorch + CUDA**. The target SBCs
(ROCK 5C / RDK X5 / Pi 5) have **no CUDA** — they accelerate inference on an
**NPU/BPU** instead. So deployment = (a) convert the models to the board's
runtime, and (b) replace the Gazebo diff-drive with a real motor driver.

### 4.0 Bill of materials

| Part | Notes |
| ---- | ----- |
| SBC | ROCK 5C, RDK X5, **or** Pi 5 (+ Hailo AI Kit) |
| 4 × cameras | USB UVC (simplest), or mix CSI + USB. 640×480 @ 15 fps is plenty |
| Powered USB hub | 4 UVC cams saturate one bus — use a hub / separate controllers |
| Chassis | 2- or 4-wheel differential drive |
| Motor driver | TB6612FNG / L298N (PWM+DIR), or a UART/I²C motor controller |
| **Distance sensors (recommended)** | 4 × VL53L1X (ToF, I²C) or HC-SR04 (ultrasonic) — one per direction for *metric* gating |
| Power | Battery + 5 V regulator for the SBC, separate motor supply |

A robust real robot uses the **cameras for classification** (what is it) and
**ToF/ultrasonic for the actual stop distance** (how far). Vision depth alone is
a useful fallback but is relative, not metric.

### 4.1 Common steps (all boards)

1. **Flash the vendor OS** (Ubuntu/Debian based) and boot to a desktop or
   headless SSH session.
2. **Verify the 4 cameras**:
   ```bash
   sudo apt install v4l-utils
   v4l2-ctl --list-devices          # find the /dev/video* of each camera
   ```
   Set the right indices in `CAMERAS` and confirm physical order in
   `CAM_DIRECTIONS` (`["FRONT","RIGHT","BACK","LEFT"]`).
3. **Python deps** (CPU/OpenCV side):
   ```bash
   pip install opencv-python numpy
   ```
   Do **not** install CUDA `torch` on these boards — see the per-board NPU path.
4. **Export the models to ONNX** on your x86 dev machine (once):
   ```bash
   yolo export model=yolov8n.pt format=onnx opset=12 imgsz=480
   # Depth Anything V2 Small -> ONNX (optional; heavier on edge):
   #   use transformers + torch.onnx.export, or Optimum, at DEPTH_INPUT=252/196
   ```
   Then convert the ONNX to the board's NPU format (per board below).

### 4.2 The control mapping (replace Gazebo with motors)

In `sim/obstacle_avoidance_sim.py` the safety policy lives in `publish_cmd()`.
On hardware, port that exact logic and drive motors instead of publishing a
Twist. The differential-drive mixer:

```python
# v = forward velocity (m/s-ish), w = yaw rate (rad/s-ish), both in [-1, 1]
def drive(v, w):
    left  = clamp(v - w, -1, 1)
    right = clamp(v + w, -1, 1)
    set_motor(LEFT,  left)     # PWM duty + DIR pin
    set_motor(RIGHT, right)

# safety filter (identical policy to the sim)
def safe(v, w, front, back, left, right):     # *_stop booleans from zones/ToF
    if front and v > 0: v = 0
    if back  and v < 0: v = 0
    if left  and not right: w = -TURN_AWAY     # obstacle left -> turn right
    elif right and not left: w = +TURN_AWAY    # obstacle right -> turn left
    return v, w
```

Skeleton hardware loop (board-agnostic; swap the inference + GPIO calls):

```python
caps = [open_camera(i) for i in CAMERAS]      # from obstacle_avoidance_four_cameras.py
while True:
    frames = [c.read()[1] for c in caps]
    dets   = npu_detect(frames)               # board NPU (YOLOv8n)
    depth  = npu_depth(frames)                # board NPU, OR read ToF sensors
    stops  = {d: any(z == "STOP" for z in zone_statuses(depth[i]))
              for i, d in enumerate(CAM_DIRECTIONS)}
    v, w = read_driver()                       # gamepad / keyboard / autonomy
    v, w = safe(v, w, stops["FRONT"], stops["BACK"], stops["LEFT"], stops["RIGHT"])
    drive(v, w)
```

> If you'd rather keep ROS 2 on the robot, run `obstacle_avoidance_sim.py`
> unchanged and write a tiny `/cmd_vel` → motor-driver node (replacing
> `ros_gz_bridge`). Either way the safety logic is reused as-is.

### 4.3 Distance-sensor gating (recommended, optional)

For each direction, read a ToF/ultrasonic distance and OR it into the stop flag:

```python
STOP_M = 0.40    # hard stop under 40 cm
front_stop = (tof_front.range_m < STOP_M) or front_camera_stop
```

This gives a reliable, metric trigger and lets you trust the robot near walls
where relative depth is ambiguous.

---

### Radxa ROCK 5C (RK3588S2)

- **SoC / accel:** RK3588S2, Mali-G610, **6 TOPS NPU (RKNPU2)**. No CUDA.
- **OS:** Radxa OS (Debian 12) / Armbian / Ubuntu.
- **Inference path:** convert ONNX → `.rknn` with **rknn-toolkit2** (on an x86
  host), run on the board with **rknn-toolkit-lite2** / the `rknpu2` runtime.

```bash
# --- on x86 host (one time) ---
pip install rknn-toolkit2
# Python: load yolov8n.onnx, set target_platform='rk3588', build & export yolov8n.rknn

# --- on the ROCK 5C ---
sudo apt install python3-rknnlite2   # or pip install rknn-toolkit-lite2
# ensure the NPU driver is present:
sudo cat /sys/kernel/debug/rknpu/load
```

- **GPIO/PWM:** 40-pin header. Use `libgpiod`/`python3-periphery` for DIR pins
  and a hardware-PWM channel (or an external TB6612/serial motor controller) for
  speed.
- **Throughput tip:** YOLOv8n runs comfortably on the NPU. Depth Anything V2 at
  4 cams is heavy — lower `DEPTH_INPUT` to 196/252, process cameras round-robin,
  or prefer ToF gating and use vision mainly for classification.

### D-Robotics RDK X5

- **SoC / accel:** Sunrise 5, **~10 TOPS BPU**. No CUDA.
- **OS:** RDK OS (Ubuntu 22.04).
- **Inference path:** convert ONNX with the **Horizon `hb_mapper`** toolchain
  → `.bin`, run with **`hobot_dnn` / `pydnn`** on the BPU. Their model zoo ships
  a ready YOLOv8 example — start from it.

```bash
# --- model conversion uses Horizon's OpenExplorer/hb_mapper docker on x86 ---
#   hb_mapper makertbin --config yolov8n_config.yaml   -> yolov8n.bin
# --- on the RDK X5 ---
python3 -c "import hobot_dnn; print('BPU runtime OK')"
```

- **GPIO:** `Hobot.GPIO` (RPi.GPIO-compatible API) for DIR/PWM, or an external
  motor controller over UART/I²C.
- RDK X5 has a strong, well-documented BPU + camera stack; it's the most
  "batteries-included" of the three for vision robotics.

### Raspberry Pi 5

- **SoC:** BCM2712 — **no on-board NPU**. CPU-only inference is too slow for
  4-camera real-time.
- **Accelerator (strongly recommended):** **Hailo AI Kit** — Hailo-8L (13 TOPS)
  or Hailo-8 (26 TOPS) on the M.2 HAT+.
- **OS:** Raspberry Pi OS (Bookworm) 64-bit.

```bash
# Hailo stack
sudo apt install hailo-all          # HailoRT + Tappas + driver
hailortcli fw-control identify      # verify the M.2 module
# Use the Hailo Model Zoo's compiled YOLOv8n .hef, or compile your ONNX
# with the Hailo Dataflow Compiler (on x86) -> yolov8n.hef
```

- **Cameras:** 4 × USB UVC on a powered hub, or 2 × CSI (Pi cameras via
  `libcamera`/`picamera2`) + 2 × USB.
- **GPIO:** on Pi 5 use **`lgpio`/`gpiozero`** (legacy `RPi.GPIO` doesn't work on
  the new RP1 I/O chip) for DIR pins; hardware PWM or a TB6612/L298N for speed.
- **Depth:** Depth Anything on Hailo-8 is possible but tight at 4 cams; the
  pragmatic Pi 5 build is **YOLO on Hailo + 4 ToF sensors** for the stop logic.

### Board comparison

| | ROCK 5C | RDK X5 | Pi 5 + Hailo-8L |
| --- | --- | --- | --- |
| AI accel | 6 TOPS NPU | ~10 TOPS BPU | 13 TOPS (add-on) |
| Convert with | rknn-toolkit2 | hb_mapper | Hailo DFC |
| Runtime | rknnlite2 | hobot_dnn | HailoRT |
| GPIO lib | periphery/libgpiod | Hobot.GPIO | lgpio/gpiozero |
| Best for | low cost, good NPU | turnkey vision stack | Pi ecosystem |

### 4.4 Autostart on boot (systemd)

```ini
# /etc/systemd/system/obstacle-avoidance.service
[Unit]
Description=4-camera obstacle avoidance
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /home/USER/obstacle_dection_four_cameras/car_avoidance_hw.py
Restart=on-failure
User=USER

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now obstacle-avoidance.service
journalctl -u obstacle-avoidance -f      # watch logs
```

---

## 5. Tuning & troubleshooting

**Detection thresholds** (top of the perception script):

| Knob | Default | Effect |
| ---- | ------- | ------ |
| `CLOSE_THRESH` | 0.70 | higher = only very near pixels count as close |
| `WARN_FRAC` | 0.10 | zone → CAUTION |
| `STOP_FRAC` | 0.25 | zone → STOP (raise if it stops too eagerly) |
| `N_ZONES` | 3 | bump to 5 for finer L/C/R resolution |
| `DEPTH_INPUT` | 308 | lower (multiple of 14) for more FPS on edge |
| `TURN_AWAY_RATE` | 0.6 | yaw rate injected by side auto-turn |

| Symptom | Fix |
| ------- | --- |
| A camera shows `NO SIGNAL` | wrong `/dev/video*` index, or USB bus saturated → powered hub / split controllers |
| Wrong tile labels | re-order `CAM_DIRECTIONS` to your physical mounting |
| Robot won't go forward | front is in `STOP` (an obstacle fills the view). Turn or reverse; or raise `STOP_FRAC` |
| Stops too far / too late | switch to **ToF gating** with a metric `STOP_M`; vision depth is relative |
| Low FPS on edge | lower `DEPTH_INPUT`, drop to fewer cameras per cycle, or skip depth and gate on YOLO + ToF |
| `RPi.GPIO` errors on Pi 5 | use `lgpio`/`gpiozero` (RP1 chip) |

---

## 6. Safety notes

- **Always keep a manual override / e-stop.** The filter blocks dangerous
  directions but does not replace a hardware kill switch on a real vehicle.
- The filter is **open-loop on speed** — it gates direction, it does not brake.
  Tune motor deceleration and `STOP_M` for your mass and speed.
- **Validate in simulation first** (§3), then on blocks at low speed, before
  trusting it near people or stairs.
- Bright sky / glare can read as "close" to monocular depth — mask the top rows
  outdoors, or rely on ToF for the stop decision.
```
