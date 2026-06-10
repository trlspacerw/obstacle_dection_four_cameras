# VTOL Camera-Grid Obstacle Avoidance — Process Log & Hardware Deployment

This document is the **full record of how the VTOL (quadplane) camera-grid
obstacle-avoidance system was designed, built, brought up in simulation, and
debugged**, followed by a **step-by-step guide for deploying it on real
hardware**. It complements the reference README
[`vtol_obstacle_avoidance.md`](vtol_obstacle_avoidance.md) (which is the "how to
run it" manual); this file is the "how it was made and what we learned" log.

Sibling builds it reuses concepts from:
- [`car_obstacle_avoidance.md`](car_obstacle_avoidance.md) — ground robot (the original perception + grid).
- [`quadcopter_obstacle_avoidance.md`](quadcopter_obstacle_avoidance.md) — multirotor (the MAVLink control pattern).

---

## 0. The concept in one paragraph

A VTOL quadplane is a hybrid: it **hovers like a quadcopter** (lift motors) and
**cruises like a fixed-wing** (pusher + wing). The avoidance system carries
**four cameras** (front/back/left/right), each running YOLOv8n detection +
Depth-Anything monocular depth, with every view split into left/center/right
zones. The key idea is that the behaviour is **mode-aware**:

- **VTOL hover** → use **all four cameras** (the aircraft is holonomic, can stop,
  reverse, strafe, yaw → full omnidirectional avoidance).
- **Fixed-wing cruise** → use **the front camera only** (a cruising wing can't
  stop/reverse/strafe and is only going forward, so only the forward view
  matters; the other three are not even processed → saves compute at speed).

The regime is read from MAVLink `EXTENDED_SYS_STATE.vtol_state` (`MC` vs `FW`),
which is correct regardless of the named flight mode.

---

## 1. How it was built

### 1.1 Choosing the airframe and SITL stack

The environment already had a full ArduPilot + Gazebo setup. The investigation
found:

| Asset | Where | Use |
| ----- | ----- | --- |
| `alti_transition_quad` (quadplane model: 4 lift motors + pusher `motor_f`, ailerons, elevator, ArduPilot plugin on FDM port 9002) | `~/ardu_ws/src/ardupilot_sitl_models/Gazebo/models` (already cloned + built) | the airframe |
| `alti_transition_quad.param` | `…/SITL_Models/Gazebo/config/` | the tuned ArduPlane params |
| ArduPlane SITL `quadplane` frames | `~/ardupilot` | the autopilot |
| Run recipe | `…/SITL_Models/Gazebo/docs/AltiTransition.md` | `gz sim alti_transition_runway.sdf` + `sim_vehicle.py -v ArduPlane --model JSON --add-param-file=…` |

### 1.2 The project (`sim_vtol/`)

```
sim_vtol/
├── models/alti_four_cam/        Alti quadplane + 4 cameras grafted on base_link
│   ├── model.sdf                (merge="true" include of alti_transition_quad + 4 camera links)
│   └── model.config
├── obstacle_world_alti.sdf      ground + hover obstacles (sides) + cruise towers (downrange +x)
├── alti_cams_bridge.yaml        Gazebo -> ROS 2 bridge (4 cams + clock)
├── vtol_obstacle_avoidance.py   the mode-aware node
├── run_gazebo_alti.sh           T1
├── run_sitl_alti.sh             T2 (ArduPlane, headless --no-mavproxy)
├── run_bridge_alti.sh           T3
├── run_avoidance_alti.sh        T4
└── run_teleop.sh                T5 (hover-only teleop)
```

Design decisions:

- **Cameras on `base_link`** of the merged quadplane, front at the nose, the
  other three around the fuselage. Topics `/cam_front|back|left|right`.
- **World layout** deliberately separates the two demos: low obstacles to the
  **left/right/back** of the spawn (so they don't block the forward transition)
  for the **hover** demo, and **tall towers downrange along +x at cruise
  altitude** for the **cruise** demo.
- **Mode-aware node** classifies regime from `EXTENDED_SYS_STATE`:
  - **MC** → process 4 frames, 2×2 grid, omnidirectional safety filter (block
    fwd/rev, strafe + yaw away, "boxed in"). Output: GUIDED body-velocity.
  - **FW** → process only the front frame, grey out the other three tiles,
    map front L/C/R zones to **bank away + climb**. Output: GUIDED body-offset
    position dodge (look-ahead + lateral + climb).
- Reused the **headless lessons** from the copter build: SITL runs
  `--no-mavproxy`, the node talks straight to `tcp:127.0.0.1:5760`, and it
  `request_data_stream`s after the heartbeat.

### 1.3 Static validation

Before running anything: `python3 -m py_compile` (syntax OK), XML parse of all
SDFs (well-formed), and a check that every MAVLink constant used
(`MAV_VTOL_STATE_*`, `MAVLINK_MSG_ID_EXTENDED_SYS_STATE`, `MAV_FRAME_BODY_OFFSET_NED`,
…) exists in the installed `pymavlink`.

---

## 2. How it was brought up and validated (live)

The stack was launched detached (`setsid … </dev/null`, perception on
`DISPLAY=:1`) in the lock-step order **Gazebo → SITL → bridge → node**, with logs
in `/tmp/vtol_sim_logs/`.

| Step | Observed result |
| ---- | --------------- |
| Gazebo loads the world | `alti` model spawned, **all 4 camera sensors recognized** and rendering (only cosmetic Ogre/QML warnings) |
| ArduPlane SITL | built in **18.8 s**, came up listening on **`5760`**, JSON-FDM coupled to Gazebo |
| Camera bridge | 4 ROS image topics publishing at **~10 Hz** |
| MAVLink probe | ArduPlane (plane type), **GPS fix 6 / 10 sats**, **`VTOL_STATE = MC`** on the ground |
| Node start | connected, set `Q_GUIDED_MODE=1`, GUIDED → **armed → VTOL takeoff to ~14 m** |
| **MC hover** | grid showed the **2×2 four-camera** view, tag `GUIDED [MC/HOVER] 4 cams`, depth+YOLO+zones all live |
| Transition (CRUISE) | airspeed built **0 → 20 m/s**, **`VTOL_STATE → FW`** at t≈3.6 s |
| **FW cruise** | grid switched to **front camera only**, other three tiles `OFF (FW cruise)`, tag `GUIDED [FW/CRUISE]: FRONT cam only`, **FPS rose to 15.6** (compute saved by dropping 3 views) |
| Front dodge | node took over GUIDED and banked/climbed near the first tower |

**Both regimes were confirmed end-to-end** — including the headline requirement
that *only the front camera is active in cruise*.

---

## 3. Problems encountered and how they were solved

This is the part most worth keeping — the non-obvious things that bit during
bring-up.

1. **ArduPlane GUIDED ignores velocity setpoints.**
   *Symptom:* a teleop climb command did nothing — altitude stayed at 15 m.
   *Cause:* unlike ArduCopter, **ArduPlane GUIDED is position/waypoint-based**;
   it does not act on streamed body-velocity setpoints (even with
   `Q_GUIDED_MODE=1`).
   *Fix/▲lesson:* real hover actuation on a quadplane must use **position
   offsets** (`MAV_FRAME_BODY_OFFSET_NED` *position*), not velocity. (Flagged as
   remaining polish — see §6.)

2. **Streaming zero-velocity "holds" blocked the transition.**
   *Symptom:* commanding CRUISE/transition while the node ran left the aircraft
   stuck in `MC`.
   *Cause:* the node streamed a zero-velocity GUIDED setpoint at 10 Hz, which
   ArduPlane interpreted as "hold position" — preventing forward acceleration.
   *Fix:* the node now only sends a hover setpoint **when the command is
   non-zero**.

3. **`GUIDED` + `DO_VTOL_TRANSITION` alone won't transition.**
   *Symptom:* VTOL state never left `MC`.
   *Cause:* a quadplane needs **forward airspeed** to hand off from lift motors
   to the wing; re-entering `GUIDED` also didn't pick up a changed
   `Q_GUIDED_MODE`.
   *Fix:* trigger the transition with a **fixed-wing auto-throttle mode
   (`CRUISE`)** (or an AUTO mission) — that actively builds airspeed. It then
   flipped to `FW` in ~3.6 s.

4. **Only one MAVLink client on `tcp:5760`.**
   *Problem:* the node owns 5760, so a second connection to command the
   transition would conflict.
   *Fix:* ArduPlane SITL also exposes **`5762`/`5763`** — the transition was
   driven there while the node kept 5760 for perception.

5. **Re-attaching the node to an airborne aircraft.**
   *Need:* restart the node mid-flight without it re-running arm/takeoff.
   *Fix:* added a **`SKIP_TAKEOFF=1`** env (assume already airborne, just
   stream/observe).

6. **`pkill -f` self-kill.**
   *Symptom:* a cleanup command exited 1 and didn't run the relaunch.
   *Cause:* the command line literally contained `vtol_obstacle_avoidance.py`,
   which matched the kill pattern — `pkill` killed its own shell.
   *Fix:* use a **bracketed pattern** (`vtol_obstacle_[a]voidance`) and don't put
   the literal target string elsewhere on the same command line.

7. **Cameras see the aircraft's own wings.**
   *Symptom:* side/front tiles showed the orange wing as a "close" obstacle →
   false STOP.
   *Cause:* camera placement on `base_link` puts the swept wing in view.
   *Fix/▲lesson:* reposition the cameras forward of / clear of the wing, or mask
   the wing region. (Remaining polish — §6.)

---

## 4. Re-running it in simulation (quick recipe)

Full detail in [`vtol_obstacle_avoidance.md` §3](vtol_obstacle_avoidance.md).
Short version (4 terminals, lock-step order):

```bash
cd sim_vtol
./run_gazebo_alti.sh        # T1  (start first)
./run_sitl_alti.sh          # T2  (first run compiles ArduPlane ~1–2 min)
./run_bridge_alti.sh        # T3
./run_avoidance_alti.sh     # T4  (arms, VTOL takeoff, opens the grid)
```

To reproduce the cruise demo: once hovering, climb to a safe altitude, then put
the aircraft in **CRUISE** (fixed-wing auto-throttle) so it transitions and flies
the +x corridor; the grid switches to front-only and dodges the towers. To attach
the node to an aircraft that is already flying: `SKIP_TAKEOFF=1 ./run_avoidance_alti.sh`.

Teardown (bracketed patterns):

```bash
pkill -9 -f "gz[ ]sim";    pkill -9 -f "sim_[v]ehicle";  pkill -9 -f "[a]rduplane"
pkill -9 -f "[p]arameter_bridge";  pkill -9 -f "vtol_obstacle_[a]voidance"
```

---

## 5. Deploying to real hardware — step by step

On a real aircraft the **flight controller** (Pixhawk/Cube running ArduPlane
QuadPlane) does stabilization, EKF, transition and motor mixing. The **companion
computer** runs perception + the mode switch and sends MAVLink to the FC. The
control path is already what the sim uses — you point `MAVLINK_URL` at a real FC.

```
        ┌──────────────────────────────┐        ┌──────────────────────────┐
 4 cams │ Companion (Jetson / RPi+Hailo)│ MAVLink│ Flight controller        │
 ──────►│  vtol_obstacle_avoidance.py   │───────►│  ArduPlane QuadPlane      │
        │  perception + mode switch     │ serial │  (lift motors + pusher)   │
        └──────────────────────────────┘ or UDP └──────────────────────────┘
```

### Step 0 — Bill of materials

| Part | Notes |
| ---- | ----- |
| VTOL quadplane airframe | enough payload margin for the companion + 4 cameras |
| Flight controller | Pixhawk 6C / Cube Orange, **ArduPlane** QuadPlane firmware |
| Companion computer | **Jetson Orin Nano/NX (CUDA → runs this torch pipeline unchanged)**; or RPi 5 + Hailo, RDK X5, ROCK 5C |
| 4 × cameras | front = critical (only one used in cruise); global-shutter preferred; USB-UVC or CSI/GMSL |
| **Forward metric sensor** | radar or long-range lidar for the cruise STOP (relative depth is not metric and a wing has a turn radius) |
| Side/rear ToF (optional) | metric gating for hover near obstacles |
| GPS + compass | required for GUIDED; add optical-flow/rangefinder or VIO for indoor/GPS-denied |
| Power | regulated 5 V BEC for the companion + cameras, separate from ESC power |
| FC↔companion link | UART (TELEM2) or USB |

### Step 1 — Build and tune the quadplane (autopilot side)

1. Assemble the airframe; wire the 4 lift ESCs + pusher ESC + aileron/elevator
   servos to the FC per your QuadPlane layout.
2. Flash **ArduPlane** and configure it as a quadplane: `Q_ENABLE=1`, set the
   frame class/type, assign the VTOL motor outputs and the forward-motor
   `SERVOx_FUNCTION=70 (throttle)`, ailerons/elevator functions.
3. Calibrate accel/compass/RC/ESCs; set the battery + power monitor.
4. **Tune in three phases, in this order, before any avoidance:**
   - VTOL hover (`Q_A_*` rate/attitude, hover throttle),
   - fixed-wing (`RLL2SRV/PTCH2SRV`, `TRIM_ARSPD_CM`, `ARSPD_FBW_MIN/MAX`),
   - **transition** (`Q_TRANSITION_MS`, `Q_ASSIST_SPEED`, `Q_RTL_MODE`).
   Use the `alti_transition_quad.param` as a *starting reference only* — every
   airframe is different.
5. Verify a clean **manual** VTOL takeoff → transition to `CRUISE` → back-
   transition → VTOL land, with an RC pilot, before adding the companion.

### Step 2 — Mount and verify the cameras

1. Mount the **front** camera with an unobstructed forward view, vibration-
   isolated, **clear of the props and the wing** (the sim showed wings in frame —
   avoid that). It is the only camera used in cruise, so prioritize it.
2. Mount back/left/right for hover coverage; angle them so the wing/booms are out
   of frame.
3. On the companion: `v4l2-ctl --list-devices` (USB) or the CSI stack; record
   each `/dev/video*` index and confirm physical order matches `CAM_DIRECTIONS =
   ["FRONT","RIGHT","BACK","LEFT"]`.

### Step 3 — Prepare the companion computer

1. Flash the vendor OS (Jetson: JetPack; Pi: Bookworm 64-bit; etc.).
2. Install perception deps: `pip install ultralytics transformers opencv-python
   numpy pymavlink` plus the board's torch (Jetson: the JetPack torch wheel; CUDA
   works out of the box). On non-CUDA boards, export YOLOv8n / Depth-Anything to
   the board's accelerator (RKNN / Horizon `hb_mapper` / Hailo DFC) — see
   [`car_obstacle_avoidance.md` §4](car_obstacle_avoidance.md).
3. Copy `yolov8n.pt` and `sim_vtol/vtol_obstacle_avoidance.py` to the companion.

### Step 4 — Wire the companion to the flight controller

1. Connect the companion UART to the FC `TELEM2` (or USB).
2. On the FC, set the companion serial to MAVLink2:
   ```
   SERIAL2_PROTOCOL = 2        # MAVLink2
   SERIAL2_BAUD     = 921      # 921600
   Q_GUIDED_MODE    = 1        # GUIDED behaves as VTOL for hover ops
   ```
3. Point the node at the link:
   ```bash
   export MAVLINK_URL="/dev/ttyTHS1,921600"     # Jetson UART example
   # or, over a network/telemetry-radio bridge:
   export MAVLINK_URL="udpin:0.0.0.0:14552"
   ```

### Step 5 — Adapt the node for real cameras and real control

The sim node reads ROS image topics (from Gazebo) and auto-arms/takes off — both
must change for hardware:

1. **Cameras:** replace the ROS `Image` subscriptions with direct capture from
   `/dev/video*` (or CSI) — reuse the capture loop from
   `obstacle_avoidance_four_cameras.py` (the car build's real-camera entry point).
   Everything downstream (YOLO, depth, zones, the mode switch) stays the same.
2. **Mode switch:** keep `request_data_stream` + `SET_MESSAGE_INTERVAL` for
   `EXTENDED_SYS_STATE` — the node already derives MC/FW from it; no change.
3. **Hover actuation (the §3 fix to apply):** for real hover dodging, send
   **position offsets** (`MAV_FRAME_BODY_OFFSET_NED`, position fields), not
   velocity, because ArduPlane GUIDED is position-based.
4. **Cruise actuation:** the existing GUIDED **body-offset position dodge**
   (look-ahead + lateral + climb) is already ArduPlane-correct.
5. **Disable auto-arm / auto-takeoff** in the field — run the node with logic
   that only *filters/dodges* and never arms or commands takeoff. The pilot arms
   and takes off manually.
6. **Add the metric forward sensor** into the front STOP:
   ```python
   STOP_M = 30.0   # tune for cruise speed + turn radius
   front_stop = (radar.range_m < STOP_M) or front_camera_stop
   ```

### Step 6 — Bench test (props OFF)

1. Power the FC + companion on the bench, props removed.
2. Run the node; confirm: 4 camera feeds, `VTOL_STATE = MC` on the ground, the
   grid renders, the decision/AVOID logic fires when you wave an obstacle in
   front, and MAVLink setpoints are being sent (watch the FC's MAVLink inspector
   in a GCS).
3. Confirm the node **never arms** on its own.

### Step 7 — Hover flight test (low, open area, RC pilot ready)

1. Pilot does a manual VTOL takeoff to a few metres in QLOITER/QHOVER.
2. Enable the companion (switch to GUIDED-VTOL). Present a large soft obstacle to
   one side and verify the hover dodge (position-offset) moves the aircraft away.
3. **Keep an RC mode switch as the e-stop** — flipping to QLOITER/QSTABILIZE
   instantly reclaims manual control. Test that takeover first.

### Step 8 — Transition + cruise flight test

1. At a safe altitude over open ground, transition to `CRUISE`.
2. Confirm the grid (or telemetry) shows the node switch to **front-camera-only**
   in the FW state, and that FPS/compute drops.
3. Fly toward a large, isolated, soft obstacle; verify the front dodge banks
   away + climbs with adequate margin (this is where `FW_LOOKAHEAD` / `FW_DODGE_Y`
   and the **metric forward sensor** matter — relative depth alone gives too
   little warning at cruise speed).
4. Decide the hand-back policy: after clearing, return control to the mission/AUTO
   (the sim left it holding the dodge target — production needs an explicit
   resume).

### Step 9 — Failsafes and autostart

1. Configure ArduPlane geofence, battery failsafe, RC-loss and GCS-loss failsafes
   **independently** of this node — the companion is an assist layer, not the
   safety net.
2. Autostart the node on the companion with systemd (set `MAVLINK_URL` in the
   unit `Environment=`); `journalctl -u <service> -f` to watch logs.

---

## 6. Status and remaining polish

**Validated (2026-06-08):** build → static checks → live bring-up → VTOL takeoff
→ MC 4-camera hover grid → CRUISE transition → FW front-camera-only grid →
front dodge. Both regimes confirmed.

**Remaining polish (not yet done):**
- Rework **VTOL hover actuation** to position-offset control (ArduPlane GUIDED is
  position-based; velocity setpoints don't move it).
- Fix **camera placement** so the wings are out of frame (or mask them).
- Tune the **cruise dodge geometry** (`FW_LOOKAHEAD`, `FW_DODGE_Y`, `FW_CLIMB`)
  and add a **resume-after-dodge** hand-back to the mission.
- Add the **metric forward sensor** path for a trustworthy cruise stop distance.

---

## 7. Safety notes

- Validate in SITL, then bench (props off), then low hover, then cruise — in that
  order, with an RC e-stop available at every flight step.
- **Cruise avoidance is geometry-limited:** a wing can't stop and has a turn
  radius — size the look-ahead/dodge for your speed and rely on a **metric**
  forward sensor, not relative depth, for the real trigger.
- **Transitions are the riskiest phase** — treat `TRANSITION_*` states
  conservatively.
- **Never auto-arm / auto-takeoff in the field** (sim convenience only).
- The companion is an **assist layer**; ArduPlane's own geofence/RTL/failsafes
  remain the safety net.
```
