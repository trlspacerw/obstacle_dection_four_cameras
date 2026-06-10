# Mode-Aware 4-Camera Obstacle Avoidance for a VTOL Quadplane

A 360°-ish obstacle-avoidance system for a **VTOL quadplane** (hybrid of a
quadcopter and a fixed-wing) using **four cameras**. It is the third member of
the family alongside the ground robot ([`car_obstacle_avoidance.md`](car_obstacle_avoidance.md))
and the multirotor ([`quadcopter_obstacle_avoidance.md`](quadcopter_obstacle_avoidance.md)),
and it reuses the **same perception pipeline** (YOLOv8n + Depth Anything V2 +
L/C/R zones). What's new is that the avoidance behaviour **switches with the
aircraft's flight regime**:

| Regime | Cameras used | Why / what it can do |
| ------ | ------------ | -------------------- |
| **VTOL hover** (multicopter) | **all 4** (front/back/left/right) | holonomic — can stop, reverse, strafe, yaw. Full omnidirectional avoidance |
| **Fixed-wing cruise** | **front only** | a cruising wing can't stop/reverse/strafe; it can only **bank away + climb**, and it's only going forward — so only the front camera matters |

In cruise the other three cameras are **not even processed** (no inference on
them), which saves the companion computer's compute for the one view that counts.

Contents:

1. [How it works](#1-how-it-works)
2. [Repository layout](#2-repository-layout)
3. [Run it in simulation (Gazebo + ArduPlane SITL)](#3-run-it-in-simulation-gazebo--arduplane-sitl)
4. [Deploy on a real VTOL](#4-deploy-on-a-real-vtol)
5. [Tuning & troubleshooting](#5-tuning--troubleshooting)
6. [Safety notes](#6-safety-notes)

---

## 1. How it works

### Perception (shared with the car/copter builds)

Each camera frame → YOLOv8n (what) + Depth Anything V2 (relative depth, larger =
closer) → normalize 0..1 → split into 3 vertical zones → each zone is
`CLEAR / CAUTION / STOP` by the fraction of "close" pixels (`STOP_FRAC` = 25 % of
pixels with depth > `CLOSE_THRESH` = 0.70). See
[`car_obstacle_avoidance.md` §1](car_obstacle_avoidance.md) for the full pipeline
diagram and the NPU export notes — they apply unchanged.

### The mode switch (the new part)

The node reads the aircraft's **VTOL state** from MAVLink
`EXTENDED_SYS_STATE.vtol_state` (`MAV_VTOL_STATE_MC` / `_FW` / `_TRANSITION_*`).
This is the *correct* signal — it's true regardless of the named flight mode, and
it tracks the actual transition between hover and wing-borne flight.

```
                         EXTENDED_SYS_STATE.vtol_state
                                     │
            ┌────────────────────────┴────────────────────────┐
       MC / TRANSITION_TO_MC                          FW / TRANSITION_TO_FW
            │                                                  │
   ┌────────▼─────────┐                              ┌─────────▼──────────┐
   │ VTOL HOVER       │                              │ FIXED-WING CRUISE  │
   │ run 4 cameras    │                              │ run FRONT camera   │
   │ omnidirectional  │                              │ only               │
   └────────┬─────────┘                              └─────────┬──────────┘
            │ holonomic mapping                                │ bank + climb
   FRONT STOP → block forward                         FRONT-LEFT  STOP → bank RIGHT
   BACK  STOP → block reverse                         FRONT-RIGHT STOP → bank LEFT
   LEFT  STOP → strafe right + yaw right              FRONT-CENTER STOP → CLIMB +
   RIGHT STOP → strafe left  + yaw left                  bank toward clearer side
   LEFT+RIGHT → "boxed in" (gate only)                (cannot stop / reverse / strafe)
```

### Control output

| Regime | MAVLink output |
| ------ | -------------- |
| **VTOL hover** | GUIDED **body-velocity** setpoints (`SET_POSITION_TARGET_LOCAL_NED`, `BODY_NED`, velocity + yaw-rate), streamed at `SETPOINT_HZ`. Needs `Q_GUIDED_MODE = 1` so GUIDED runs on the lift motors — the node sets this on connect. |
| **Fixed-wing cruise** | a GUIDED **body-offset position target** (`BODY_OFFSET_NED`: `FW_LOOKAHEAD` ahead, `±FW_DODGE_Y` lateral, `FW_CLIMB` up). The node switches to GUIDED to take over the dodge, then ArduPlane banks/climbs toward the offset waypoint. |

Driver intent (teleop `Twist` on `/cmd_vel_teleop`) is filtered in **hover only**;
in cruise the mission/auto flies the route and the node only intervenes to dodge.

> **Validated 2026-06-08 (and what it taught us):** full stack builds and runs;
> VTOL takeoff → multicopter-state **4-camera** grid → CRUISE-triggered
> transition → fixed-wing-state **front-camera-only** grid all confirmed live.
> Two important ArduPlane realities surfaced:
> - **ArduPlane GUIDED is position/waypoint-based, not velocity-based** (unlike
>   Copter). Streaming body-velocity setpoints does **not** move a quadplane, and
>   streaming *zero*-velocity "holds" actually blocks the forward transition. The
>   node therefore only emits a hover velocity setpoint when the command is
>   non-zero; for real hover dodging on ArduPlane use **position offsets**
>   (`MAV_FRAME_BODY_OFFSET_NED` position), not velocity.
> - **The VTOL→FW transition needs forward airspeed**, so trigger it with a
>   fixed-wing auto-throttle mode (**CRUISE**/AUTO mission), not `GUIDED` +
>   `DO_VTOL_TRANSITION` alone. `SKIP_TAKEOFF=1` lets the node attach to an
>   already-airborne aircraft.

> ⚠️ Same caveat as the copter: **relative depth is not metric** and degrades
> against large flat surfaces. At cruise speed the look-ahead is short relative
> to the aircraft's turn radius — back the front camera with a forward-looking
> **metric** sensor (radar / long-range lidar) for real deployments (§4).

---

## 2. Repository layout

```
obstacle_dection_four_cameras/
├── yolov8n.pt                          shared YOLOv8n weights
├── vtol_obstacle_avoidance.md          (this file)
└── sim_vtol/
    ├── vtol_obstacle_avoidance.py      mode-aware perception + safety + MAVLink
    ├── obstacle_world_alti.sdf         world: ground + hover obstacles + cruise towers
    ├── alti_cams_bridge.yaml           Gazebo -> ROS 2 bridge (4 cams + clock)
    ├── models/alti_four_cam/           Alti quadplane + 4 cameras (merges alti_transition_quad)
    │   ├── model.sdf
    │   └── model.config
    ├── run_gazebo_alti.sh              T1: launch Gazebo
    ├── run_sitl_alti.sh                T2: ArduPlane QuadPlane SITL (headless)
    ├── run_bridge_alti.sh             T3: ros_gz_bridge for the cameras
    ├── run_avoidance_alti.sh          T4: the mode-aware node
    ├── run_teleop.sh                   T5 (optional): keyboard teleop (hover only)
    ├── run_joy_alti.sh                 T5 (optional): RC radio -> GUIDED+avoidance (hover)
    ├── joy_to_cmdvel.py                RadioMaster joystick -> /cmd_vel_teleop (jsN direct)
    ├── run_rc_joystick_alti.sh         T5 (optional): RC radio -> native RC (manual flight)
    └── rc_joystick_alti.py             RadioMaster joystick -> RC_CHANNELS_OVERRIDE
```

The airframe is the stock **Alti Transition QuadPlane** from `SITL_Models`
(4 lift motors + a pusher, ailerons + elevator). `alti_four_cam` merges it and
adds the four camera links on `base_link`.

---

## 3. Run it in simulation (Gazebo + ArduPlane SITL)

Validated stack: **Ubuntu 22.04 + ROS 2 Humble + Gazebo Sim 8 (Harmonic) +
ArduPlane SITL + `ardupilot_gazebo` + `SITL_Models`**, NVIDIA GPU for perception.

### 3.1 Dependencies

Same as the copter build (see
[`quadcopter_obstacle_avoidance.md` §3.1](quadcopter_obstacle_avoidance.md)) —
ROS 2, `ros-gz`, `teleop_twist_keyboard`, `ultralytics transformers opencv-python
pymavlink`, and CUDA torch. **Plus** the model pack that carries the Alti
quadplane (already present in this environment at `~/ardu_ws/src/ardupilot_sitl_models`):

```bash
cd ~/ardu_ws/src
git clone https://github.com/ArduPilot/SITL_Models.git ardupilot_sitl_models   # if missing
```

`run_gazebo_alti.sh` puts `SITL_Models/Gazebo/{models,worlds}`, this repo's
`models/`, and the `ardupilot_gazebo` trees on `GZ_SIM_RESOURCE_PATH`, and the
`ArduPilotPlugin` `.so` on `GZ_SIM_SYSTEM_PLUGIN_PATH`.

### 3.2 Launch (4 terminals, in order)

ArduPilot lock-step: start Gazebo first, then SITL.

```bash
cd sim_vtol

# Terminal 1 — Gazebo with the Alti-quadplane obstacle world
./run_gazebo_alti.sh

# Terminal 2 — ArduPlane QuadPlane SITL (first run compiles ArduPlane, ~1–2 min)
./run_sitl_alti.sh

# Terminal 3 — bridge the 4 cameras (+ /clock) into ROS 2
./run_bridge_alti.sh

# Terminal 4 — mode-aware perception + safety + MAVLink (opens the grid window)
./run_avoidance_alti.sh

# Terminal 5 (optional) — hover-only manual flight. Pick ONE:
./run_teleop.sh      # keyboard
./run_joy_alti.sh    # RadioMaster GX12 / any EdgeTX radio (see §3.6)
```

On startup the node connects to SITL on `tcp:127.0.0.1:5760`, sets
`Q_GUIDED_MODE=1`, requests telemetry + `EXTENDED_SYS_STATE`, then GUIDED → arm →
**VTOL takeoff** to `TAKEOFF_ALT` (15 m).

To hand-fly the aircraft yourself — change modes, arm, take off, steer it around
and trigger the fixed-wing transition — attach an interactive **MAVProxy**
console to a spare SITL port; see [§3.5](#35-drive-it-yourself-from-a-mavproxy-terminal).

### 3.3 Demonstrating both regimes

- **VTOL hover avoidance** — right after takeoff the aircraft is in the
  multicopter state, so all four cameras are active and the grid is the familiar
  2×2. The world places low obstacles to the **left / right / back** of the spawn
  (clear of the forward transition path); strafe toward them with teleop and
  watch `STRAFE LEFT/RIGHT`, `STOP FWD/REV`, `BOXED IN`.
- **Fixed-wing cruise avoidance** — command a transition to forward flight
  (e.g. switch to `CRUISE`/`GUIDED` and fly down +x, or run an AUTO mission). Once
  the VTOL state flips to `FW`, the grid shows the **front camera large with the
  other three tiles greyed `OFF (FW cruise)`**. The world has tall **towers down
  the +x corridor** at cruise altitude; the node responds with `BANK LEFT/RIGHT`
  and `CLIMB + BANK …`.

### 3.4 Headless / detached & teardown

Identical to the copter build (see
[`quadcopter_obstacle_avoidance.md` §3.3–3.5](quadcopter_obstacle_avoidance.md)):
SITL runs `--no-mavproxy` and the node talks straight to `tcp:5760` (with
`request_data_stream`); launch each service with `setsid … </dev/null` on
`DISPLAY=:1`; verify VTOL state with a quick probe; read pose from
`gz topic -e -t /world/alti_world/dynamic_pose/info`. Teardown (bracketed
patterns):

```bash
pkill -9 -f "gz[ ]sim";     pkill -9 -f "sim_[v]ehicle"
pkill -9 -f "[a]rduplane";  pkill -9 -f "[p]arameter_bridge"
pkill -9 -f "vtol_obstacle_[a]voidance"
```

### 3.5 Drive it yourself from a MAVProxy terminal

`run_sitl_alti.sh` launches ArduPlane with **`--no-mavproxy`** (the interactive
MAVProxy console needs a real TTY/`DISPLAY` and dies under the detached
`setsid … </dev/null` launch, taking SITL down with it). The avoidance node then
talks straight to the autopilot on `tcp:127.0.0.1:5760`.

That does **not** mean MAVProxy is unavailable — ArduPlane SITL exposes several
MAVLink TCP endpoints, not just one:

| Port | Serial | Used by |
| ---- | ------ | ------- |
| `5760` | SERIAL0 | the avoidance node (`MAVLINK_URL` default) |
| `5762` | SERIAL1 | **free — attach MAVProxy here** |
| `5763` | SERIAL2 | free (second GCS / `mavlink` inspector) |

So you can run an interactive MAVProxy **alongside** everything else. In a fresh
terminal (a real one with keyboard focus — not a detached service):

```bash
mavproxy.py --master=tcp:127.0.0.1:5762 --console --map
```

You get the MAVProxy command line plus the console + map windows. Confirm the
link with `status` (you should see heartbeats and the current mode).

#### Quadplane flight modes

A QuadPlane has **VTOL (multicopter) modes** and **fixed-wing modes** — switch
with `mode <NAME>`:

| Regime | Modes | Notes |
| ------ | ----- | ----- |
| VTOL hover | `QSTABILIZE` `QHOVER` `QLOITER` `QLAND` `QRTL` | `QLOITER` = GPS position hold (easiest to fly by stick) |
| Fixed-wing | `FBWA` `CRUISE` `LOITER` `RTL` `MANUAL` | wing-borne; needs forward airspeed |
| Either | `GUIDED` `AUTO` | companion / mission control (the node uses `GUIDED`) |

#### Get airborne

```text
mode QLOITER
arm throttle
takeoff 15        # VTOL takeoff to 15 m (works in QLOITER / GUIDED)
```

> Heads-up: by default the avoidance node (Terminal 4) **auto-arms and
> auto-VTOL-takes-off** on startup. If it's running, the aircraft is already in
> the air — skip the arm/takeoff lines and just use MAVProxy to change modes and
> steer.

#### Drive it around in hover — two ways

**A. Stick it to a point (GUIDED + map).** The simplest "go there":

```text
mode GUIDED
```
…then **right-click the map → "Fly to"** at the spot you want — the quadplane
flies to that lat/lon and holds. `guided 25` (just an altitude) makes it
climb/descend to 25 m at the current position. This needs `Q_GUIDED_MODE = 1`
(the node already sets it; SITL params include it too).

**B. Fly it by stick (QLOITER + RC override).** In `QLOITER`/`QHOVER` you can
"push the sticks" from MAVProxy with `rc <chan> <pwm>` (1000–2000, 1500 =
centre). Channels: **1 = roll (strafe), 2 = pitch (fwd/back), 3 = throttle
(climb), 4 = yaw**:

```text
mode QLOITER
rc 2 1400      # nose-down → fly FORWARD   (lower = more forward)
rc 2 1500      # re-centre → stop / hold
rc 1 1600      # roll right → strafe RIGHT
rc 4 1600      # yaw right
rc 3 1600      # climb     (rc 3 1400 = descend; 1500 = hold)
rc all 1500    # recenter every stick (hover in place)
```

This drives the same holonomic hover the keyboard teleop (Terminal 5) exercises,
so you'll see the node's `STRAFE`, `STOP FWD/REV`, `BOXED IN` overrides react as
you push toward the side/back obstacles.

#### Trigger a fixed-wing transition (to see cruise avoidance)

```text
mode CRUISE        # or FBWA — commands forward, wing-borne flight
rc 3 1700          # add throttle so it accelerates and transitions
```

Once airspeed builds, `EXTENDED_SYS_STATE.vtol_state` flips to `FW`; the grid
greys out the side/back tiles and the node switches to **bank + climb** against
the towers down the +x corridor. `mode QRTL` brings it back to a VTOL landing.

#### Land

```text
mode QLAND         # vertical descent + disarm at touchdown
# or:  mode QRTL   # return to launch, then QLAND
```

#### One client owns the setpoints

MAVProxy (5762) and the avoidance node (5760) command the **same** vehicle. Mode
changes, arm/disarm, takeoff and map "Fly to" from MAVProxy are fine. But if you
*and* the node both stream `GUIDED` setpoints at once they'll fight and you'll get
jitter. In practice: let MAVProxy own mode/arm/takeoff and let the node own the
avoidance setpoints — or stop the node when you want to hand-fly.

#### Optional: make MAVProxy the single hub

If you'd rather route everything through one MAVProxy instance, start it on the
primary port with a UDP fan-out and point the node at that instead:

```bash
# Terminal 2b — start this BEFORE the avoidance node
mavproxy.py --master=tcp:127.0.0.1:5760 --out=udp:127.0.0.1:14551 --console --map

# Terminal 4 — node reads MAVLINK_URL from the env (no code change)
MAVLINK_URL=udp:127.0.0.1:14551 ./run_avoidance_alti.sh
```

Now MAVProxy owns the real link and the node shares it over UDP — handy when you
want one console that sees both your commands and the node's.

### 3.6 Fly it with a RadioMaster GX12 (RC radio) — with live avoidance

You can hand-fly the VTOL with a real transmitter while the 4-camera avoidance
stays active. The trick is that the avoidance node already subscribes to
`/cmd_vel_teleop`, maps it to a hover velocity, **and gates it through the STOP
filter before forwarding it to ArduPlane**. So the radio just becomes another
publisher of that topic — no change to the avoidance logic, and every
`STOP FWD/REV`, `STRAFE`, `BOXED IN` override still applies to your stick input.

```
 RadioMaster GX12 ──USB HID──▶ /dev/input/jsN ──▶ joy_to_cmdvel.py
                                                      │ /cmd_vel_teleop
                                                      ▼
        ArduPlane GUIDED ◀── avoidance node (4-cam STOP filter) ◀──┘
```

**Scope:** this drives the **VTOL hover** regime (holonomic: forward/back,
strafe, yaw, climb) — same as the keyboard teleop. In fixed-wing cruise the node
ignores teleop and only auto-dodges; switch regimes from MAVProxy (§3.5).

> **Why not `joy_node`?** `ros-humble-joy` is SDL2-based and reads the radio's
> **evdev** node (`/dev/input/eventN`). On this GX12 the evdev node does **not**
> emit axis events, so `joy_node`/`ros2 topic echo /joy` shows the sticks frozen
> at neutral even while they move. The legacy **joydev** node
> (`/dev/input/jsN`) *does* report them, so `joy_to_cmdvel.py` reads `jsN`
> directly — no `joy_node`, no `/joy` topic.

#### One-time radio setup

1. On the GX12 (EdgeTX), select **USB Joystick (HID)** mode — when you plug in
   USB, choose *"USB Joystick"* on the radio's popup (not *USB Storage* / *USB
   Serial*), and clear any *"Throttle Warning"* / alarm popup (EdgeTX halts
   output while a popup is up). Linux enumerates it as `/dev/input/js0`
   (`OpenTX Radiomaster GX12 Joystick`).
2. Verify it's seen **and sending data** — `js0` must exist *and* report moving
   axes:
   ```bash
   ls -l /dev/input/by-id/*Radiomaster*        # -> ../js0
   CALIBRATE=1 ./run_joy_alti.sh               # move a stick: values must change
   ```
   If the values stay frozen, the radio isn't transmitting (re-select USB
   Joystick mode / clear the popup / try a data-capable cable + port).

#### Launch

Bring up T1–T4 as usual (§3.2). Then **instead of** `run_teleop.sh`:

```bash
# Terminal 5 — RC radio teleop (don't also run run_teleop.sh)
./run_joy_alti.sh
```

`run_joy_alti.sh` just runs `joy_to_cmdvel.py`, which reads `/dev/input/jsN` and
publishes `/cmd_vel_teleop` at 50 Hz. The avoidance node (T4) auto-takes-off to
15 m and holds; then your sticks fly it.

#### Stick mapping (EdgeTX Mode-2 defaults)

| Stick | Axis | Twist | Effect |
| ----- | ---- | ----- | ------ |
| right ↕ | pitch (`a1`) | `linear.x` | fly **forward / back** |
| right ↔ | roll (`a0`) | `linear.y` | **strafe** left / right |
| left ↕ | throttle (`a2`) | `linear.z` | **climb / descend** |
| left ↔ | rudder (`a3`) | `angular.z` | **yaw** left / right |

The three gimbal axes (pitch/roll/yaw) self-centre → **sticks centred = hover
hold**. The throttle does *not* self-centre (it rests at an end), so vertical
control behaves like ALT_HOLD: **centre = hold, up = climb, down = descend**, and
it stays *inert until you move the throttle through centre once* (so it can't
lurch right after takeoff). Default speeds: 2.0 m/s horizontal, 1.0 m/s vertical,
0.6 rad/s yaw.

#### Calibrate / fix a reversed or wrong stick

Axis indices and directions can differ by EdgeTX channel-order settings. Inspect
the **raw `jsN` values** (this is the built-in calibrate mode — no ROS, no
`joy_node`), moving **one stick at a time**:

```bash
CALIBRATE=1 ./run_joy_alti.sh     # prints a0..a7 live; Ctrl-C to quit
```

Note, for each control, which `aN` moves and its sign at full throw. Set the
mapper's sign so the *intended* motion comes out positive:

- **pitch** forward should give `linear.x > 0`,
- **roll** right should give `linear.y < 0` (node turns this into strafe-right),
- **throttle** up should give `linear.z > 0`,
- **rudder** right should give `angular.z < 0` (node turns this into yaw-right).

Then export any overrides before launching (unset ones keep the defaults):

```bash
# example: throttle turned out to be axis 3, and pitch was reversed
AX_THR=3 SIGN_PITCH=1 ./run_joy_alti.sh
```

| Env var | Default | Meaning |
| ------- | ------- | ------- |
| `AX_PITCH` `AX_ROLL` `AX_THR` `AX_YAW` | `1 0 2 3` | `jsN` axis index per control |
| `SIGN_PITCH` `SIGN_ROLL` `SIGN_THR` `SIGN_YAW` | `-1 -1 1 -1` | flip a reversed direction (`±1`) |
| `SPEED_XY` `SPEED_Z` `YAW_RATE` | `2.0 1.0 0.6` | max stick speed (m/s, m/s, rad/s) |
| `DEADZONE` | `0.08` | stick noise band around centre |
| `ENABLE_BUTTON` | `-1` | joystick button that must be on to command; `-1` = always on. Set to a GX12 switch for a **deadman/override** gate |
| `CALIBRATE` | `0` | `1` = print live `aN`/button values and exit |
| `VERBOSE` | `0` | `1` logs the live Twist at ~2 Hz |
| `JS_DEV` | `0` | `/dev/input/jsN` index |

> **Tip — a kill switch.** Find a GX12 toggle's button index with
> `CALIBRATE=1 ./run_joy_alti.sh` (it prints `btns:<n>` when a switch is on), then
> set `ENABLE_BUTTON=<n>` so the radio only commands while that switch is on; flip
> it off and the node instantly reverts to hover-hold. Recommended before flying
> the obstacle course aggressively.

#### What you should see

Fly toward the side/back obstacles in hover: as a zone goes `STOP`, the node
clamps your stick input — pushing into a blocked side yields `STRAFE`/`STOP
FWD/REV`, and a wall on both sides shows `BOXED IN` (pitch-gated). The red
`AVOID:` banner on the grid window lights up exactly as with keyboard teleop, but
now it's reacting to *your* radio inputs.

### 3.7 Fly it as a normal RC transmitter (arm with the sticks, manual modes)

§3.6 flies through the **companion/GUIDED** path: the radio is an *intent* the
avoidance node filters, and the node arms + takes off for you. If instead you
want to fly the QuadPlane **like a real ArduPilot RC setup** — arm with the stick
gesture, hand-fly `QSTABILIZE` / `QLOITER` / `QHOVER` / `FBWA` — you need the
radio injected into the autopilot's **RC channels**, which the §3.6 bridge does
*not* do (it publishes a ROS topic, not RC). That's what `rc_joystick_alti.py` is
for: it reads `/dev/input/jsN` directly and streams `RC_CHANNELS_OVERRIDE` into
SITL.

```
 RadioMaster GX12 ──USB HID──▶ /dev/input/jsN ──▶ rc_joystick_alti.py
                                                      │ RC_CHANNELS_OVERRIDE
                                                      ▼  (tcp:5762)
                                                ArduPlane SITL  (ch1-4 = RC)
```

> **Not MAVProxy's `joystick` module.** It's SDL2/pygame-based and reads the
> evdev node — same blind spot as `joy_node` on this GX12. Reading `jsN` and
> sending the MAVLink ourselves is what works.

**Port plan** (SITL exposes three endpoints — §3.5): `5760` = avoidance node,
**`5762` = this RC bridge**, `5763` = MAVProxy (for mode changes).

#### Launch — strict order, with a wait-gate at each step

Order matters. Each stage must be **ready** before the next or you hit the
classic failures (SITL hangs half-booted, or the RC override gets scrambled).
Bring up **T1–T2 only**; **skip T4** (its auto-takeoff fights manual flight).

1. **Terminal 1 — Gazebo.** Wait until its ArduPilot plugin is listening:
   ```bash
   ./run_gazebo_alti.sh
   ss -ltn | grep 9002              # GATE: must show LISTEN before starting SITL
   ```
2. **Terminal 2 — SITL.** Wait until all three MAVLink ports are up (~30 s):
   ```bash
   ./run_sitl_alti.sh
   ss -ltn | grep -E ':576[0-3]'    # GATE: must show 5760, 5762 AND 5763
   ```
3. **Terminal 3 — RC bridge** (port `5762`):
   ```bash
   VERBOSE=1 ./run_rc_joystick_alti.sh   # GATE: must print "[rc] autopilot sys=1"
   ```
4. **Terminal 4 — MAVProxy on `5763`** (⚠️ never the bridge's `5762`):
   ```bash
   mavproxy.py --master=tcp:127.0.0.1:5763 --console
   ```

The bridge sets `ARMING_RUDDER=2` so the stick gesture arms.

#### Fly

```text
mode QLOITER                 # in MAVProxy (friendliest: GPS + altitude hold)
```
- throttle stick **all the way down**, then `arm throttle`
  (or the stick gesture: **throttle down + rudder right**, hold ~2 s),
- **raise the throttle** → it lifts off; pitch / roll / yaw fly it,
- to land: throttle down until it settles, then `disarm` in MAVProxy.

`QSTABILIZE` is bare manual stabilize (throttle = direct thrust); `QLOITER` /
`QHOVER` hold position / altitude.

#### Stick directions — verified for this GX12 (baked in as defaults)

| Stick | RC ch | Sign | Behaviour |
| ----- | ----- | ---- | --------- |
| throttle up / down | ch3 | `SIGN_THR=+1` | up = climb, **down ≈ 1000 (armable)** |
| pitch fwd / back | ch2 | `SIGN_PITCH=-1` | forward = move forward |
| yaw left / right | ch4 | `SIGN_YAW=-1` | right = nose right |
| roll left / right | ch1 | `SIGN_ROLL=+1` | right = strafe right |

`./run_rc_joystick_alti.sh` now flies correctly with no extra flags. On a
different radio, check directions with `VERBOSE=1` (prints the PWMs) or
`status RC_CHANNELS` in MAVProxy (throttle down must read ~1000), and flip any
reversed axis, e.g. `SIGN_ROLL=-1 ./run_rc_joystick_alti.sh`. Env vars:
`AX_PITCH/AX_ROLL/AX_THR/AX_YAW` (default `1 0 2 3`),
`SIGN_PITCH/SIGN_ROLL/SIGN_THR/SIGN_YAW` (default `-1 +1 +1 -1`),
`DEADZONE` (`0.05`), `RATE_HZ` (`50`), `JS_DEV` (`0`), `MAVLINK_URL`
(`tcp:127.0.0.1:5762`).

#### Gotchas (all handled by the scripts — kept here so they stay fixed)

| Symptom | Cause | Resolution |
| ------- | ----- | ---------- |
| sticks move in `CALIBRATE` but `/joy` stays frozen | `joy_node`/SDL reads the evdev node this GX12 doesn't populate | the bridge reads `/dev/input/jsN` directly |
| override ignored; bridge prints `heartbeat sys=0` | messages addressed to MAVLink system 0 | bridge finds the autopilot's real `sys=1` |
| "RC not reaching" / scrambled `chan*_raw` | bridge **and** MAVProxy on the same port `5762` | bridge on `5762`, MAVProxy on **`5763`** |
| SITL stuck, only `5760` opens, no `9002` link | SITL started before Gazebo's plugin was up | wait for `9002` LISTEN before launching SITL |
| won't arm, throttle "high" at idle | throttle axis inverted | `SIGN_THR` (now `+1` for this GX12) |

#### Keeping obstacle avoidance — the GUARDIAN (avoidance *in* QLOITER)

GUIDED setpoints are ignored in pilot-flown modes, and **ArduPlane has no
built-in proximity avoidance** (no `AVOID_*`/`PRX_*` params, unlike Copter), so
the companion has to intervene itself. The avoidance node's **GUARDIAN** does
exactly that while you hand-fly `QLOITER`/`QHOVER`:

1. Bring up T1–T3 as above, then start the avoidance node **already-airborne**
   (or before takeoff — the guardian waits until you're armed and above ~3 m):
   ```bash
   SKIP_TAKEOFF=1 ./run_avoidance_alti.sh      # T4, connects on tcp:5760
   ```
2. Arm and fly in `QLOITER` with the radio as usual. While every camera is
   CLEAR the node only watches — the RC flies the aircraft natively.
3. When any camera zone goes **STOP**, the node switches to `GUIDED` and the
   grid status line shows **`GUARDIAN`**. Your sticks **stay live**: the node
   reads them back via `RC_CHANNELS` and forwards them as streamed body
   **position-offset** targets (ArduPlane GUIDED ignores velocity setpoints)
   through the same STOP gate (forward blocked → forward stick ignored; side
   blocked → auto strafe away).
4. After ~1.5 s of all-clear it hands `QLOITER` back automatically. Flipping
   the mode switch yourself at any point makes the guardian stand down.

Tunables (env on T4): `GUARDIAN=0` disables it; `RC_SPEED_XY` / `RC_SPEED_Z` /
`RC_YAW_RATE` scale the gated stick response, `GUARD_CLEAR_S` / `GUARD_MIN_ALT`
the hand-back hysteresis and engage floor (constants in the script header).
The old manual hand-off (`mode GUIDED` in MAVProxy) still works too.

---

## 4. Deploy on a real VTOL

Like the copter, the control path is **already hardware-shaped**: the node speaks
MAVLink to an autopilot, so you point `MAVLINK_URL` at a real ArduPlane
flight controller (Pixhawk/Cube running QuadPlane firmware) instead of SITL.

```
        ┌──────────────────────────────┐        ┌──────────────────────────┐
 4 cams │ Companion (Jetson / RPi+Hailo)│ MAVLink│ Flight controller        │
 ──────►│  vtol_obstacle_avoidance.py   │───────►│  ArduPlane QuadPlane      │
        │  perception + mode switch     │ serial │  (lift motors + pusher)   │
        └──────────────────────────────┘ or UDP └──────────────────────────┘
```

Key differences from the multirotor deployment:

1. **The mode switch is the whole point.** The node already derives it from
   `EXTENDED_SYS_STATE`, which any ArduPlane QuadPlane sends — no extra wiring.
   In cruise it runs **one** camera, so a modest companion can keep up at speed.
2. **Camera coverage by regime.** Mount the front camera with a clear, vibration-
   isolated forward view (it's the only one used at cruise). The side/back cameras
   only matter in hover/transition — fine to use cheaper/wider modules there.
3. **Cruise needs a longer-range, metric forward sensor.** At wing speed a
   monocular relative-depth STOP gives very little time/space to turn. Pair the
   front camera with forward radar or a long-range lidar and OR a metric range
   into the front STOP; vision then classifies *what* while the metric sensor
   gives *how far / how long until impact*.
4. **GUIDED behaviour / params.** Set `Q_GUIDED_MODE = 1` so VTOL GUIDED accepts
   velocity setpoints. Decide your cruise dodge policy: the node takes over
   GUIDED to bank/climb — make sure your mission/GCS expects the companion to do
   that, and keep an RC mode switch that instantly reclaims control.
5. **Companion compute** — same table as the copter
   ([`quadcopter_obstacle_avoidance.md` §4.4](quadcopter_obstacle_avoidance.md));
   Jetson runs this torch+CUDA pipeline unchanged. In cruise you're running a
   single camera, so even edge NPUs have plenty of headroom.

**Do not auto-arm / auto-VTOL-takeoff in the field** (the script does it for the
sim only). Let the pilot take off, then enable companion avoidance.

---

## 5. Tuning & troubleshooting

Perception knobs (`CLOSE_THRESH`, `WARN_FRAC`, `STOP_FRAC`, `N_ZONES`,
`DEPTH_INPUT`) are shared — see
[`car_obstacle_avoidance.md` §5](car_obstacle_avoidance.md).

VTOL-specific knobs (top of `vtol_obstacle_avoidance.py`):

| Knob | Default | Effect |
| ---- | ------- | ------ |
| `TAKEOFF_ALT` | 15 m | VTOL takeoff altitude (env `TAKEOFF_ALT`) |
| `STRAFE_SPEED` | 1.5 m/s | hover sideways dodge speed |
| `YAW_AWAY` | 0.4 rad/s | hover yaw-away rate |
| `SETPOINT_HZ` | 10 Hz | hover velocity setpoint stream rate (≥ ~3 Hz) |
| `FW_LOOKAHEAD` | 80 m | how far ahead the cruise dodge waypoint is placed |
| `FW_DODGE_Y` | 35 m | lateral offset of the cruise dodge (turn radius vs. reaction) |
| `FW_CLIMB` | 25 m | climb added when something is dead ahead |

| Symptom | Fix |
| ------- | --- |
| Grid never switches to `FW/CRUISE` | `EXTENDED_SYS_STATE` not arriving — the node requests it via `SET_MESSAGE_INTERVAL`; confirm the link and that the vehicle actually transitioned (it stays `MC` while hovering) |
| Hover velocity commands ignored | `Q_GUIDED_MODE` not set / not in a GUIDED-VTOL state — the node sets the param on connect; verify it took |
| `SIM_VEHICLE: MAVProxy exited`, SITL dies | use the headless `--no-mavproxy` path; the node talks to `tcp:5760` directly |
| Model won't spawn / black tiles | check `SITL_Models` is on `GZ_SIM_RESOURCE_PATH` (base `alti_transition_quad`) and that the world includes `gz-sim-sensors-system` (ogre2) for the cameras |
| Won't transition to forward flight | quadplanes need airspeed/throttle to transition — give it room and a forward command (CRUISE/AUTO); tune `Q_*` transition params if needed |
| Cruise dodge too sharp / too late | tune `FW_DODGE_Y` (lateral) and `FW_LOOKAHEAD` for your airframe's turn radius and cruise speed |

> First bring-up note: getting a quadplane to transition and cruise cleanly in
> SITL takes some tuning of the ArduPlane `Q_*`/transition params — the
> perception + mode-switch + decision logic is independent of that and can be
> validated in hover first.

---

## 6. Safety notes

- **Validate in SITL first**, then bench-test with props removed before flight.
- **Keep an RC e-stop.** The node takes over GUIDED to dodge in cruise; a mode
  switch back to a manual/RC mode must instantly override it.
- **Cruise avoidance is geometry-limited.** A wing has a turn radius and can't
  stop — `FW_LOOKAHEAD`/`FW_DODGE_Y` must be sized for your speed, and you should
  rely on a metric forward sensor, not relative depth, for the real trigger (§4).
- **Transitions are the riskiest phase.** Near the ground during VTOL↔FW
  transition the vehicle is neither fully hovering nor flying; treat
  `TRANSITION_*` states conservatively (the node classifies them with their
  destination regime).
- **GUIDED needs a position estimate** and a streaming link; configure ArduPlane
  geofence, RTL and battery failsafes independently of this node.
- **No auto-arm / auto-takeoff in the field** — sim convenience only.
```
