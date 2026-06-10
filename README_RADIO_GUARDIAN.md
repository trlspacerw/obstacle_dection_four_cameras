# VTOL Sim — Radio Flight + Obstacle-Avoidance GUARDIAN

**Date:** 2026-06-10
**What this covers:** everything changed/validated in this session — flying the
Alti quadplane by RadioMaster GX12 in `QLOITER` **with obstacle avoidance
active** (the GUARDIAN), the radio axis calibration for *this exact* GX12, and
the failures we hit on the way (so you never debug them twice).

Main project README: `../vtol_obstacle_avoidance.md` (§3.7 has the GUARDIAN
section).

---

## 1. The problem this solves

The avoidance node (`vtol_obstacle_avoidance.py`) commands the aircraft with
**GUIDED** setpoints. ArduPilot ignores those in pilot-flown modes, so when you
hand-fly `QLOITER` with the radio, avoidance did nothing. And ArduPlane has
**no built-in proximity avoidance** to fall back on — the full parameter dump
(`mav.parm`) contains zero `AVOID_*` / `PRX_*` / `OA_*` parameters (unlike
ArduCopter), so feeding the FC obstacle distances would not help either.

## 2. What was added/changed

### 2.1 GUARDIAN (in `vtol_obstacle_avoidance.py`)

While you hand-fly `QLOITER`/`QHOVER` (armed, above 3 m), the node watches the
4-camera STOP flags. When any zone goes **STOP** it:

1. switches itself to `GUIDED` (grid status line shows **`GUARDIAN`**),
2. keeps your sticks live — it reads them back from the FC via `RC_CHANNELS`
   and forwards them through the same STOP gate as **`BODY_OFFSET_NED`
   position-offset targets** (ArduPlane GUIDED ignores *velocity* setpoints —
   validated 06-08). Forward-into-the-obstacle is ignored; a blocked side
   triggers an automatic strafe away,
3. hands your mode back after **1.5 s** of all-clear. Flipping your mode
   switch (or `mode QLOITER` in MAVProxy) makes it stand down instantly.

Tunables (env vars on the avoidance terminal):

| Env | Default | Meaning |
|---|---|---|
| `GUARDIAN` | `1` | `0` disables the guardian entirely |
| `GUARD_CLEAR_S` | `1.5` | seconds of all-clear before handing the mode back |
| `GUARD_MIN_ALT` | `3.0` | m rel-alt below which it never engages (landings) |
| `RC_SPEED_XY` | `2.0` | m/s at full stick during an intervention |
| `RC_SPEED_Z` | `1.0` | m/s climb at full throttle stick |
| `RC_YAW_RATE` | `0.6` | rad/s at full rudder stick |

### 2.2 Robustness fixes

- `vtol_obstacle_avoidance.py`: the MAVLink reader (`_drain`) now runs inside
  the try-block (one bad packet no longer kills the setpoint thread) and
  drains up to 500 msgs/cycle so it recovers from backlogs.
- `rc_joystick_alti.py`: now **drains inbound MAVLink** every loop (it never
  read its socket before, ballooning the FC's TCP send buffer), and **exits
  cleanly when the radio is unplugged** (`OSError: ENODEV`) instead of
  streaming frozen stick values forever.

### 2.3 Radio calibration (baked into `run_rc_joystick_alti.sh`)

See §4 — the roll/yaw axis indices on this GX12 are swapped vs the script
defaults.

---

## 3. How to run it (radio + avoidance), in order

```bash
cd ~/obstacle_dection_four_cameras/sim_vtol

# T1 Gazebo — wait until UDP 9002 is bound:
./run_gazebo_alti.sh                 #  ss -lun | grep 9002
# T2 ArduPlane SITL — wait for ALL of 5760/5762/5763 (~1 min):
./run_sitl_alti.sh                   #  ss -ltn | grep -E ':576[023]'
# T3 camera bridge:
./run_bridge_alti.sh
# T4 avoidance node + GUARDIAN (no auto-takeoff — you fly):
SKIP_TAKEOFF=1 ./run_avoidance_alti.sh
# T5 RC radio bridge (GX12 plugged in, "USB Joystick (HID)" mode):
./run_rc_joystick_alti.sh
# T6 (optional) MAVProxy console — ONLY ever on 5763:
mavproxy.py --master=tcp:127.0.0.1:5763 --console
```

**Port plan:** `5760` avoidance node · `5762` RC bridge · `5763` MAVProxy.
One client per port — SITL serves only the **first** connection.

**Fly:** `mode QLOITER` (MAVProxy) → throttle stick fully **down** → hold
**throttle down + rudder right ~2 s** → armed → rudder back to center → raise
throttle past mid to lift off. Center throttle ≈ hold altitude. Land: throttle
low until it settles, then **throttle down + rudder left** to disarm.

---

## 4. This GX12's exact axis mapping (and how to change it)

Calibrated live 2026-06-10 with labeled stick-holds. **The defaults in
`rc_joystick_alti.py` assume yaw=ax3 / roll=ax0 — on this radio they are
swapped**, and both horizontal axes read `right = +1`:

| Physical stick | joydev axis | FC channel | env override in effect |
|---|---|---|---|
| **left horizontal** (rudder) | `axis 0` | ch4 yaw | `AX_YAW=0`, `SIGN_YAW=+1` |
| **right vertical** (elevator) | `axis 1` | ch2 pitch | default (`AX_PITCH=1`, `SIGN_PITCH=-1`) |
| **left vertical** (throttle, *non-centering*) | `axis 2` | ch3 throttle | default (`AX_THR=2`, `SIGN_THR=+1`) |
| **right horizontal** (aileron) | `axis 3` | ch1 roll | `AX_ROLL=3`, `SIGN_ROLL=+1` |

These are **persisted as defaults in `run_rc_joystick_alti.sh`** — you don't
need to pass anything; `./run_rc_joystick_alti.sh` just works.

### Changing the mapping (new radio / EdgeTX model change / Mode 1↔2)

Every assignment is an env var — no code edits needed:

```bash
# which joydev axis index feeds which channel:
AX_ROLL=3 AX_PITCH=1 AX_THR=2 AX_YAW=0 \
# +1 or -1 to flip a direction:
SIGN_ROLL=+1 SIGN_PITCH=-1 SIGN_THR=+1 SIGN_YAW=+1 \
./run_rc_joystick_alti.sh
```

Conventions the FC expects (what "correct" looks like):

| Channel | stick action | PWM |
|---|---|---|
| ch3 throttle | stick **down** | **~1000** (required for arming!) |
| ch4 yaw | stick **right** | ~2000 |
| ch1 roll | stick **right** | ~2000 |
| ch2 pitch | stick **forward** | ~1000 (low = nose forward) |

To make a change permanent, edit the `export AX_*` / `export SIGN_*` block
near the bottom of `run_rc_joystick_alti.sh`.

### Recalibrating from scratch (do it this way — it avoids the traps)

1. Watch the live PWMs: `VERBOSE=1 PYTHONUNBUFFERED=1 ./run_rc_joystick_alti.sh`
   (or raw axes with `jstest /dev/input/js0` if installed).
2. Move **one stick, one direction at a time**, and **hold ≥1 s** — read off
   which axis moves and its sign. Ignore transients while the stick travels.
3. **Traps that cost us an hour:**
   - The **throttle does not spring back** — it *parks*. When a program opens
     the joydev node, the kernel replays the parked position as an INIT event
     that looks exactly like a live stick move. Ignore the first burst.
   - Unplug/replug the radio ⇒ pick **"USB Joystick (HID)"** on its popup
     (never USB Storage), then **restart the bridge** — and check whether the
     device came back as `/dev/input/js1` (`JS_DEV=1 ./run_rc_joystick_alti.sh`).

---

## 5. Failures hit this session (symptoms → fix)

| Symptom | Root cause | Fix |
|---|---|---|
| MAVProxy shows no GPS, `Gyro 0 rate 0Hz`, won't arm | a **second** `run_sitl` launch hijacked the Gazebo FDM handshake then died — original SITL left with zero sensor data (Gazebo keeps stepping at RTF≈1, so it *looks* fine) | kill all duplicates, restart SITL only (Gazebo/bridge can stay). **Never launch any `run_*.sh` twice** |
| avoidance node silently stops reacting | a duplicate node jammed tcp:5760 (one client per port); reader thread died on a bad packet | duplicates killed; reader hardened (survives parse errors, drains backlog) |
| sticks "do nothing", all channels 1500 | radio dropped out of USB-HID mode after sleep/replug | replug, choose "USB Joystick (HID)", restart bridge |
| arm gesture ignored | rudder was landing on the wrong FC channel (axis indices swapped, §4) | calibrated mapping baked into the run script |
| `PreArm: AHRS: not using configured AHRS type` right after boot | EKF3 still settling | wait ~20 s after GPS fix; clears itself |

## 6. Where things are

| File | What |
|---|---|
| `sim_vtol/README_RADIO_GUARDIAN.md` | **this file** |
| `sim_vtol/vtol_obstacle_avoidance.py` | avoidance node + GUARDIAN |
| `sim_vtol/rc_joystick_alti.py` | radio → RC_CHANNELS_OVERRIDE bridge |
| `sim_vtol/run_rc_joystick_alti.sh` | launch + **persisted GX12 axis mapping** |
| `../vtol_obstacle_avoidance.md` §3.7 | GUARDIAN docs in the main README |

**Status:** sim validated end-to-end through arming + radio flight in QLOITER.
The GUARDIAN's in-flight intervention (auto-brake/strafe on a STOP zone) is
implemented but still awaiting its first live obstacle pass.
