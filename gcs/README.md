# VTOL Ground Station — fly the simulated drone from your browser

A browser cockpit for the MuJoCo simulation: practise flying with RC-style sticks (on-screen,
keyboard, or your real RC transmitter over USB), or give it a destination and watch it fly the
delivery on its own.

## Run it

Inside WSL (conda env `drone311`), from the project folder:

```bash
bash scripts/run_gcs.sh
```

Then open **http://localhost:8765** in any browser on Windows. Stop with Ctrl+C.
Options: `--policy runs/s2_waypoint/final.onnx` (choose the RL policy), `--fps 20`, `--port 8765`.
Without `--policy` it uses the newest trained policy it finds in `runs/` (ONNX preferred).

## Controls (Mode 2, like a real transmitter)

| Stick / switch | MANUAL · POSITION | MANUAL · ANGLE | Wing-borne (SD = WING) |
|---|---|---|---|
| Left stick ↕ | climb / descend (centre = hold height) | collective throttle (does not spring back) | climb / descend |
| Left stick ↔ | yaw rate | yaw rate | — |
| Right stick ↕ | fly forward / back (≤ 4 m/s) | pitch angle (±30°) | pull back = climb |
| Right stick ↔ | fly left / right | roll angle (±30°) | turn (heading rate) |
| **SA** | arm / disarm (throttle must be centred or low) | arm (throttle must be **low**) | |
| **SB** | MANUAL / AUTO | | |
| **SC** | POSITION (assisted) / ANGLE (stabilised) | | |
| **SD** | HOVER / WING — flies the scripted transition (above 8 m) and the back-transition | | |
| **SE** | RL assist on/off (residual policy on top of the baseline in hover flight) | | |
| **DROP** | release the payload | | |

Keyboard: **W/S** throttle · **A/D** yaw · **arrows** pitch/roll · **T** arm · **M** manual/auto ·
**V** hover/wing · **P** drop.

**Real transmitter:** EdgeTX/OpenTX radios (RadioMaster, Jumper, FrSky) and ELRS radios appear as a USB
joystick. Plug in, move a stick, open *Gamepad / RC transmitter* under the switches and assign each
channel to an axis (tick *inv* if it moves the wrong way). The mapping is remembered in the browser.

**Learning path:** start in POSITION with RL assist on (hard to crash), then turn RL assist off,
then try ANGLE mode — you control the height yourself, like a real stabilised quad. In ANGLE mode
mid-stick throttle is roughly hover. Crashes (hard landing > 4 m/s or flipping near the ground)
are detected; press **RESET SIM**. Wind (2–4 m/s with gusts) is selectable at reset.

## AUTO delivery

Type East/North coordinates (metres from home) or click the map, then **Start delivery**:

1. **CLIMB** to 15 m holding heading (yawing near the ground flips a tilt-rotor — yaw is
   differential nacelle tilt and saturates at ≈ ±7°).
2. **ALIGN** — turn slowly (10°/s) to face the destination, **SETTLE** 3 s.
3. **TRANSITION → CRUISE → BACK** — the validated baseline controller flies the conversion to
   wing-borne flight, cruises at 17 m/s and back-transitions 220 m before the pad.
4. **APPROACH** — hover-regime flight to the pad, nose-first, with the **RL residual policy active**.
5. **DESCEND → ON_PAD** — land, **release the payload** (the vehicle's mass, CG and inertia
   change in the physics), wait, and fly leg 2 home the same way.

Destinations closer than 300 m are flown as a multicopter (no transition). Measured, calm air:
427 m round trip in 217 s, landings 0.04–0.24 m from the pad centres, 22.7 % battery
(`tests/test_gcs.py` runs a full round trip).

## How it works

- `server.py` steps the real physics (`sim/vehicle.py`, 500 Hz) and controller (100 Hz) at real
  time, renders a chase/nose/orbit/top camera with MuJoCo offscreen (EGL), and streams JPEG frames
  plus 10 Hz telemetry over one websocket; the browser sends stick positions at 30 Hz.
  The video frame rate adapts so the physics always stays real-time.
- `sim_core.py` is headless and testable. The baseline transition logic was written for flight
  along world +x, so each fixed-wing leg runs it in a rotated **leg frame**: the controller sees
  the state rotated by −ψ; its outputs are body-frame actuator commands and need no rotation.
- The RL policy is used where it was trained — hover-regime flight (manual POSITION assist and
  the AUTO approach). Its observation is built in the current heading frame because training
  always started at heading 0 (hover physics is yaw-invariant apart from wind).
- Two physics bugs were found by this tool and fixed with tests: a 0/0 NaN in the slipstream
  model with motors stopped at rest, and MuJoCo's cached subtree mass not being updated when the
  payload is released (which put the computed CG tens of metres away).

Limitations: the RL transition stage was scoped out, so takeoff, transition, cruise and landing
are always flown by the baseline; the ground station is single-pilot (all connected browsers see
and control the same aircraft).
