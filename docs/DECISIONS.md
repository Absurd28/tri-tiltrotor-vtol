# DECISIONS.md

Design and engineering decisions for the Tilt-Tri-Rotor VTOL drone project.
Each entry explains the choice, the alternatives considered, and the reason — for viva preparation.

---

## D-00: Use WSL2 (Linux) instead of Windows natively

**Decision:** Run all simulation, training and tooling inside WSL2 Ubuntu 26.04.

**Reason:** Windows Smart App Control (SAC) is enabled on the development machine at enforcement
level. SAC blocks unsigned native DLLs/PYDs loaded at runtime, which prevents MuJoCo, NumPy,
SciPy and all other PyPI scientific packages from importing. SAC cannot be bypassed with
`Unblock-File`; turning it off is irreversible without reinstalling Windows.

**Alternatives:**
- Disable SAC → irreversible, security regression.
- Use conda-forge packages → conda packages are not Windows-signed either; same block.
- Docker → requires Docker Desktop which needs admin and was listed as ask-first.

**Consequence:** All `python` commands must be run inside the WSL2 session. The project files
live on the Windows filesystem (`/mnt/c/...`), accessible from both sides.

---

## D-01: Python 3.11 via Miniconda3 (conda-forge)

**Decision:** Use Python 3.11.18 installed via `conda create -c conda-forge python=3.11`.

**Reason:** Ubuntu 26.04 ships Python 3.14, which is too new for pre-built wheels from MuJoCo,
Stable-Baselines3 and ONNX Runtime. `apt install python3.11` requires sudo (interactive
password prompt, not scriptable). Miniconda installs to home dir with no sudo. Python 3.11
has the widest pre-built-wheel support for the entire approved stack as of 2025-11.

**Alternatives:**
- pyenv → valid, but requires build-deps (gcc, libssl etc.) and compilation time.
- Python 3.12 → also feasible; 3.11 chosen for SB3 compatibility certainty.

---

## D-02: ENU world frame, FLU body frame

**Decision:** World frame East-North-Up (z-up), body frame Forward-Left-Up (x=forward, y=left,
z=up). This follows the convention stated in the project brief.

**Reason:** ENU with z-up is natural for ground-based simulation (MuJoCo defaults to z-up).
FLU matches aerospace convention for multi-rotor vehicles and is compatible with PX4/ArduPilot.
Roll right → negative roll about x-axis in this convention.

**Alternative:** NED (North-East-Down) used in aviation autopilots → requires sign flips in the
control code; deferred to Phase 8 when autopilot integration is decided.

---

## D-03: 9-actuator model (3 throttle, 3 tilt, 3 virtual surface)

**Decision:** The actuator space has 9 dimensions: left throttle, right throttle, tail throttle,
left tilt, right tilt, tail tilt, aileron (virtual, symmetric opposite ailerons), elevator
(virtual, symmetric elevators), rudder (virtual, single rudder or differential).

**Reason:** This is the minimum set that captures all degrees of freedom in both hover and
cruise. Redundancy between tail-tilt and throttles for pitch is handled by the mixer, not by
adding extra dimensions.

---

## D-04: Counter-rotating wing motors (clockwise and counter-clockwise)

**Decision:** Left wing motor rotates CCW (when viewed from above), right wing motor CW.

**Reason:** Opposite reaction torques cancel in symmetric hover. The tail motor (small, often
off in cruise) is not counter-rotated — its residual 0.17–0.23 N·m yaw torque is handled by
differential wing tilt (small opposite tilts that create a net yaw moment without changing
net thrust direction appreciably at small angles).

---

## D-05: Physics step 2 ms, control at 100 Hz

**Decision:** MuJoCo integrator runs at 500 Hz (dt=0.002 s); baseline controller runs at 100 Hz;
RL policy runs at 50 Hz.

**Reason:** 500 Hz is sufficient to capture motor dynamics (time constant ~70 ms) with ≥35
steps per time constant. 100 Hz matches typical flight-controller update rates. RL at 50 Hz
reduces compute load while still being faster than the drone's attitude dynamics bandwidth
(~10–20 Hz).

---

## D-06: Six surface actuators in MJCF, three virtual surfaces in software

**Decision:** The MJCF has 6 position actuators (aileron L/R, elevator L/R, rudder L/R).
The control layer exposes 3 virtual commands (aileron, elevator, rudder) and maps them
to the 6 servos (ailerons opposite-sign, elevator/rudder pairs same-sign).

**Reason:** MuJoCo tendon-coupled actuators would leave the differential mode of each
surface pair uncontrolled (floating) unless extra joint springs are added. Separate
actuators are simpler, and the virtual mapping lives in the mixer where it belongs.
The 9-dim action space from the brief (3 throttle + 3 tilt + 3 surfaces) is assembled
at the control/RL layer.

---

## D-07: Elevator and rudder servos inside the rear fuselage (pushrod drive)

**Decision:** The 4 tail-surface servos (48 g) sit at x = -0.18 m in the fuselage and
drive the surfaces through pushrods, instead of sitting at the surfaces (x ≈ -0.55).

**Reason:** CG balance. With servos at the tail, the battery balance point moves to
x = 0.18 m — far outside the 0.06–0.10 m battery bay. Pushrod-driven tail surfaces are
standard practice in RC aircraft for exactly this reason. Also reduces Iyy by ~0.013 kg·m².

---

## D-08: Avionics split: FC+IMU at CG, remaining electronics in the nose

**Decision:** 50 g (flight controller + IMU) at the CG; 220 g (companion computer, GPS,
sensors, wiring) in a nose bay at x = 0.325 m.

**Reason:** The tail group (motor+prop+axle+linkage ≈ 0.28 kg at x ≈ -0.4 m) needs forward
mass to balance. The battery alone cannot do it within its 6–10 cm bay. The IMU stays at
the CG where it measures cleanly. Nose bay x = 0.325 was tuned so Iyy stays within 10%
of budget (at x = 0.40 it was +13.9%).

---

## D-09: Nose skid added

**Decision:** A small third skid under the nose (10 g), in addition to the two boom skids.

**Reason:** The boom skids sit entirely behind the CG (booms span x -0.15 to -0.62);
on the ground the drone would tip nose-down. Three-point stance is statically stable.

---

## D-10: MJCF primitives instead of CadQuery meshes (Phase 1)

**Decision:** The model is built from boxes, cylinders and capsules generated
parametrically from params.yaml, not STL meshes.

**Reason:** Primitives are parametric (one source of truth in params.yaml), compile
instantly, give exact analytic inertia, and make clearance/collision checks fast and
robust. Meshes add visual fidelity only; aerodynamics are analytic anyway. CadQuery
meshes remain an option for the report/presentation later.

---

## D-11: Notch front wall position fixed from the spec

**Decision:** The bulkhead (notch front wall) is placed so its notch-facing surface is at
x = -0.21 m, exactly 0.19 m ahead of the tail hub (-0.40 m).

**Reason:** First placement had the box centred at -0.222 m, putting the wall face 2.4 cm
too close to the prop — the tilt-sweep clearance check caught it (0.26 cm min clearance vs
1.52 cm target; now 2.66 cm). Good example of why the acceptance checks exist.

---

## D-12: Transition control architecture (Phase 3, lessons from failures)

**Decision:** The transition is flown by the hover cascade with five specific mechanisms,
each added after a diagnosed failure:

1. **All scheduling on the FORWARD air component** (not total airspeed). Total airspeed
   includes sink rate: a dive raised "airspeed", advanced the tilt schedule and cut thrust,
   deepening the dive (positive feedback loop).
2. **Tilt leads airspeed with a time ramp** (6 deg/s, paused by a sink guard). A tilt-rotor
   accelerates BY tilting; scheduling tilt purely on airspeed deadlocks at zero.
3. **Roll-yaw decoupling in the mixer.** At tilt, differential wing thrust (roll) tips
   thrust forward differentially = yaw moment. Yaw is deliberately slow (tilt servos),
   so the injected yaw built sideslip, the fins turned it into roll, and the loop
   diverged at 33 deg tilt. The mixer now feeds the predicted cross-moment back.
4. **Wing-lift feedforward from measured alpha with a thrust floor.** Estimating lift
   from pitch attitude overestimates in a climb and starved the motors (and with them
   all attitude authority) mid-transition. FF active only in TRANSITION/BACK: in gusty
   hover it fluctuates by tens of newtons and the velocity integrator is calmer.
5. **Back-transition on a time ramp with gated pitch laws.** Scheduling tilt-back on
   airspeed deadlocks (any altitude-loop thrust at tilt 90 points forward and keeps the
   speed up). The hover accel-law pitch is gated out above 8 m/s: at speed, pitch mainly
   adds LIFT (not thrust redirection) and ballooned the altitude by 14 m.

**Evidence:** eval/mission_calm.png — alt deviation 2.98 m, landing 0.24 m.

---

## D-13: Yaw bandwidth one third of roll/pitch

**Decision:** Rate-loop bandwidth 8 rad/s for roll/pitch, 2.8 rad/s for yaw; yaw attitude
command scaled by 0.4.

**Reason:** Yaw is actuated by differential NACELLE TILT through 120 deg/s servos — far
slower than motor-thrust response (70 ms). Driving yaw at the same bandwidth as roll made
the nacelles oscillate +/-15 deg during takeoff. In cruise the fins/rudder weathervane the
nose instead, and during transition the heading reference tracks the velocity vector
rather than fighting the fins.

---

## D-14: Inflow-corrected throttle inversion and Glauert slipstream

**Decision:** (a) The mixer inverts thrust->throttle including the axial-inflow term
(quadratic in omega). (b) Slipstream induced velocity uses the Glauert forward-speed
correction vi = vi_h^2/sqrt(V^2+vi_h^2), and the wing-wash overlap scales with sin^2(tilt).

**Reason:** (a) The static inversion under-delivers hugely at speed: at 14 m/s a "7.5 N"
static throttle produced ~1 N (prop near pitch speed) and the aircraft could not reach
the cruise gate. (b) The hover induced-velocity formula overstated the wash at speed by
~4x; the resulting washed-strip download cancelled the differential-thrust roll authority
and the aircraft rolled over at 33-48 deg tilt (see eval/debug_roll decomposition).

---