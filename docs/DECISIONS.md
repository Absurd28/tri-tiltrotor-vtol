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