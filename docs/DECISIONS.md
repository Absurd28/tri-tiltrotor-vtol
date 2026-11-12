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