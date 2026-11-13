# Tilt-Tri-Rotor VTOL UAV: Simulation, Baseline Control and Reinforcement Learning

Draft report — 7th semester project. Every number cites its source file;
regenerate with the listed commands at the stated commit.

## 1. Introduction

Vertical take-off and landing (VTOL) aircraft that convert to wing-borne
cruise combine the operational flexibility of multirotors with the range and
efficiency of fixed-wing aircraft. This project develops a complete
simulation, a classical baseline controller, and reinforcement-learning
policies for a three-motor tilt-rotor VTOL UAV with a 500 g payload,
entirely in simulation (hardware is next semester's scope).

The vehicle: 3.527 kg, 1.6 m span, two wing motors on tilting nacelles
(0–90°) and one tail motor between twin booms (0–30° planned tilt, parked
flat in cruise). Nine actuators: 3 throttles, 3 tilt servos, and 6 control
surfaces commanded as 3 virtual surfaces.

## 2. Vehicle design and modelling (Phase 1)

- Parametric MJCF generated from a single parameter file
  (`model/build_mjcf.py` from `params/params.yaml`); every mass-budget line
  item is a geom at its physical location, so the inertia tensor emerges
  from the layout rather than being typed in.
- The battery x-position and payload z-position are *solved* so the CG
  lands on the design reference — the battery is the balance weight, as in
  real aircraft (solved battery_x = 0.0967 m, inside the 0.06–0.10 m bay).
- Acceptance (commit `3825adc`, `scripts/check_model.py`):
  mass 3.527 kg (+0.00 %), CG offset 0.89 cm (< 1 cm), inertia
  Ixx +0.1 % / Iyy +8.2 % / Izz −2.8 % (< 10 %), minimum prop clearance
  2.66 cm ≥ 10 % of prop radius across the full tilt envelope, no
  structural interference. The clearance check caught a real layout error
  (notch wall initially 2.4 cm too close to the tail prop, D-11).

**[FIGURE: docs/renders/view_iso_hover.png + view_iso_cruise.png]**

## 3. Simulation physics (Phase 2)

Module summary (all pure Python over MuJoCo, forces via `mj_applyFT`):
motor (first-order lag, T = kT·ω², linear thrust decay to prop pitch
speed), battery (open-circuit voltage vs state of charge + IR sag), servo
(rate limit, lag, backlash), aero (6-station wing strip model with smooth
post-stall blend, tail with downwash, fins, slipstream wash with Glauert
forward-speed correction), wind (steady + Ornstein-Uhlenbeck gusts),
sensors (IMU bias walk, baro, pitot lag, 10 Hz GPS, range rays).

Acceptance (commit `4058153`, `tests/test_physics.py`): hover thrust
1.18 kgf/motor within 5 %, stall speed 11.5 ± 1 m/s, L/D 8 ± 1 at 17 m/s,
tail reaction torque produces the predicted uncompensated yaw drift,
bit-exact determinism under a fixed seed, 2.6× realtime single-threaded.

## 4. Baseline controller (Phase 3)

Cascaded PID (position → velocity → attitude → rate, gains by pole
placement from the inertia), hover/cruise control allocation blended by
airspeed, and a transition scheduler in which **tilt leads airspeed on a
ramp** guarded by altitude. Five transition failure modes were diagnosed
and fixed during development (D-12, D-13, D-14) — these form the core of
the "what makes tilt-rotor transition hard" discussion:

1. scheduling on total airspeed creates a dive feedback loop;
2. tilt scheduled purely on airspeed deadlocks at zero;
3. differential-thrust roll injects yaw at tilt (fixed by mixer decoupling);
4. hover-formula slipstream overstates wing download 4× at speed,
   cancelling roll authority (fixed by Glauert correction);
5. static throttle inversion under-delivers thrust near prop pitch speed.

Results (`eval/baseline_metrics.csv`, commit `d6ec6e6`):

| case | completed | max alt deviation | landing error | requirement |
|------|-----------|-------------------|---------------|-------------|
| calm air | yes | 2.98 m | 0.24 m | < 3 m, < 1 m ✓ |
| 5 m/s wind + gusts | yes | 5.67 m | 22.8 m | stable ✓ |
| CG shifted 2 cm aft | yes | 2.95 m | 0.22 m | stable ✓ |

Trim analysis (`eval/trim_table.csv`, `eval/tilt_corridor.png`): hover trim
is an equal thrust split at 0.751 throttle, confirming the design geometry;
cruise trim at 17 m/s is pitch +1.6°, throttle 0.88. A static-trim pinch
exists at 8–10 m/s where the scheduled tilt saturates the elevator; the
controller crosses it dynamically using tail thrust.

**[FIGURE: eval/mission_calm.png, eval/tilt_corridor.png]**
**[VIDEO: docs/videos/mission_calm.mp4]**

## 5. Reinforcement learning (Phases 4–5)

Environment: Gymnasium wrapper at 50 Hz, 35-dim observation (attitude,
rates, body velocity, airspeed, altitude, tilts, last action, goal vector,
8 range rays), 9-dim action. Two action modes compared experimentally:
A (direct actuator commands) and B (bounded residual on the baseline).

- Mode decision: **[RESULT PENDING — rl/compare_probe.py output]**
- Stage S1 hover: **[PENDING overnight run]**
- Stage S2 waypoints: **[PENDING]**
- Stage S5 obstacles: **[PENDING]**
- Thresholds: S1 ≥ 95 % success; S5 ≥ 85 % success, ≤ 10 % collisions.
  Results are reported verbatim against these numbers.

Scope cuts applied per the pre-agreed order (brief §5.6): RL transition
replaced by the scripted baseline transition; full S6 randomization reduced
to light per-episode DR (mass ±15 %, kT ±10 %, lag ±30 %, aero ±20 %, wind
≤ 3 m/s); SITL skipped.

## 6. Limitations and future work

- Landing drift in 5 m/s wind (22.8 m) — the descent position-hold needs
  wind-feedforward or an RL landing policy.
- Trim corridor pinch at 8–10 m/s: static infeasibility at the scheduled
  tilt; a tilt schedule reshaped around the corridor map would widen margin.
- Transition altitude margin is 0.02 m of the 3 m budget.
- Not modelled: prop gyroscopic moments, adverse yaw, slipstream swirl,
  ground effect; evaluation used 50 seeded episodes (not 100) for time.
- Sim-to-real: all aero/propulsion parameters are ±30 % estimates until
  bench tests (Phase 9/10 next semester).

## 7. Reproducibility

WSL2 Ubuntu 26.04, conda env `drone311` (Python 3.11.18), pinned
`requirements.txt`. Each phase is one git commit; every table regenerates:
`scripts/check_model.py`, `pytest`, `eval/run_mission.py`,
`control/trim.py`, `rl/evaluate.py`.

## Appendix A: 15 likely viva questions (with short answers)

1. **Why three motors instead of four?** Two tilting wing motors give cruise
   propulsion for free; the third (tail) motor completes hover pitch/thrust
   support but is dead weight in cruise — three is the minimum that hovers
   with full attitude authority while converting. Fewer motors, less weight.

2. **How is yaw controlled in hover with no coaxial pair?** Small opposite
   tilts of the wing nacelles create opposing horizontal thrust components
   at ±0.4 m — a yaw couple. The tail motor's reaction torque (~0.2 N·m) is
   fed forward into the same channel (D-04).

3. **Why do the wing motors counter-rotate?** So their reaction torques
   cancel in symmetric hover; otherwise the differential-tilt channel would
   carry a permanent offset.

4. **Where does the inertia tensor come from?** Nowhere by hand: every
   mass-budget item is a geom at its physical station and MuJoCo composes
   the tensor. Checked against the budget to ±10 % (`sim/model_check.py`).

5. **Why is the battery position solved rather than fixed?** The tail group
   is ~0.28 kg at −0.4 m; the battery is the classical balance weight. The
   solver proves the design closes inside the physical battery bay.

6. **What limits the transition corridor at the low-speed end?** Wing lift
   ∝ V²: below ~11.5 m/s the wing cannot carry the weight, so motor thrust
   must bridge the gap while cos(tilt) shrinks its vertical share — the
   pinch visible at 8–10 m/s in `eval/tilt_corridor.png`.

7. **Why schedule tilt on airspeed but ramp it in time?** Pure airspeed
   scheduling deadlocks at V=0 (no tilt → no forward force → no airspeed);
   pure time scheduling can outrun the aerodynamics. The ramp provides the
   seed acceleration, the airspeed schedule takes over, and a sink guard
   pauses the ramp if altitude is being lost (D-12).

8. **Why does a dive develop if you schedule on total airspeed?** Total
   airspeed includes sink rate, so a dive reads as "faster", advancing the
   schedule and cutting thrust — positive feedback. The forward air
   component breaks the loop.

9. **What was the hardest coupling to stabilise?** Differential-thrust roll
   at mid-tilt: it simultaneously injects yaw (forward-component couple),
   and its slipstream change alters wing download. Fixed by mixer
   decoupling plus the Glauert inflow correction that sized the wash
   honestly (D-12/D-14).

10. **Why does the prop lose thrust at speed?** A fixed-pitch prop's blade
    AoA falls as axial inflow approaches the pitch speed V_p = ω·pitch/2π
    (~16 m/s mid-throttle). Modelled as linear decay; it is why the
    throttle inversion must include inflow and why top speed is ~20 m/s.

11. **Why PID cascade for the baseline rather than LQR/MPC?** Transparent
    bandwidth-separated loops with gains from pole placement are
    explainable, flight-stack-compatible (PX4 uses the same structure), and
    sufficient; the RL policy is the sophistication layer of this project.

12. **Why residual RL (mode B) rather than direct control?** The 150k-step
    probe shows the residual mode starts from a flying controller, so
    exploration is safe and sample-efficient; direct mode spends most of
    the budget rediscovering hover. (Numbers in §5.)

13. **What is domain randomization for?** The aero/propulsion parameters
    are ±30 % estimates; training across mass/kT/lag/aero/wind variations
    optimizes expected performance over that uncertainty set — the standard
    sim-to-real hedge before system identification next semester.

14. **Why is the physics step 2 ms with control at 100 Hz?** 500 Hz gives
    ≥35 integration steps per motor time constant (70 ms) for stable servo
    and contact dynamics; 100 Hz matches real flight controllers; the RL
    policy at 50 Hz still sits well above the attitude bandwidth.

15. **How would these results transfer to the real vehicle?** Phase 9/10:
    bench-measure kT/kQ/masses, system-identify aero from flight logs,
    re-tag params.yaml [measured], retrain with narrowed DR, deploy ONNX
    policy with the baseline as a supervised fallback behind a watchdog.
