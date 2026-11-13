# Tilt-Tri-Rotor VTOL UAV: Simulation, Baseline Control and Reinforcement Learning

Draft report — 7th semester project. Every number cites its source file;
regenerate with the listed commands at the stated commit.

## 1. Introduction

Vertical take-off and landing (VTOL) aircraft that convert to wing-borne
cruise combine the operational flexibility of multirotors with the range and
efficiency of fixed-wing aircraft. This project develops a complete
simulation, a classical baseline controller, and reinforcement-learning
policies for a three-motor tilt-rotor VTOL UAV with a 500 g payload,
entirely in simulation, plus a browser ground station for manual practice and
autonomous delivery missions, and the complete electronics design (building
and flight-testing the hardware is next semester's scope).

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

Results (`eval/baseline_metrics.csv`, regenerated at the final commit):

| case | completed | max alt deviation | landing error | requirement |
|------|-----------|-------------------|---------------|-------------|
| calm air | yes | 2.98 m | 0.22 m | < 3 m, < 1 m ✓ |
| CG shifted 2 cm aft | yes | 2.95 m | 0.23 m | stable ✓ |
| 3 m/s wind + gusts | yes | 4.04 m | 12.3 m | stable ✓ (drifts) |
| 4 m/s wind + gusts | **no** | — | — | stable ✗ |
| 5 m/s wind + gusts | **no** | — | — | stable ✗ |

**Correction to the Phase 3 result.** The Phase 3 commit reported the 5 m/s
wind mission as completed. Re-running the full suite at the end showed it
fails. Bisecting proved the Phase 3 pass was luck: the takeoff in 5 m/s wind
was already a near-crash (bouncing off the ground twice, 70 m drift), and the
trajectory is chaotic — correcting the initial battery voltage on reset (a
Phase 4 bug fix, 22.2 → 25.2 V for one control step) reproduces Phase 3
bit-for-bit with the old value and fails with the new one. Root cause: hover
yaw is actuated by differential nacelle tilt, which saturates at 7° and, at
tilt 0, can only tilt one nacelle forward (half authority). In crosswind the
fins' weathervane moment exceeds it, the heading spins (−111° → −176° in 3 s)
and the spin couples into roll. A quick mixer change (moving the clipped tilt
to the other nacelle) was tried and rejected: it broke the calm-air altitude
limit (4.28 m). The proper fix — a weathervane yaw mode (hover nose-into-wind,
as PX4 does for VTOLs) with into-wind transitions — is future work, and needs
the RL residuals retrained because they are trained on top of this baseline.
The test is kept as a strict expected failure carrying this explanation.

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

**Mode decision (D-15).** 150k PPO steps each on hover, same seed and
hyperparameters: mode A (direct) never survived an episode (mean length
106/750 steps, best reward −17); mode B (residual) averaged 527/750 steps with
best reward +178. Mode B was used for all stages.

**Residual initialisation (D-16) — a failure worth reporting.** The first
overnight S1 run used library defaults (Gaussian exploration std 1.0). At 600k
steps it crashed 9/10 deterministic hover episodes, while the baseline alone
succeeds 10/10. Measured: residual noise of std 0.05 already drops baseline hover
success to 62 %, std 0.1 to 0 %. Fix: zero-initialised actor output (the untrained
policy *is* the baseline), initial std 0.1, learning rate 1e-4. A regression test
(`tests/test_env.py`) enforces it.

**Model selection (D-17).** Checkpoints every 100k steps; the reported policy is
the best checkpoint on 20 validation episodes (seeds 20000+), tested once on 50
held-out episodes (seeds 10000+), all with domain randomisation. The last
checkpoint is reported alongside.

**Training budget actually spent:** S1 hover 1.0M steps, S2 waypoints 1.0M
(warm-started from S1), S5 obstacles 1.5M (warm-started from S2, difficulty 0.4).
A WSL restart killed S2 at 233k steps; it resumed from the 200k checkpoint.

Results (`eval/rl_metrics.csv`, 50 test episodes each, DR on):

⟨RL_TABLE⟩

Against the pre-registered thresholds, verbatim: **S1 ≥ 95 % success —
⟨S1_VERDICT⟩. S5 ≥ 85 % success and ≤ 10 % collisions — ⟨S5_VERDICT⟩.**

⟨RL_DISCUSSION⟩

Scope cuts applied per the pre-agreed order (brief §5.6): RL transition
replaced by the scripted baseline transition; full S6 randomization reduced
to light per-episode DR (mass ±15 %, kT ±10 %, lag ±30 %, aero ±20 %, wind
≤ 3 m/s); SITL skipped; 50 test episodes instead of 100.

ONNX export (`rl/export_onnx.py`): policies run in onnxruntime with max
deviation from the PyTorch policy of 8.9e-8 (limit 1e-5) — the format the
companion computer would run.

## 6. Ground station: manual practice and autonomous delivery

`gcs/` (see `gcs/README.md`) runs the same physics and controller at real time
and streams rendered video and telemetry to a browser cockpit.

- **Manual:** Mode-2 sticks from the screen, keyboard, or a real RC transmitter
  in USB-joystick mode (axis mapping UI). Three handling modes, like a real
  autopilot: POSITION (sticks command velocity, centred throttle holds height,
  optional RL residual assist), ANGLE (sticks command attitude, pilot flies the
  collective), and a VTOL switch that flies the scripted transition to
  wing-borne flight, where the sticks steer heading and altitude.
- **Automatic delivery:** the user enters a destination; the vehicle climbs,
  turns to the leg heading, transitions, cruises, back-transitions, flies an
  RL-assisted precision approach, lands, releases the 0.5 kg payload (mass, CG
  and inertia change in the physics), and flies home the same way. Hops under
  300 m are flown as a multicopter.
- **Design:** the transition controller was validated for flight along +x, so
  each leg runs it in a rotated "leg frame" rather than modifying it (D-18).
- **Results (headless, calm air, `tests/test_gcs.py` + mission logs):**

  | mission | outcome | pad error | home error | time | battery |
  |---|---|---|---|---|---|
  | (400, 150) — 427 m, wing | complete | 0.04 m | 0.08 m | 217 s | 22.7 % |
  | (300, 300) — 424 m, wing | complete | 0.04 m | 0.06 m | 226 s | — |
  | (40, −30) — 50 m, multicopter | complete | 0.22 m | 0.24 m | 94 s | 12.8 % |
  | (−40, 40), (0, −60) — multicopter | complete | — | 0.21 / 0.11 m | ≈ 105 s | — |

- **Found by building it:** four mission-layer failure modes (yaw near the
  ground, fast turns exciting a yaw oscillation, back-transition overshoot after
  the payload drop, tail-first approach) and two physics bugs (0/0 NaN in the
  slipstream model with motors stopped; stale MuJoCo subtree mass after payload
  release). All fixed with regression tests (D-18).

**[FIGURE: cockpit screenshot during a delivery — capture with
`bash scripts/run_gcs.sh`, AUTO, destination (400, 150)]**

## 7. Electronics design

`hardware/` contains the block diagram, pin map, wiring rules, BOM and a
power-budget script (checked by `tests/test_hardware.py`). Pixhawk 6C Mini flight
controller (triple IMU: ICM-42688-P + BMI055 gyros/accelerometers, MS5611 baro),
u-blox M10 GNSS + compass, MS4525DO airspeed, TF-Luna lidars (down, forward),
Raspberry Pi Zero 2 W companion computer running the ONNX RL policy, Camera
Module 3 for pad detection, FPV camera + VTX, ExpressLRS RC, SiK telemetry,
3 × 40 A ESCs, 10 servos, separate servo and companion BECs.

| quantity | value | check |
|---|---|---|
| hover power / current / endurance | 638 W / 28.7 A / 8.4 min | — |
| cruise power / endurance | 176 W / 30 min (~31 km) | — |
| ESC margin over max motor current (23 A) | 1.72× | ≥ 1.5× ✓ |
| battery burst margin over 69 A peak | 3.3× | ≥ 2× ✓ |
| thrust-to-weight | 1.79 | ≥ 1.6 ✓ |
| avionics mass vs budget line | 274 g / 270 g (+1.5 %) | ≤ 10 % ✓ |

Every simulated block maps to a part with a named calibration test for Phase 9
(hardware/README.md §5). The largest sim-to-real gap is perception: the RL
obstacle policy observes 8 horizontal range rays, the hardware carries one
forward lidar + camera (a 360° lidar would bust the mass budget by 40 %).

## 8. Limitations and future work

- RL: ⟨RL_LIMITATION⟩
- **Wind:** the full baseline mission completes up to 3 m/s steady wind
  (12 m landing drift) and fails at 4–5 m/s (hover yaw saturation against the
  weathervane moment, §4). Needs a weathervane yaw mode + into-wind
  transitions, then RL retraining.
- Trim corridor pinch at 8–10 m/s: static infeasibility at the scheduled
  tilt; a tilt schedule reshaped around the corridor map would widen margin.
- Transition altitude margin is 0.02 m of the 3 m budget.
- Hover yaw authority (differential nacelle tilt, saturating at ±7°) is weak:
  it limits turn rates and makes sideways/tail-first flight fragile. A tilt
  range below 0° (backwards) or a small yaw vane would help.
- Not modelled: prop gyroscopic moments, adverse yaw, slipstream swirl,
  ground effect.
- Sim-to-real: all aero/propulsion parameters are ±30 % estimates until
  bench tests (Phase 9/10 next semester).

## 9. Reproducibility

WSL2 Ubuntu 26.04, conda env `drone311` (Python 3.11.18), pinned
`requirements.txt`. Every table regenerates from a command listed in the
README (`scripts/check_model.py`, `pytest`, `eval/run_mission.py`,
`control/trim.py`, `rl/select_checkpoint.py`, `scripts/evaluate_all.sh`,
`hardware/power_budget.py`); seeds are fixed throughout.

## Appendix A: 20 likely viva questions (with short answers)

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

16. **Why did the first RL run make the controller worse?** PPO's default
    exploration std (1.0) on a residual of ±25 % throttle is far larger than the
    baseline's error tolerance (std 0.1 already gives 0 % hover success), so every
    rollout crashed and the policy mean drifted. Zero-initialising the output
    layer and std 0.1 fixed it (D-16).

17. **Why not just report the last checkpoint?** The last PPO iterate is not
    the best one; picking the best checkpoint on *validation* seeds and testing on
    *disjoint* seeds is standard early stopping and keeps the test numbers honest
    (D-17). Both best and last are reported.

18. **How does the ground station fly to any direction if the transition was
    tuned along +x?** It rotates the controller's view of the state into a "leg
    frame" pointing at the destination. Actuator commands are body-frame, so
    nothing on the output side changes, and the validated controller is reused
    exactly (D-18).

19. **Why is the companion computer a Pi Zero 2 W, not a Pi 5 or Jetson?** The
    policy is a 128×128 MLP (≈ 0.2 ms per step); mass is the constraint — the
    avionics line is already at 274 g of a 270 g budget. Inner loops stay on the
    flight controller so the aircraft keeps flying if the companion fails (D-19).

20. **What happens to the aircraft when the payload is released?** 0.5 kg leaves
    6 cm below the CG: mass −14 %, CG moves up ≈ 1.2 cm, stall speed drops from
    11.5 to ≈ 10.7 m/s. The simulator updates mass, CG and inertia at release; the
    lighter aircraft floats further in the back-transition, which is why the
    deceleration starts 220 m before the pad (D-18).
