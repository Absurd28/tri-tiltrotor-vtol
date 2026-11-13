# HANDOFF.md

## Phase 3: Trim and baseline controller — COMPLETE (2026-11-13)

### Done
- control/pid.py: PID + quaternion attitude helpers (FLU nose-up-positive
  euler convention, documented — two sign bugs lived there)
- control/mixer.py: hover/cruise allocation, airspeed blend, roll-yaw
  decoupling, tail-reaction feedforward, inflow-corrected throttle inversion
- control/baseline.py: cascaded PID (pos->vel->att->rate), transition state
  machine (TAKEOFF/HOVER/TRANSITION/CRUISE/BACK/HOVER2/LAND), tilt-leads-
  airspeed ramp, sink guard, lift feedforward, back-transition pitch gating
- control/trim.py: analytic trim solver 0-20 m/s + tilt-corridor map
- eval/run_mission.py: mission runner, metrics CSV, plots, video rendering

### Evidence (eval/baseline_metrics.csv)
| case | completed | alt dev (m) | landing (m) | limit |
|------|-----------|-------------|-------------|-------|
| calm | yes | 2.98 | 0.24 | <3 / <1 ✓ |
| wind 5 m/s | yes | 5.67 | 22.8 | "stable" ✓ (see issues) |
| CG +2 cm | yes | 2.95 | 0.22 | stable ✓ |

- eval/trim_table.csv: hover trim = equal split at 0.751 throttle (design
  intent confirmed); cruise 17 m/s at pitch +1.6 deg, thr 0.88
- eval/tilt_corridor.png: feasible corridor; known pinch at V 8-10 where
  static trim saturates the elevator (crossed dynamically by the controller)

### Open issues
1. Landing drift in 5 m/s wind: 22.8 m (gates now use ground speed; the
   descent itself still drifts). Candidate for the RL comparison story.
2. Trim corridor pinch V 8-10 m/s: static infeasibility at scheduled tilt;
   flight crosses it transiently. Report discussion item.
3. Transition altitude margin is thin (2.98 of 3.0 m) — sensitive to gains.

### Phase history
- Phase 0 (568c2bc): scaffold, WSL2 env, 23 tests
- Phase 1 (3825adc): parametric MJCF, mass/CG/inertia/clearance checks pass
- Phase 2 (4058153): physics (motor/servo/aero/wind/sensors), 40 tests

## Next: Phase 4 — RL environment (Gymnasium), then overnight S1/S2 training
