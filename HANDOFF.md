# HANDOFF.md

## Status: Phases 0–6 complete (simulation scope), 2026-11-14

Everything in the brief's simulation scope is built, tested and documented, with the
pre-agreed cuts (RL transition → scripted; S6 → light DR; SITL skipped; 50 test episodes).
Added on request: browser ground station (manual RC practice + autonomous delivery) and the
complete electronics design. Hardware build/flight test = Phases 9–10 (next phase).

| Area | Result | Evidence |
|---|---|---|
| Model (P1) | mass exact, CG 0.89 cm, inertia ≤ 8.2 %, prop clearance 2.66 cm | `scripts/check_model.py`, `tests/test_model.py` |
| Physics (P2) | hover thrust ±5 %, stall 11.5 ± 1 m/s, L/D 8 ± 1, deterministic | `tests/test_physics.py` |
| Baseline (P3) | calm mission alt dev 2.98 m (< 3), landing 0.22 m (< 1); CG +2 cm ✓; 3 m/s wind ✓; **4–5 m/s wind ✗** | `eval/baseline_metrics.csv`, report §4 |
| RL (P4–5) | **thresholds NOT met**: S1 86 % (≥ 95), S5 8 % / 52 % coll. (≥ 85 / ≤ 10); hover tracking −26 % | `eval/rl_metrics.csv`, report §5 |
| Ground station | manual (ANGLE/POSITION/wing) + auto delivery; round trips land 0.04–0.24 m from pads | `gcs/`, `tests/test_gcs.py` |
| Electronics | full design; ESC 1.72×, BEC 1.67×, battery 3.3× margins; T/W 1.79; avionics +7 % and motors +7 % mass (within 10 %) | `hardware/`, `tests/test_hardware.py` |
| Tests | 69 passed, 1 strict xfail (5 m/s wind, documented) | `pytest` |

## Open issues (ordered by value)
1. **Hover yaw authority** (differential nacelle tilt, saturates at 7°, half authority at tilt 0)
   — root cause of the 4–5 m/s wind failure and of many RL-episode departures. Fix: weathervane
   yaw mode (nose into wind in hover) + into-wind transitions. Then **retrain all RL stages**
   (residuals depend on the baseline). D-20.
2. RL budget: ≈ 1M steps per stage. Obstacle avoidance in guidance mode G (D-21) needs a
   difficulty curriculum and ≥ 10M steps; select on ≥ 100 validation episodes.
3. Waypoint task is time-limited by design (3 × up to 17 m in 30 s at 4 m/s); scale the time
   budget with distance.
4. Trim corridor pinch at 8–10 m/s; transition altitude margin 0.02 m of 3 m.

## How to run
- Tests: `pytest` (WSL, env drone311; runner `C:/Temp/run_pytest.sh`).
- Ground station: `bash scripts/run_gcs.sh` → http://localhost:8765 (uses `runs/s1_hover/best.onnx`).
- Everything else: README "Reproduce every result".

## Gotchas
- Long WSL jobs must be launched detached:
  `Start-Process wsl.exe -ArgumentList "-d","Ubuntu-26.04","--","bash","script.sh" -WindowStyle Hidden`
  (`nohup` inside `wsl -- bash -c` dies with wsl.exe; a WSL restart killed training once).
- `runs/` (policies, checkpoints, selection.csv) is not in git — back it up separately.
- Path contains an apostrophe: write WSL scripts to C:\Temp, no BOM.

## Commit history
P0 568c2bc · P1 3825adc · P2 4058153 · P3 d6ec6e6 · P4 f85f54d, afd54ca · P5 a64e6af ·
GCS + hardware 89b215b · corrections b89fbec · mode G 9be712e · final: tag `v1.0-sim`.
