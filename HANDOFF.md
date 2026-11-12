# HANDOFF.md

## Phase 1: Modelling, 3D and MuJoCo — COMPLETE (2026-11-12)

### Done
- model/build_mjcf.py: parametric MJCF generator from params.yaml
  - 52 geoms, every mass-budget line item placed physically
  - battery x and payload z SOLVED for CG (battery=0.0967 m, payload z=-0.06 m)
  - 3 tilt hinges (wing 0-90 deg, tail 0-30 deg) + 6 surface hinges, 9 position actuators
  - thrust sites (left/right/tail), imu/pitot/rangefinder sites + sensors
- model/vtol.xml: generated model (committed), model_info.yaml traceability
- sim/model_check.py: mass/CG/inertia report + tilt-sweep clearance (mj_geomDistance)
- scripts/check_model.py: ALL CHECKS PASS
  - mass 3.527 kg (0.00% err), CG 0.89 cm, Ixx +0.1% / Iyy +8.2% / Izz -2.8%
  - min prop clearance 2.66 cm >= 1.52 cm target; no structural interpenetration
- scripts/render_views.py: 7 views in docs/renders/ (EGL, offwidth 1280)
- scripts/viewer_tilt.py: interactive viewer, keys Q/A W/S E/D Z X
- tests/test_model.py: 11 acceptance tests; full suite 23/23 green

### Parameter changes
- params.yaml: added `layout:` section (z-heights, bays, servo placement)
- avionics_nose_x 0.40 -> 0.325 (Iyy within 10%); see D-08

### Open issues
- Iyy +8.2% vs budget: the detailed buildup is heavier in pitch than the early
  estimate; within tolerance but worth re-measuring on the real airframe (Phase 10).
- Renders use MUJOCO_GL=egl (WSL2); osmesa lib not installed (not needed).

## Next: Phase 2 — simulation physics (motor, servo, aero, wind, sensors)