#!/usr/bin/env python3
"""Debug: telemetry for the first 20 s of the mission."""
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from sim.vehicle import Vehicle  # noqa: E402
from control.baseline import BaselineController  # noqa: E402
from control.pid import quat_to_euler  # noqa: E402

veh = Vehicle(seed=0)
ctrl = BaselineController()
s = veh.reset(seed=0, pos=(0, 0, 0.147))
dt = veh.dt_ctrl

for i in range(7000):
    u = ctrl.update(s, dt)
    s = veh.step(u, wind_override=(0, 0, 0))
    if ctrl.mode == "DONE" or s["pos"][2] < -0.5:
        print("END at t=", s["time"], "mode", ctrl.mode)
        break
    if i % 100 == 0:
        r, p, y = np.rad2deg(quat_to_euler(s["quat"]))
        print(f"t={s['time']:5.1f} {ctrl.mode:10s} "
              f"pos=({s['pos'][0]:6.1f},{s['pos'][1]:5.1f},{s['pos'][2]:5.1f}) "
              f"rpy=({r:6.1f},{p:6.1f},{y:6.1f}) "
              f"V={s['airspeed']:5.1f} u_thr=({u[0]:.2f},{u[1]:.2f},{u[2]:.2f}) "
              f"tilt=({np.rad2deg(s['tilt'][0]):4.1f},{np.rad2deg(s['tilt'][2]):4.1f}) "
              f"T=({s['thrusts'][0]:5.1f},{s['thrusts'][1]:5.1f},{s['thrusts'][2]:5.1f})")
