#!/usr/bin/env python3
"""Decompose roll moments during the transition failure (t=18..23)."""
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402
from sim.vehicle import Vehicle  # noqa: E402
from control.baseline import BaselineController  # noqa: E402
from control.pid import quat_to_euler  # noqa: E402

veh = Vehicle(seed=0)
ctrl = BaselineController()
s = veh.reset(seed=0, pos=(0, 0, 0.147))
dt = veh.dt_ctrl

for i in range(6000):
    u = ctrl.update(s, dt)
    s = veh.step(u, wind_override=(0, 0, 0))
    t = s["time"]
    if t > 23.5 or ctrl.mode == "CRUISE":
        break
    if t > 18.0 and i % 20 == 0:
        d = veh.data
        R = s["R_bw"]
        com = d.subtree_com[veh.bid_base]
        # thrust roll moment about CG (world->body x)
        M_thr = np.zeros(3)
        for k, sid in enumerate(veh.sid_thrust):
            axis_w = d.site_xmat[sid].reshape(3, 3)[:, 2]
            F = axis_w * s["thrusts"][k]
            M_thr += R.T @ np.cross(d.site_xpos[sid] - com, F)
        # aero moment: recompute from the aero module at current state
        hinge = dict(
            aileron_left=d.qpos[veh.jadr["j_aileron_left"]],
            aileron_right=d.qpos[veh.jadr["j_aileron_right"]],
            elevator=0.5 * (d.qpos[veh.jadr["j_elevator_left"]]
                            + d.qpos[veh.jadr["j_elevator_right"]]),
            rudder=0.5 * (d.qpos[veh.jadr["j_rudder_left"]]
                          + d.qpos[veh.jadr["j_rudder_right"]]),
        )
        v_air_b = s["v_air_b"]
        omega_b = s["omega_b"]
        # reconstruct slipstream entries as vehicle.step does
        slip = []
        for k in (0, 1):
            vi = veh.motors[k].induced_velocity(s["thrusts"][k], veh.aero.rho,
                                                veh.battery.disk_area)
            tilt_k = s["tilt"][k]
            frac = 0.1 + 0.9 * np.sin(tilt_k)
            axis_w = d.site_xmat[veh.sid_thrust[k]].reshape(3, 3)[:, 2]
            slip.append((+1 if k == 0 else -1,
                         veh.p["aero"]["slipstream_decay"] * vi * frac,
                         R.T @ axis_w))
        forces, tq = veh.aero.forces(v_air_b, omega_b, hinge, slip, True)
        M_aero = tq.copy()
        M_wash = np.zeros(3)
        for (F, r), st in zip(forces[:6], veh.aero.stations):
            if st["washed"]:
                M_wash += np.cross(r, F)
        for F, r in forces:
            M_aero += np.cross(r, F)
        r_, p_, y_ = np.rad2deg(quat_to_euler(s["quat"]))
        print(f"t={t:5.1f} roll={r_:6.1f} p={np.rad2deg(omega_b[0]):6.1f}d/s "
              f"Mx_thr={M_thr[0]:6.2f} Mx_aero={M_aero[0]:6.2f} "
              f"(wash {M_wash[0]:6.2f}) ailL={np.rad2deg(hinge['aileron_left']):5.1f} "
              f"T=({s['thrusts'][0]:4.1f},{s['thrusts'][1]:4.1f}) "
              f"tilt=({np.rad2deg(s['tilt'][0]):4.1f},{np.rad2deg(s['tilt'][1]):4.1f})")
