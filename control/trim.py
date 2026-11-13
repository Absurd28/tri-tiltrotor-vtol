"""Analytic trim solver: steady level flight from 0 to 20 m/s.

For each airspeed V (with wing tilt from the transition schedule), solves
[thr_wing, thr_tail, elevator, pitch] so that body-frame force residuals
(x, z) and pitch moment are zero, using the same aero and motor models as
the simulation. Produces eval/trim_table.csv and the tilt-corridor plot
(for which tilts a bounded-throttle trim exists at each speed).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402
from sim.aero import WingAero  # noqa: E402

G = 9.81


class TrimSolver:
    def __init__(self, params=None):
        self.p = params if params is not None else load_params()
        self.aero = WingAero(self.p)
        pr = self.p["propulsion"]
        self.kT = pr["kT"]
        self.w_max = pr["max_omega"]
        self.pitch_m = pr["prop_pitch_in"] * 0.0254
        self.kq_kt = pr["kQ_over_kT"]
        self.m = self.p["mass"]["total"]
        g = self.p["geometry"]
        lay = self.p["layout"]
        self.r_wing_motor = np.array([g["motor_left_x"], 0, lay["nacelle_hub_z"]])
        self.r_tail = np.array([g["motor_tail_x"], 0, lay["tail_hub_z"]])
        self.rho = self.p["simulation"]["air_density"]
        prop_r = pr["prop_diameter"] / 2
        self.disk_area = np.pi * prop_r**2

    def motor_thrust(self, thr, v_ax):
        w = thr * self.w_max
        if w < 1:
            return 0.0
        vp = w / (2 * np.pi) * self.pitch_m
        return self.kT * w * w * float(np.clip(1 - v_ax / vp, 0, 1.1))

    def residuals(self, x, V, tilt):
        thr_w, thr_t, elev, pitch = x
        # body air velocity for level flight at speed V, pitch attitude
        v_air_b = np.array([V * np.cos(pitch), 0.0, -V * np.sin(pitch)])
        hinge = dict(aileron_left=0.0, aileron_right=0.0,
                     elevator=elev, rudder=0.0)
        axis_b = np.array([np.sin(tilt), 0, np.cos(tilt)])
        v_ax = float(v_air_b @ axis_b)
        T_w = self.motor_thrust(thr_w, v_ax)        # per wing motor
        T_t = self.motor_thrust(thr_t, float(v_air_b[2]) * -1.0)

        # slipstream for the aero model (Glauert)
        vi_h2 = max(2 * T_w, 0.0) / (2 * self.rho * 2 * self.disk_area)
        vi = vi_h2 / np.sqrt(V**2 + vi_h2 + 1e-9)
        frac = 0.08 + 0.92 * np.sin(tilt)**2
        vs = self.p["aero"]["slipstream_decay"] * vi * frac
        slip = [(+1, vs, axis_b), (-1, vs, axis_b)]

        forces, tq = self.aero.forces(v_air_b, np.zeros(3), hinge, slip,
                                      notch_open=tilt < np.deg2rad(60))
        F = np.zeros(3)
        M = tq.copy()
        for f, r in forces:
            F += f
            M += np.cross(r, f)
        # thrust wrenches
        F += 2 * T_w * axis_b + T_t * np.array([0, 0, 1])
        M += 2 * np.cross(self.r_wing_motor, T_w * axis_b)
        M += np.cross(self.r_tail, T_t * np.array([0, 0, 1]))
        # gravity in body frame: with nose-up pitch theta the world -z pull
        # acquires a BACKWARD body-x component: g_b = (-g sin0, 0, -g cos0)
        F += self.m * G * np.array([-np.sin(pitch), 0, -np.cos(pitch)])
        return np.array([F[0], F[2], M[1]])

    def solve(self, V, tilt, x0=None):
        """Multi-start least squares: the residual surface has local minima
        (post-stall branch of the lift curve), so a cold start can stall."""
        starts = []
        if x0 is not None:
            starts.append(list(x0))
        starts += [
            [0.75, 0.75, 0.0, 0.02],    # hover-like
            [0.85, 0.3, 0.2, 0.05],     # mid-transition
            [0.9, 0.0, 0.3, 0.03],      # cruise-like
            [0.6, 0.5, 0.0, 0.1],
        ]
        best, best_res = None, np.inf
        for s0 in starts:
            sol = least_squares(
                self.residuals, s0, args=(V, tilt),
                bounds=([0, 0, -0.35, -0.3], [1, 1, 0.35, 0.35]),
                xtol=1e-10, ftol=1e-10)
            res = np.linalg.norm(sol.fun)
            if res < best_res:
                best, best_res = sol.x, res
            if best_res < 1e-3:
                break
        return best, best_res


def tilt_schedule(V, p):
    lo, hi = p["control"]["tilt_schedule_v"]
    return np.deg2rad(90.0) * np.clip((V - lo) / (hi - lo), 0, 1)


def main():
    import csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    p = load_params()
    ts = TrimSolver(p)
    rows = []
    x0 = None
    for V in np.arange(0, 20.5, 1.0):
        tilt = tilt_schedule(V, p)
        x, res = ts.solve(V, tilt, x0)
        feasible = res < 0.5
        rows.append([V, np.rad2deg(tilt), *x, res, feasible])
        x0 = list(x) if feasible else None   # don't warm-start from failures
        print(f"V={V:5.1f}  tilt={np.rad2deg(tilt):5.1f}deg  "
              f"thrW={x[0]:.3f} thrT={x[1]:.3f} elev={np.rad2deg(x[2]):+6.2f}deg "
              f"pitch={np.rad2deg(x[3]):+6.2f}deg  res={res:.2e} "
              f"{'OK' if feasible else 'INFEASIBLE'}")

    out = ROOT / "eval" / "trim_table.csv"
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["V", "tilt_deg", "thr_wing", "thr_tail", "elev_rad",
                    "pitch_rad", "residual", "feasible"])
        w.writerows(rows)
    print("wrote", out.relative_to(ROOT))

    # corridor: for each V, which fixed tilts admit a trim
    Vs = np.arange(0, 20.5, 1.0)
    tilts = np.deg2rad(np.arange(0, 91, 5))
    feas = np.zeros((len(tilts), len(Vs)), dtype=bool)
    for j, V in enumerate(Vs):
        x0c = None
        for i, tl in enumerate(tilts):
            x, res = ts.solve(V, tl, x0c)
            feas[i, j] = res < 0.5
            if feas[i, j]:
                x0c = list(x)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.imshow(feas, origin="lower", aspect="auto", cmap="Greens",
              extent=[Vs[0], Vs[-1], 0, 90], alpha=0.7)
    sched = [np.rad2deg(tilt_schedule(V, p)) for V in Vs]
    ax.plot(Vs, sched, "b-", lw=2, label="controller schedule")
    ax.set_xlabel("airspeed m/s")
    ax.set_ylabel("wing tilt deg")
    ax.set_title("Feasible tilt-airspeed corridor (green = trim exists)")
    ax.legend()
    fig.savefig(ROOT / "eval" / "tilt_corridor.png", dpi=110,
                bbox_inches="tight")
    print("wrote eval/tilt_corridor.png")


if __name__ == "__main__":
    main()
