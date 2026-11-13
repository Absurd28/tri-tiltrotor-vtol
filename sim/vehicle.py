"""Vehicle: MuJoCo model + motors + battery + servos + aero + wind + sensors.

Control interface (9 commands, the project's canonical action vector):
    u = [thr_L, thr_R, thr_T,          0..1   motor throttles
         tilt_L, tilt_R, tilt_T,       0..1   normalized tilt (0=up, 1=max)
         aileron, elevator, rudder]    -1..1  virtual surfaces

Virtual-to-physical surface mapping (signs follow the hinge conventions
documented in sim/aero.py; D-06):
    aileron  +1 -> roll right: left hinge -a_max*u, right hinge +a_max*u...
    (see _surface_map below; chosen so positive command = positive body moment:
     +aileron = +roll rate (right wing down is NEGATIVE here: FLU +roll = left
     wing down? No: rotation about +x moves +y(left) toward +z(up): LEFT wing
     RISES = roll right. So +aileron command = roll RIGHT.)
    elevator +1 -> nose up
    rudder   +1 -> nose left (+yaw about z-up)

step(u) advances one control period (default 100 Hz) = N physics substeps at
500 Hz. Forces applied via mj_applyFT into data.qfrc_applied.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(ROOT))

from params import load_params  # noqa: E402
from sim.motor import Motor, Battery  # noqa: E402
from sim.servo import ServoBank  # noqa: E402
from sim.aero import WingAero  # noqa: E402
from sim.wind import Wind  # noqa: E402
from sim.sensors import SensorSuite  # noqa: E402

MODEL_XML = ROOT / "model" / "vtol.xml"


class Vehicle:
    def __init__(self, xml_path=MODEL_XML, seed=0, wind=None, params=None,
                 model=None):
        self.p = params if params is not None else load_params()
        self.model = model if model is not None else \
            mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.dt_phys = self.model.opt.timestep
        self.dt_ctrl = self.p["simulation"]["control_dt"]
        self.n_sub = max(int(round(self.dt_ctrl / self.dt_phys)), 1)

        spins = self.p["propulsion"]["prop_spin"]
        self.motors = [Motor(self.p, s) for s in spins]
        self.battery = Battery(self.p)
        self.servos = ServoBank(self.p)
        self.aero = WingAero(self.p)
        self.wind = wind if wind is not None else Wind(seed=seed)
        self.sensors = SensorSuite(self.model, self.p, seed=seed)
        self.notch_open = True   # tail prop exposed (hover); cruise closes it

        m = self.model
        self.bid_base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.bid_nac = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
                        for n in ("nacelle_left", "nacelle_right", "tail_pod")]
        self.sid_thrust = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)
                           for n in ("thrust_left", "thrust_right", "thrust_tail")]
        self.jadr = {}
        for jn in ("j_nacelle_left", "j_nacelle_right", "j_tail_pod",
                   "j_aileron_left", "j_aileron_right",
                   "j_elevator_left", "j_elevator_right",
                   "j_rudder_left", "j_rudder_right"):
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self.jadr[jn] = m.jnt_qposadr[j]

        self.tilt_max = np.deg2rad([self.p["tilt"]["wing_max_deg"],
                                    self.p["tilt"]["wing_max_deg"],
                                    self.p["tilt"]["tail_max_deg"]])
        sr = self.p["surfaces"]
        self.surf_max = np.deg2rad([sr["aileron_range_deg"],
                                    sr["elevator_range_deg"],
                                    sr["rudder_range_deg"]])
        self.key_parked = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "parked")
        self.last_thrusts = np.zeros(3)
        self.v_batt = self.battery.v_nom
        self.reset(seed)

    # ------------------------------------------------------------------ #
    def reset(self, seed=None, pos=None, quat=None, vel=None, hover_init=False):
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_parked)
        if pos is not None:
            self.data.qpos[0:3] = pos
        if quat is not None:
            self.data.qpos[3:7] = quat
        if vel is not None:
            self.data.qvel[0:3] = vel
        for mtr in self.motors:
            mtr.reset()
        self.battery.energy_J = self.battery.capacity_J
        self.servos.reset()
        self.wind.reset(seed)
        self.sensors.reset(seed)
        self.last_thrusts = np.zeros(3)
        if hover_init:
            # start motors at hover speed so the vehicle doesn't drop
            w_h = np.sqrt(self.p["propulsion"]["hover_thrust_per_motor"]
                          / self.p["propulsion"]["kT"])
            for mtr in self.motors:
                mtr.reset(w_h)
        mujoco.mj_forward(self.model, self.data)
        return self.state()

    # ------------------------------------------------------------------ #
    def _surface_map(self, u_surf):
        """[aileron, elevator, rudder] in -1..1 -> 6 hinge commands (rad).
        +aileron = roll right: left TE down (hinge -), right TE up (hinge +).
        Rotation about +x carries +y(left) to +z(up): left wing up = roll right.
        Left lift up needs left TE down = negative left hinge... that gives
        MORE left lift -> left wing rises -> +roll. So left=-u, right=+u?
        Careful: +roll(right) needs LEFT lift UP: left hinge NEGATIVE (TE down),
        right hinge POSITIVE (TE up).
        +elevator = nose up = tail lift down = both TE up = hinge +.
        +rudder = nose left (+z moment): fin force to the RIGHT (-y) at the
        tail: rudder TE left = hinge NEGATIVE.
        """
        a, e, r = np.clip(u_surf, -1, 1) * self.surf_max
        return np.array([-a, +a, +e, +e, -r, -r])

    def step(self, u, wind_override=None):
        """Advance one control period. u: 9-vector (see module docstring)."""
        u = np.asarray(u, dtype=float)
        thr = np.clip(u[0:3], 0, 1)
        tilt_cmd = np.clip(u[3:6], 0, 1) * self.tilt_max
        surf_cmd = self._surface_map(u[6:9])

        m, d = self.model, self.data
        for _ in range(self.n_sub):
            # servo shaping at physics rate, then MJCF position targets
            shaped = self.servos.step(np.concatenate([tilt_cmd, surf_cmd]),
                                      self.dt_phys)
            d.ctrl[0:9] = shaped

            wind_w = self.wind.step(self.dt_phys) if wind_override is None \
                else np.asarray(wind_override)

            d.qfrc_applied[:] = 0.0

            R_bw = d.xmat[self.bid_base].reshape(3, 3)   # body->world
            v_com_w = d.qvel[0:3]
            # free joint: qvel[3:6] is BODY-frame angular velocity (verified
            # empirically in tests/check_conventions.py)
            omega_b = d.qvel[3:6].copy()
            omega_w = R_bw @ omega_b
            v_air_w = v_com_w - wind_w
            v_air_b = R_bw.T @ v_air_w

            # ---- motors ----
            v_batt, v_scale = self.battery.step(self.last_thrusts, self.dt_phys)
            self.v_batt = v_batt
            thrusts = np.zeros(3)
            slip = []
            for i, mtr in enumerate(self.motors):
                sid = self.sid_thrust[i]
                axis_w = d.site_xmat[sid].reshape(3, 3)[:, 2]
                # air velocity at the site (incl. rotation)
                r_w = d.site_xpos[sid] - d.subtree_com[self.bid_base]
                v_site_w = v_com_w + np.cross(omega_w, r_w)
                v_ax = float(np.dot(v_site_w - wind_w, axis_w))
                T, Q = mtr.step(thr[i], v_ax, self.dt_phys, v_scale)
                thrusts[i] = T
                force_w = axis_w * T
                torque_w = axis_w * Q
                mujoco.mj_applyFT(m, d, force_w, torque_w,
                                  d.site_xpos[sid], self.bid_nac[i],
                                  d.qfrc_applied)
                if i < 2:  # wing-prop slipstream over the wing
                    vi = mtr.induced_velocity(T, self.aero.rho,
                                              self.battery.disk_area,
                                              float(np.linalg.norm(v_air_b)))
                    tilt = d.qpos[self.jadr["j_nacelle_left" if i == 0
                                            else "j_nacelle_right"]]
                    # geometric overlap of the slipstream tube with the wing:
                    # the disc sits 0.13 m AHEAD of the LE, so at low/mid tilt
                    # the wake passes mostly in front of the wing; sin^2 keeps
                    # mid-tilt wash moderate (sin was strong enough to cancel
                    # the differential-thrust roll authority - see debug_roll)
                    frac = 0.08 + 0.92 * np.sin(tilt)**2
                    vs = self.p["aero"]["slipstream_decay"] * vi * frac
                    axis_b = R_bw.T @ axis_w
                    slip.append((+1 if i == 0 else -1, vs, axis_b))
            self.last_thrusts = thrusts

            # ---- aero ----
            hinge = dict(
                aileron_left=d.qpos[self.jadr["j_aileron_left"]],
                aileron_right=d.qpos[self.jadr["j_aileron_right"]],
                elevator=0.5 * (d.qpos[self.jadr["j_elevator_left"]]
                                + d.qpos[self.jadr["j_elevator_right"]]),
                rudder=0.5 * (d.qpos[self.jadr["j_rudder_left"]]
                              + d.qpos[self.jadr["j_rudder_right"]]),
            )
            forces, tq_b = self.aero.forces(v_air_b, omega_b, hinge, slip,
                                            self.notch_open)
            com_w = d.subtree_com[self.bid_base]
            for F_b, r_b in forces:
                F_w = R_bw @ F_b
                p_w = com_w + R_bw @ r_b
                mujoco.mj_applyFT(m, d, F_w, np.zeros(3), p_w,
                                  self.bid_base, d.qfrc_applied)
            mujoco.mj_applyFT(m, d, np.zeros(3), R_bw @ tq_b, com_w,
                              self.bid_base, d.qfrc_applied)

            mujoco.mj_step(m, d)

        return self.state()

    # ------------------------------------------------------------------ #
    def state(self) -> dict:
        d = self.data
        R_bw = d.xmat[self.bid_base].reshape(3, 3)
        v_b = R_bw.T @ d.qvel[0:3]
        wind_now = self.wind.steady + self.wind.gust
        v_air_b = R_bw.T @ (d.qvel[0:3] - wind_now)
        tilts = np.array([d.qpos[self.jadr[j]] for j in
                          ("j_nacelle_left", "j_nacelle_right", "j_tail_pod")])
        return dict(
            pos=d.qpos[0:3].copy(), quat=d.qpos[3:7].copy(),
            vel_w=d.qvel[0:3].copy(), vel_b=v_b,
            omega_b=d.qvel[3:6].copy(),   # free joint: already body frame
            v_air_b=v_air_b, airspeed=float(np.linalg.norm(v_air_b)),
            tilt=tilts, R_bw=R_bw,
            motor_omega=np.array([mtr.omega for mtr in self.motors]),
            thrusts=self.last_thrusts.copy(),
            v_batt=self.v_batt, soc=self.battery.soc, time=d.time,
        )

    def read_sensors(self) -> dict:
        s = self.state()
        return self.sensors.read(self.data, s["v_air_b"][0], self.dt_ctrl)
