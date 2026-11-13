"""Control allocation: body moment + thrust commands -> 9 actuator commands.

Hover allocation (tilt ~ 0):
  roll  M_x: wing thrust differential, lever = motor_y (0.40 m)
  pitch M_y: wing pair vs tail thrust, levers 0.20 m / 0.40 m.
             With F split [T_w, T_w, T_t]: M_y = -0.2*(2 T_w) + 0.4*T_t,
             so equal thirds is moment-free (the geometry was designed so).
  yaw   M_z: opposite small wing-tilt increments: M_z ~ 0.8 * T_w * d_tilt,
             plus feedforward for the tail reaction torque (D-04).
Cruise allocation (tilt ~ 90): moments go to the surfaces, scaled by 1/q_dyn.
Blend: weight w(q_dyn-equivalent airspeed) moves authority motor->surface.

Sign conventions (FLU): +M_x roll right (left wing up), +M_y nose DOWN,
+M_z nose left. Virtual surfaces: +aileron = +M_x, +elevator = nose UP
(= -M_y), +rudder = +M_z (nose left).
"""
from __future__ import annotations

import numpy as np


class Mixer:
    def __init__(self, p: dict):
        g = p["geometry"]
        c = p["control"]
        a = p["aero"]
        self.y_m = g["motor_left_y"]          # 0.40
        self.x_m = g["motor_left_x"]          # 0.20
        self.x_t = -g["motor_tail_x"]         # 0.40 (abs)
        self.kq_kt = p["propulsion"]["kQ_over_kT"]
        self.T_max = p["propulsion"]["max_thrust_per_motor"]
        self.kT = p["propulsion"]["kT"]
        self.w_max = p["propulsion"]["max_omega"]
        self.pitch_m = p["propulsion"]["prop_pitch_in"] * 0.0254
        self.rho = p["simulation"]["air_density"]
        self.blend_v = c["mixer_blend_v"]
        self.tail_fade_v = c["tail_fade_v"]
        self.tilt_max = np.deg2rad(p["tilt"]["wing_max_deg"])

        # surface moment effectiveness per unit (q_dyn * deflection), from the
        # aero geometry: M = q*S_eff*CLa*tau * arm  =>  defl = M/(q*K)
        S = g["wing_area"]
        self.K_ail = (2 * 0.0546 * a["CL_alpha"] * a["aileron_tau"] * 0.676)
        self.K_elev = (2 * g["stabilizer_area_each"] * a["CLa_tail"]
                       * a["elevator_tau"] * 0.50)
        self.K_rud = (2 * g["fin_area_each"] * abs(a["CYb_fin"])
                      * a["rudder_tau"] * 0.54)
        sr = p["surfaces"]
        self.surf_max = np.deg2rad([sr["aileron_range_deg"],
                                    sr["elevator_range_deg"],
                                    sr["rudder_range_deg"]])

    @staticmethod
    def _ramp(x, lo, hi):
        return float(np.clip((x - lo) / (hi - lo), 0.0, 1.0))

    def thrust_to_throttle(self, T, v_scale=1.0, v_axial=0.0):
        """Invert the thrust model INCLUDING axial inflow:
            T = kT*w^2 - kT*w*v_ax*2*pi/pitch
        (quadratic in w). The static inversion under-delivers badly at speed:
        at 14 m/s a 'hover' throttle produces almost no thrust because the
        prop is near its pitch speed."""
        T = np.clip(T, 0.0, self.T_max)
        if T <= 0.0:
            return 0.0
        b = v_axial * 2 * np.pi / self.pitch_m
        w = 0.5 * (b + np.sqrt(b * b + 4 * T / self.kT))
        return float(np.clip(w / (self.w_max * max(v_scale, 0.3)), 0.0, 1.0))

    def mix(self, F_total, M_cmd, tilt_cmd, airspeed, v_scale=1.0,
            tail_reaction_ff=True):
        """
        F_total: total motor thrust magnitude command (N)
        M_cmd:   body moment command (3,) N*m
        tilt_cmd: scheduled wing tilt (rad); tail tilt kept 0 (hover) here
        airspeed: for the blends
        returns u9 = [thr_L, thr_R, thr_T, tiltL, tiltR, tiltT, ail, el, rud]
        """
        Mx, My, Mz = M_cmd
        w = self._ramp(airspeed, *self.blend_v)       # 0 hover -> 1 cruise
        tail_live = 1.0 - self._ramp(airspeed, *self.tail_fade_v)

        # ---- motor share of moments ----
        # roll keeps half its authority on the motors through the blend:
        # ailerons are weakest exactly where the slipstream wash disturbs
        # roll the most (mid-transition). Once the blend is complete the
        # motors drop out of roll entirely: with low collective a thrust
        # differential can only ADD thrust (one-sided), which was enough to
        # cancel cruise drag and prevented deceleration in back-transition.
        w_roll = 0.5 * w if w < 0.95 else 1.0
        Mx_m, My_m, Mz_m = (1 - w_roll) * Mx, (1 - w) * My, (1 - w) * Mz

        # pitch + collective: solve T_t and wing pair from F and My
        # My_m = -x_m*(T_L+T_R)*cos(tilt) + x_t*T_t   (vertical components)
        ct = max(np.cos(tilt_cmd), 0.10)
        if tail_live > 1e-3:
            T_t = (My_m + self.x_m * F_total * ct) / (self.x_t + self.x_m * ct)
            T_t = np.clip(T_t, 0.0, self.T_max) * tail_live
        else:
            T_t = 0.0
        T_pair = max(F_total - T_t, 0.0)

        # roll: differential on the wing pair (vertical lever y_m*cos tilt)
        dT = Mx_m / max(2 * self.y_m * ct, 0.05)
        T_L = np.clip(T_pair / 2 + dT, 0.0, self.T_max)
        T_R = np.clip(T_pair / 2 - dT, 0.0, self.T_max)

        # yaw: differential tilt increments + tail reaction feedforward
        if tail_reaction_ff and T_t > 0.01:
            # tail spin +1 => reaction -z (nose right); add nose-left moment
            Mz_m += self.kq_kt * T_t
        # decouple roll->yaw: at tilt, the roll differential dT also tips
        # thrust forward differentially, creating M_z = -2*y*sin(tilt)*dT;
        # feed the opposite demand into the yaw channel so roll commands do
        # not excite the (slow) yaw axis
        Mz_m += 2 * self.y_m * np.sin(tilt_cmd) * dT
        T_w_mean = max((T_L + T_R) / 2, 0.5)
        d_tilt = Mz_m / max(2 * self.y_m * T_w_mean, 0.5)
        d_tilt = np.clip(d_tilt, -0.12, 0.12)
        # +Mz (nose left): left motor tilts BACK (-d), right motor FWD (+d)
        tilt_L = np.clip(tilt_cmd - d_tilt, 0.0, self.tilt_max)
        tilt_R = np.clip(tilt_cmd + d_tilt, 0.0, self.tilt_max)

        # ---- surface share of moments (w), authority ~ q_dyn ----
        q_dyn = max(0.5 * self.rho * airspeed**2, 5.0)
        ail = w_roll * Mx / (q_dyn * self.K_ail)
        elev = w * (-My) / (q_dyn * self.K_elev)   # +elev = nose up = -My
        rud = w * Mz / (q_dyn * self.K_rud)
        surf = np.clip(np.array([ail, elev, rud]) / self.surf_max, -1, 1)

        # axial inflow at the props ~ airspeed component along the tilted axis
        v_ax_w = airspeed * np.sin(tilt_cmd)
        u = np.array([
            self.thrust_to_throttle(T_L, v_scale, v_ax_w),
            self.thrust_to_throttle(T_R, v_scale, v_ax_w),
            self.thrust_to_throttle(T_t, v_scale, 0.0),
            tilt_L / self.tilt_max,
            tilt_R / self.tilt_max,
            0.0,                                   # tail tilt (reserved)
            surf[0], surf[1], surf[2],
        ])
        return u
