"""Baseline flight controller: cascaded PID + mixer + transition scheduler.

Architecture (viva):
  position (P) -> velocity (PI) -> acceleration demand
  acceleration -> attitude setpoint + total-thrust demand (hover geometry)
  attitude (quaternion P) -> body-rate setpoint
  rate (PI, gains = I * bandwidth) -> body moment demand
  Mixer allocates moments/thrust to motors (hover) and surfaces (cruise),
  blended by airspeed; wing tilt follows an airspeed schedule, never time,
  so the transition cannot run ahead of the aerodynamics.

Transition physics: as tilt tau grows, vertical motor support falls as
cos(tau) while wing lift grows as V^2. The controller feeds forward an
analytic wing-lift estimate and divides the remaining weight by cos(tau);
a small scheduled pitch-up (0->5 deg with tilt) keeps the lift share high
enough that the thrust demand stays feasible through the corridor pinch.

Modes: TAKEOFF -> HOVER -> TRANSITION -> CRUISE -> BACK -> HOVER2 -> LAND -> DONE
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from params import load_params  # noqa: E402
from control.pid import (PID, quat_to_euler, euler_to_quat,  # noqa: E402
                         attitude_rate_cmd)
from control.mixer import Mixer  # noqa: E402

G = 9.81


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class BaselineController:
    def __init__(self, params=None, mission=None):
        self.p = params if params is not None else load_params()
        c = self.p["control"]
        self.c = c
        self.m = self.p["mass"]["total"]
        self.I = np.array([self.p["inertia"]["Ixx"],
                           self.p["inertia"]["Iyy"],
                           self.p["inertia"]["Izz"]])
        a = self.p["aero"]
        self.S = self.p["geometry"]["wing_area"]
        self.rho = self.p["simulation"]["air_density"]
        self.CL0 = a["CL0"]
        self.CLa = a["CL_alpha"]
        self.i_w = np.deg2rad(a["wing_incidence_deg"])

        self.mixer = Mixer(self.p)
        # yaw acts through slow tilt servos (120 deg/s): give it ~1/3 the
        # bandwidth of roll/pitch or it oscillates the nacelles
        bw = np.array([c["rate_bw"], c["rate_bw"], c["rate_bw"] * 0.35])
        self.kp_rate = bw * self.I                    # M = I*bw*(w_cmd-w)
        self.ki_rate = c["rate_ki"] * self.I * np.array([1, 1, 0.3])
        self.kp_att = c["att_bw"]
        self.kp_vel = c["vel_bw"]
        self.ki_vel = c["vel_ki"]
        self.kp_pos = c["pos_bw"]

        self.thr_pid = PID(c["cruise_thr_kp"], c["cruise_thr_ki"],
                           out_limit=0.9)
        self.rate_i = np.zeros(3)
        self.vel_i = np.zeros(3)

        # default mission: climb, hover, out-and-back cruise triangle-less leg
        self.mission = mission or dict(
            home=np.array([0.0, 0.0, 15.0]),
            cruise_wp=np.array([500.0, 0.0, 15.0]),
            land=np.array([0.0, 0.0, 0.0]),
        )
        self.reset()

    def reset(self):
        self.mode = "TAKEOFF"
        self.timer = 0.0
        self.rate_i[:] = 0
        self.vel_i[:] = 0
        self.thr_pid.reset()
        self.yaw_cmd = 0.0
        self.log_mode = []

    # ---------------- helpers ---------------- #
    def _lift_est(self, V, pitch):
        alpha_w = pitch + self.i_w
        cl = np.clip(self.CL0 + self.CLa * alpha_w, -0.2, 1.2)
        return 0.5 * self.rho * V * V * self.S * cl

    def _tilt_schedule(self, V):
        lo, hi = self.c["tilt_schedule_v"]
        return np.deg2rad(90.0) * np.clip((V - lo) / (hi - lo), 0.0, 1.0)

    # ---------------- main ---------------- #
    def update(self, s: dict, dt: float) -> np.ndarray:
        """s: Vehicle.state() dict. Returns u9."""
        pos, vel = s["pos"], s["vel_w"]
        quat = s["quat"]
        omega = s["omega_b"]
        # schedule on the FORWARD air component only: total airspeed includes
        # sink rate, which creates a dive feedback loop (dive -> V up -> more
        # tilt & less thrust -> deeper dive)
        V = max(float(s["v_air_b"][0]), 0.0)
        roll, pitch, yaw = quat_to_euler(quat)
        self.timer += dt
        msn = self.mission

        # ---------- mode machine ----------
        if self.mode == "TAKEOFF":
            p_cmd = msn["home"].copy()
            v_ff = np.array([0, 0, self.c["climb_rate"]])
            tilt = 0.0
            if pos[2] > msn["home"][2] - 0.5:
                self.mode, self.timer = "HOVER", 0.0
        elif self.mode == "HOVER":
            p_cmd = msn["home"].copy()
            v_ff = np.zeros(3)
            tilt = 0.0
            if self.timer > 3.0:
                self.mode, self.timer = "TRANSITION", 0.0
        elif self.mode == "TRANSITION":
            # push forward; altitude held by the hover cascade.
            # sink guard: if we are losing altitude, stop advancing the
            # speed demand and let thrust recover the height first
            p_cmd = np.array([pos[0] + 30.0, msn["home"][1], msn["home"][2]])
            sink = msn["home"][2] - pos[2]
            if sink > 1.5:
                if getattr(self, "_vff_hold", None) is None:
                    self._vff_hold = V
                v_fwd = min(self._vff_hold, V + 1.0)
            else:
                self._vff_hold = None
                v_fwd = min(4.0 + 1.2 * self.timer, 16.0)
            v_ff = np.array([v_fwd, 0, 0])
            # nacelle tilt LEADS the airspeed: a tilt-rotor accelerates by
            # tilting, so a time ramp provides the initial forward thrust
            # component; the airspeed schedule takes over once V > 6 m/s.
            # The ramp pauses while the sink guard is active.
            if not hasattr(self, "_tilt_ramp"):
                self._tilt_ramp = 0.0
            if sink <= 1.5:
                self._tilt_ramp = min(self._tilt_ramp
                                      + np.deg2rad(6.0) * dt, np.deg2rad(40.0))
            tilt = max(self._tilt_schedule(V), self._tilt_ramp)
            if V >= 15.5 and abs(vel[2]) < 2.0:
                self.mode, self.timer = "CRUISE", 0.0
        elif self.mode == "CRUISE":
            p_cmd = msn["cruise_wp"].copy()
            v_ff = np.zeros(3)
            tilt = np.deg2rad(90.0)
            dist = np.linalg.norm((msn["cruise_wp"] - pos)[:2])
            if dist < 140.0:
                self.mode, self.timer = "BACK", 0.0
        elif self.mode == "BACK":
            p_cmd = np.array([msn["cruise_wp"][0], msn["cruise_wp"][1],
                              msn["home"][2]])
            v_ff = np.zeros(3)
            # tilt back on a TIME ramp (mirror of the forward ramp): waiting
            # for the airspeed to drop first deadlocks, because at high tilt
            # any thrust the altitude loop demands points forward and keeps
            # the speed up. Verticalising the nacelles both kills the forward
            # push and restores vertical support while the nose-up bleeds V.
            ramp_down = np.deg2rad(90.0) - np.deg2rad(5.0) * self.timer
            tilt = min(self._tilt_schedule(V), max(ramp_down, 0.0))
            # gate on GROUND speed: with a tailwind the airspeed reads low
            # while still moving fast over the ground (31 m landing miss in
            # the 5 m/s wind test before this change)
            if np.linalg.norm(vel[0:2]) <= 3.0:
                self.mode, self.timer = "HOVER2", 0.0
                self.hover2_pt = pos.copy()
                self.hover2_pt[2] = msn["home"][2]
        elif self.mode == "HOVER2":
            p_cmd = self.hover2_pt
            v_ff = np.zeros(3)
            tilt = 0.0
            # descend only once actually stopped, or the residual speed
            # drifts the touchdown point (timeout so gusts cannot hold the
            # state machine hostage)
            if (self.timer > 2.0 and np.linalg.norm(vel) < 1.0) \
                    or self.timer > 10.0:
                self.mode, self.timer = "LAND", 0.0
                self.land_pt = pos.copy()
        elif self.mode == "LAND":
            p_cmd = np.array([self.land_pt[0], self.land_pt[1], 0.0])
            v_ff = np.array([0, 0, -self.c["descent_rate"]])
            tilt = 0.0
            if pos[2] < 0.20 and abs(vel[2]) < 0.4:
                self.mode = "DONE"
        else:  # DONE
            return np.zeros(9)

        self.log_mode.append(self.mode)
        blend = np.clip((V - self.c["mixer_blend_v"][0])
                        / (self.c["mixer_blend_v"][1] - self.c["mixer_blend_v"][0]),
                        0.0, 1.0)

        # ---------- outer loops ----------
        if self.mode == "CRUISE":
            u = self._cruise_update(s, p_cmd, dt)
        else:
            u = self._hover_update(s, p_cmd, v_ff, tilt, dt, blend)
        return u

    # ---------------- hover / transition cascade ---------------- #
    def _hover_update(self, s, p_cmd, v_ff, tilt, dt, blend):
        pos, vel, quat, omega = s["pos"], s["vel_w"], s["quat"], s["omega_b"]
        V = max(float(s["v_air_b"][0]), 0.0)
        roll, pitch, yaw = quat_to_euler(quat)
        c = self.c

        v_cmd = self.kp_pos * (p_cmd - pos) + v_ff
        v_cmd[0:2] = np.clip(v_cmd[0:2], -c["max_vel_xy"], c["max_vel_xy"])
        # transition: let the feed-forward dominate forward speed
        if self.mode == "TRANSITION":
            v_cmd[0] = v_ff[0]
        v_cmd[2] = np.clip(v_cmd[2], -c["max_vel_z"], c["max_vel_z"])

        e_v = v_cmd - vel
        self.vel_i += e_v * dt
        self.vel_i = np.clip(self.vel_i, -3.0, 3.0)
        a_cmd = self.kp_vel * e_v + self.ki_vel * self.vel_i

        # attitude setpoint from horizontal acceleration (yaw-aligned frame)
        cy, sy = np.cos(yaw), np.sin(yaw)
        a_xb = cy * a_cmd[0] + sy * a_cmd[1]
        a_yb = -sy * a_cmd[0] + cy * a_cmd[1]

        tilt_frac = tilt / np.deg2rad(90.0)
        if self.mode == "TRANSITION":
            # forward accel comes from the tilt; small scheduled pitch-up
            pitch_cmd = np.deg2rad(5.0) * tilt_frac
        elif self.mode == "BACK":
            # nose-up bleeds speed, but only once slow enough that the extra
            # lift cannot balloon the altitude (at 17 m/s even 6 deg nose-up
            # makes ~1.6x weight of lift); as the nacelles verticalise the
            # normal acceleration law takes over so the position loop can
            # actually stop the aircraft.
            speed_bleed = np.clip((16.0 - V) / 8.0, 0.0, 1.0)
            # the accel-law pitch assumes hover physics (pitch redirects
            # thrust); above ~8 m/s it mainly adds LIFT and balloons the
            # altitude, so it only takes over once genuinely slow
            accel_pitch = np.clip(-a_xb / G, -c["max_tilt_angle_cmd"],
                                  c["max_tilt_angle_cmd"])
            accel_gate = np.clip((8.0 - V) / 4.0, 0.0, 1.0)
            # aero trim: while fast the wing must fly at ~2.5 deg or it sinks
            # (level pitch at 17 m/s gives CL ~ 0.3, only 60 % of weight)
            trim_pitch = np.deg2rad(2.5) * np.clip((V - 8.0) / 4.0, 0.0, 1.0)
            pitch_cmd = (tilt_frac * np.deg2rad(8.0) * speed_bleed + trim_pitch
                         + (1 - tilt_frac) * accel_pitch * accel_gate)
        else:
            pitch_cmd = np.clip(-a_xb / G, -c["max_tilt_angle_cmd"],
                                c["max_tilt_angle_cmd"])
        roll_cmd = np.clip(-a_yb / G, -c["max_tilt_angle_cmd"],
                           c["max_tilt_angle_cmd"])
        if self.mode in ("TRANSITION", "BACK"):
            # mid-transition: weathervane the yaw with the velocity vector
            # (the fins want it anyway; fighting them rolls the aircraft)
            # and keep roll demands small
            if np.linalg.norm(vel[0:2]) > 3.0:
                self.yaw_cmd = np.arctan2(vel[1], vel[0])
            roll_cmd = np.clip(roll_cmd, -np.deg2rad(10), np.deg2rad(10))

        q_des = euler_to_quat(roll_cmd, pitch_cmd, self.yaw_cmd)
        w_cmd = attitude_rate_cmd(quat, q_des, self.kp_att)
        w_cmd[2] *= 0.4    # yaw attitude loop softened (slow tilt servos)
        M = self._rate_loop(w_cmd, omega, dt)

        # total thrust: weight + vertical accel minus wing-lift feedforward,
        # divided by the vertical thrust fraction cos(tilt).
        # The lift estimate uses the MEASURED aerodynamic alpha, not the pitch
        # attitude: during a climb alpha < pitch, and overestimating lift here
        # starved the motors of thrust (and the controller of its differential
        # thrust authority) mid-transition.
        v_air = s["v_air_b"]
        alpha_air = np.arctan2(-v_air[2], max(v_air[0], 0.5))
        alpha_air = np.clip(alpha_air, np.deg2rad(-5), np.deg2rad(12))
        # lift FF is essential through the transition corridor, but in pure
        # hover modes a gusty headwind makes it fluctuate by tens of newtons;
        # there the velocity-loop integrator handles the small real wing lift
        # far more calmly, so the FF stays off outside TRANSITION/BACK
        ff_scale = 1.0 if self.mode in ("TRANSITION", "BACK") \
            else min(2.0 * tilt_frac, 1.0)
        lift = self._lift_est(V, alpha_air) * ff_scale
        f_vert = self.m * (G + a_cmd[2]) - lift
        ct = max(np.cos(tilt), 0.25)
        F_total = np.clip(f_vert / ct, 0.0, 3 * self.mixer.T_max)
        if self.mode == "TRANSITION" or (self.mode == "BACK" and V < 8.0):
            # keep enough collective for attitude authority (differential
            # thrust and tail pitch leverage both scale with thrust) -- but
            # drop the floor when above target altitude or it sustains a
            # climb, and never floor at high tilt in BACK (the thrust would
            # point forward and fight the deceleration)
            floor = 15.0 if pos[2] < p_cmd[2] + 1.5 else 8.0
            F_total = max(F_total, floor)
        # during BACK at low speed the schedule returns thrust support
        if self.mode == "LAND" and s["pos"][2] < 0.35:
            F_total *= 0.85   # soften touchdown

        return self.mixer.mix(F_total, M, tilt, V, v_scale=s["v_batt"] / 22.2)

    # ---------------- cruise loops ---------------- #
    def _cruise_update(self, s, wp, dt, decel=False):
        pos, vel, quat, omega = s["pos"], s["vel_w"], s["quat"], s["omega_b"]
        V = max(float(s["v_air_b"][0]), 0.0)
        roll, pitch, yaw = quat_to_euler(quat)
        c = self.c

        # heading to waypoint. Bank-to-turn sign in FLU: +roll = left wing up
        # = bank RIGHT = yaw rate NEGATIVE, so the gain is negative.
        d = wp - pos
        hdg = np.arctan2(d[1], d[0])
        roll_cmd = np.clip(-c["heading_kp"] * wrap_pi(hdg - yaw),
                           -c["cruise_roll_max"], c["cruise_roll_max"])
        self.yaw_cmd = yaw  # yaw follows naturally; rudder damps

        # altitude -> pitch
        e_h = wp[2] - pos[2]
        pitch_cmd = np.clip(c["cruise_alt_kp"] * e_h
                            - c["cruise_alt_vz_kp"] * vel[2]
                            + np.deg2rad(2.0),          # trim alpha
                            -c["cruise_pitch_max"], c["cruise_pitch_max"])

        # airspeed -> wing throttle (thrust is forward at tilt 90)
        V_cmd = 6.0 if decel else c["cruise_speed_cmd"]
        thr = self.thr_pid(V_cmd - V, dt)
        F_total = np.clip(thr, 0.0, 1.0) * 2 * self.mixer.T_max \
            if not decel else 0.0

        q_des = euler_to_quat(roll_cmd, pitch_cmd, yaw)
        w_cmd = attitude_rate_cmd(quat, q_des, self.kp_att)
        M = self._rate_loop(w_cmd, omega, dt)

        tilt = self._tilt_schedule(V) if decel else np.deg2rad(90.0)
        return self.mixer.mix(F_total, M, tilt, V,
                              v_scale=s["v_batt"] / 22.2)

    def _rate_loop(self, w_cmd, omega, dt):
        e = w_cmd - omega
        self.rate_i += e * dt
        self.rate_i = np.clip(self.rate_i, -1.0, 1.0)
        return self.kp_rate * e + self.ki_rate * self.rate_i
