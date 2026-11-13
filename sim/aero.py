"""Aerodynamics: component buildup / strip model.

Frames: body FLU (x fwd, y left, z up). All forces returned as
(force_world, torque-free application point) pairs so the caller applies them
with mj_applyFT and moments emerge from geometry — no hand-coded moment arms.

Wing: 6 spanwise stations (inner, washed, outer per side). Each station sees
  v_local = v_air_body + omega x r + slipstream (washed stations only)
which naturally produces roll/pitch/yaw damping and asymmetric stall.

Lift curve with smooth post-stall blend (Beard & McLain):
  sigma(a) blends linear CL to flat-plate 2*sign(a)*sin^2(a)*cos(a)
Drag: station profile CD + induced CL^2/(pi*e*AR); fuselage parasite drag
(the remainder of aircraft CD0) + notch increment applied at the CG.

Control surfaces are read from the MJCF hinge angles (physical sign):
  aileron hinge + = TE up  -> local lift decrease on that side
  elevator hinge + = TE up -> tail lift decrease -> nose up
  rudder hinge +  = TE right -> fin force +y (left) -> nose right
Slipstream washing: the strip behind each wing prop sees extra axial flow
v_s = decay * v_induced along the thrust axis; the washed fraction grows with
tilt (disc ahead of the LE in hover barely touches the wing; at 90 deg the
slipstream is chordwise): frac = 0.1 + 0.9*sin(tilt).
"""
from __future__ import annotations

import numpy as np

EPS = 1e-6


class WingAero:
    def __init__(self, p: dict):
        a = p["aero"]
        g = p["geometry"]
        lay = p["layout"]
        self.rho = p["simulation"]["air_density"]
        self.CL0 = a["CL0"]
        self.CLa = a["CL_alpha"]
        self.e = a["oswald_e"]
        self.AR = g["wing_aspect_ratio"]
        self.i_w = np.deg2rad(a["wing_incidence_deg"])
        self.a_stall = np.deg2rad(a["alpha_stall_deg"])
        self.M = a["stall_blend_M"]
        self.Cm_ac = a["Cm_ac"]
        self.chord = g["wing_chord"]
        self.CD0_wing = 0.02   # wing profile share of aircraft CD0
        self.CD0_other = p["aero"]["CD0"] - self.CD0_wing  # fuselage/booms/interference
        self.notch_dCD = a["notch_open_dCD"]
        self.S = g["wing_area"]
        self.ail_tau = a["aileron_tau"]

        # 6 spanwise stations: per side [inner, washed, outer]
        # span segments per side: 0..0.248, 0.248..0.552 (prop-washed), 0.552..0.80
        half = g["wing_span"] / 2
        seg = [(0.0, 0.248), (0.248, 0.552), (0.552, half)]
        x_ac = g["wing_le_x"] - 0.25 * self.chord
        z = lay["wing_z"]
        self.stations = []  # (pos_body, area, washed, aileron)
        for sgn in (+1, -1):
            for (y0, y1), tag in zip(seg, ("inner", "washed", "outer")):
                yc = sgn * 0.5 * (y0 + y1)
                area = (y1 - y0) * self.chord
                self.stations.append({
                    "r": np.array([x_ac, yc, z]),
                    "area": area,
                    "washed": tag == "washed",
                    "aileron": tag == "outer",
                    "side": sgn,   # +1 left, -1 right
                })

        # Tail (stab stubs, both sides lumped): AC at quarter chord
        self.S_tail = 2 * g["stabilizer_area_each"]
        self.r_tail = np.array([-0.50, 0.0, 0.02])
        self.CLa_t = a["CLa_tail"]
        self.deps = a["tail_downwash"]
        self.tau_e = a["elevator_tau"]

        # Fins (lumped)
        self.S_fin = 2 * g["fin_area_each"]
        self.r_fin = np.array([-0.54, 0.0, 0.10])
        self.CYb = abs(a["CYb_fin"])
        self.tau_r = a["rudder_tau"]

    # ---- lift/drag primitives ----
    # Post-stall blend: fully linear up to alpha_stall (so CL_max is actually
    # reached), then a smoothstep ramp to the flat-plate curve over ~6 deg.
    # (A centered sigmoid like Beard-McLain halves CL at alpha_stall, which
    # understates CL_max and pushed the stall speed to 13.8 m/s in testing.)
    _RAMP = np.deg2rad(6.0)

    def _sigma(self, a):
        t = (abs(a) - self.a_stall) / self._RAMP
        t = np.clip(t, 0.0, 1.0)
        return t * t * (3 - 2 * t)          # smoothstep

    def cl(self, alpha, dCL=0.0):
        lin = self.CL0 + self.CLa * alpha + dCL
        fp = 2.0 * np.sign(alpha) * np.sin(alpha)**2 * np.cos(alpha)
        s = self._sigma(alpha)
        return (1 - s) * lin + s * fp

    def cd(self, alpha, cl):
        lin = self.CD0_wing + cl**2 / (np.pi * self.e * self.AR)
        fp = 2.0 * np.sin(alpha)**2
        s = self._sigma(alpha)
        return (1 - s) * lin + s * fp

    # ---- main entry ----
    def forces(self, v_air_b, omega_b, hinge, slip, notch_open=False):
        """Compute aero forces in BODY frame.

        v_air_b: velocity of the vehicle relative to the air, body frame (3,)
        omega_b: body rates (3,)
        hinge: dict with aileron_left/right, elevator, rudder hinge angles (rad)
               elevator/rudder are the mean of their pairs
        slip: list of (side, v_slip, axis_b) wing-prop slipstream entries,
              side +1 left / -1 right, axis_b = thrust direction in body frame
        returns list of (F_body(3,), point_body(3,)) and Cm_ac torque (3,)
        """
        out = []
        q_dyn_ref = 0.5 * self.rho * max(np.dot(v_air_b, v_air_b), EPS)

        slip_map = {s[0]: (s[1], s[2]) for s in slip}

        for st in self.stations:
            v_loc = v_air_b + np.cross(omega_b, st["r"])
            if st["washed"] and st["side"] in slip_map:
                vs, axis_b = slip_map[st["side"]]
                v_loc = v_loc + vs * axis_b
            u, w = v_loc[0], v_loc[2]
            V2 = u * u + w * w
            if V2 < EPS:
                continue
            alpha = np.arctan2(-w, u) + self.i_w
            dCL = 0.0
            if st["aileron"]:
                ail = hinge["aileron_left"] if st["side"] > 0 else hinge["aileron_right"]
                dCL = -self.CLa * self.ail_tau * ail   # hinge + = TE up = less lift
            cl = self.cl(alpha, dCL)
            cd = self.cd(alpha, cl)
            q = 0.5 * self.rho * V2 * st["area"]
            # wind-axis unit vectors in the body x-z plane
            Vxz = np.sqrt(V2)
            ex = np.array([u / Vxz, 0, w / Vxz])       # along local flow
            ez = np.array([-w / Vxz, 0, u / Vxz])      # perpendicular, "lift up"
            F = q * (cl * ez - cd * ex)
            out.append((F, st["r"]))

        # tail with downwash and elevator
        v_t = v_air_b + np.cross(omega_b, self.r_tail)
        u, w = v_t[0], v_t[2]
        V2t = u * u + w * w
        if V2t > EPS:
            alpha_w = np.arctan2(-v_air_b[2], max(v_air_b[0], EPS)) + self.i_w
            alpha_t = np.arctan2(-w, u) - self.deps * alpha_w
            cl_t = self.CLa_t * alpha_t - self.CLa_t * self.tau_e * hinge["elevator"]
            cl_t = np.clip(cl_t, -1.2, 1.2)
            cd_t = 0.01 + cl_t**2 / (np.pi * 0.7 * 2.0)
            q = 0.5 * self.rho * V2t * self.S_tail
            Vxz = np.sqrt(V2t)
            ex = np.array([u / Vxz, 0, w / Vxz])
            ez = np.array([-w / Vxz, 0, u / Vxz])
            out.append((q * (cl_t * ez - cd_t * ex), self.r_tail))

        # fins: side force from sideslip + rudder, yaw damping via omega x r
        v_f = v_air_b + np.cross(omega_b, self.r_fin)
        u, v = v_f[0], v_f[1]
        V2f = u * u + v * v
        if V2f > EPS:
            beta = np.arctan2(-v, abs(u) + EPS)
            cy = self.CYb * beta + self.CYb * self.tau_r * hinge["rudder"]
            cy = np.clip(cy, -1.5, 1.5)
            q = 0.5 * self.rho * V2f * self.S_fin
            out.append((np.array([0.0, q * cy, 0.0]), self.r_fin))

        # fuselage parasite drag (+ notch increment) at the CG, opposing motion
        V = np.linalg.norm(v_air_b)
        if V > EPS:
            cd0 = self.CD0_other + (self.notch_dCD if notch_open else 0.0)
            F_fus = -0.5 * self.rho * V * v_air_b * cd0 * self.S
            out.append((F_fus, np.zeros(3)))

        # wing camber pitching moment: aero convention has +Cm = nose-up, and
        # in FLU nose-up is NEGATIVE M_y, so the body torque is -q*S*c*Cm.
        # (Cm_ac = -0.05 therefore gives the physical nose-DOWN camber moment.)
        tq = np.array([0.0, -q_dyn_ref * self.S * self.chord * self.Cm_ac, 0.0])
        return out, tq
